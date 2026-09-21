/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "mpcController.h"

#include <fstream>
#include <iomanip>

MpcController::MpcController(const solverConfig& config, std::unique_ptr<SolverBackend> backend)
    : m_config(config)
    , m_backend(std::move(backend))
    , m_iw(m_backend->workIntSize())
    , m_w(m_backend->workRealSize())
{
    m_refStride = m_config.nx + m_config.nu;

    m_initialStates.resize(m_config.nx);
    m_uPrev.assign(m_config.nu, 0.0);

    m_initializeSolverIO();
    m_packBounds();

    // Fixed 8-in/6-out nlpsol layout -- see SolverBackend.h.
    m_arg.resize(8);
    m_res.resize(6);
    m_bindSolverIO();
}

MpcController::~MpcController() = default;

void MpcController::initLaunch() {
    m_launched = true;
    m_timeAtLaunched = std::chrono::steady_clock::now();

    Logger::instance().log(LogType::NMPC_EVENT,
        std::to_string(m_trackingNumber) + "," + std::to_string(Logger::instance().nowMilliseconds()) + ","
        + std::to_string(Logger::nowWallTimeMs()) + ",LAUNCH triggered");
}

void MpcController::loadTrajectory(const std::string& file) {

    std::ifstream fileStream(file);
    if (!fileStream.is_open())
        throw std::runtime_error("Cannot open trajectory file");

    m_referenceTrajectory.clear();

    std::string line;
    while (std::getline(fileStream, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string field;

        // first nx fields : states
        for (int i = 0; i < m_refStride; i++) {
            std::getline(ss, field, ',');
            m_referenceTrajectory.push_back(std::stod(field));
        }
    }

    m_onReferenceTrajectoryChanged();
    LOG_INFO("Trajectory loaded from file, number of points = " + std::to_string(m_numTrajectoryPoints));
}

void MpcController::saveTrajectory(const std::string& file) const {
    std::lock_guard lock(m_solveMutex); // same guard getTrajectoryForVehicle()/getDebugInfo() use -- may run while solve() is active

    if (m_referenceTrajectory.empty())
        throw std::runtime_error("saveTrajectory: no trajectory loaded/generated yet");

    std::ofstream fileStream(file);
    if (!fileStream.is_open())
        throw std::runtime_error("saveTrajectory: cannot open file for writing: " + file);

    // Full double round-trip precision, so the file std::stod's back in
    // loadTrajectory() to bit-for-bit (or near enough) the same values.
    fileStream << std::setprecision(17);

    for (size_t row = 0; row < m_numTrajectoryPoints; ++row) {
        const size_t rowStart = row * m_refStride;
        for (size_t i = 0; i < m_refStride; ++i) {
            if (i > 0) fileStream << ',';
            fileStream << m_referenceTrajectory[rowStart + i];
        }
        fileStream << '\n';
    }

    if (!fileStream.good())
        throw std::runtime_error("saveTrajectory: write failed (disk full?): " + file);

    LOG_INFO("Trajectory saved to file, number of points = " + std::to_string(m_numTrajectoryPoints));
}

void MpcController::setReferenceTrajectory(std::vector<double> referenceTrajectory) {
    std::lock_guard lock(m_solveMutex); // same guard getTrajectoryForVehicle()/getDebugInfo() use

    if (referenceTrajectory.size() % m_refStride != 0) {
        throw std::runtime_error("setReferenceTrajectory: size (" + std::to_string(referenceTrajectory.size()) +
            ") is not a multiple of nx+nu (" + std::to_string(m_refStride) + ") -- generator/solver layout mismatch");
    }

    m_referenceTrajectory = std::move(referenceTrajectory);
    m_onReferenceTrajectoryChanged();
    LOG_INFO("Trajectory generated in-process, number of points = " + std::to_string(m_numTrajectoryPoints));
}

void MpcController::m_onReferenceTrajectoryChanged() {
    m_numTrajectoryPoints = m_referenceTrajectory.size() / m_refStride;
    m_endIdxTraj = m_numTrajectoryPoints > m_config.N ? m_numTrajectoryPoints - m_config.N : 0;

    std::ostringstream msg;
    msg << m_trackingNumber << "," << Logger::instance().nowMilliseconds() << "," << Logger::nowWallTimeMs()
        << ",TRAJECTORY loaded, points=" << m_numTrajectoryPoints
        << ", numUavs=" << m_config.numUavs
        << ", hasPayload=" << (hasPayload() ? "true" : "false")
        << ", N=" << m_config.N;
    Logger::instance().log(LogType::NMPC_EVENT, msg.str());
}

std::map<uint8_t, uavCommandsFlags> MpcController::solve(const std::map<uint8_t, uavStates>& latestStates) {

    std::lock_guard lock(m_solveMutex);

    m_unpackLatestStates(latestStates, m_initialStates);

    // Shift solution or pack initial guess
    if (m_lastSolveMs <= 0.0) {
        m_packInitialGuess();
    } else {
        m_shiftSolution();
    }

    if (m_launched) {
        size_t idx = m_lastIdxTraj;
        double bestCost = m_computeReferenceCost(idx);

        // only move forward
        while (idx + 1 < m_endIdxTraj) {
            const double nextCost = m_computeReferenceCost(idx + 1);

            // stop once cost increases
            if (nextCost > bestCost)
                break;

            bestCost = nextCost;
            ++idx;
        }
        m_pendingSteps = idx - m_lastIdxTraj;
        m_lastIdxTraj = idx;

        if (m_endIdxTraj == 0 || idx +1 >= m_endIdxTraj) {
            m_endedTraj = true;
        }
    }

    m_packParameters();
    m_bindSolverIO();


    // Solve
    {
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs, false);
        const int flag = m_backend->solve(m_arg.data(), m_res.data(), m_iw.data(), m_w.data());
        m_lastFlag = flag;

        const auto converged = m_solutionIsValid(flag);
        (void)converged;
    }

    // Extract and return u0 for each UAV
    auto controls = m_extractControls();

    // Remember the just-applied first-stage control (physical units, UAV
    // 1's block -- U_prev in the one-UAV NLP has no per-UAV structure to
    // worry about) for next solve's dU0 rate-penalty parameter. Skipped on
    // a violation: m_x may not hold a meaningful solution then, and
    // m_extractControls() already fell back to the reference trajectory's
    // planned control for the returned command in that case.
    if (!m_violation) {
        const int offset = m_config.nx;
        for (int i = 0; i < m_config.nu; ++i) {
            m_uPrev[i] = m_x[offset + i] * m_config.scalesControls[i];
        }
    }

    m_logTransitions();

    m_trackingNumber += 1;
    return controls;
}

void MpcController::m_logTransitions() {
    if (m_inFlight != m_prevInFlight) {
        Logger::instance().log(LogType::NMPC_EVENT,
            std::to_string(m_trackingNumber) + "," + std::to_string(Logger::instance().nowMilliseconds()) + ","
            + std::to_string(Logger::nowWallTimeMs()) + ","
            + (m_inFlight ? "INFLIGHT detected (speed threshold crossed)" : "INFLIGHT cleared"));
        m_prevInFlight = m_inFlight;
    }

    if (m_endedTraj != m_prevEndedTraj) {
        Logger::instance().log(LogType::NMPC_EVENT,
            std::to_string(m_trackingNumber) + "," + std::to_string(Logger::instance().nowMilliseconds()) + ","
            + std::to_string(Logger::nowWallTimeMs()) + ","
            + (m_endedTraj ? "TRAJECTORY ended, idx=" + std::to_string(m_lastIdxTraj) : "TRAJECTORY resumed"));
        m_prevEndedTraj = m_endedTraj;
    }

    // m_violation is recomputed fresh every solve (see m_solutionIsValid),
    // so both directions are meaningful here: entering flags a real solver
    // problem to look into after the test, and clearing tells you exactly
    // how many ticks (trackingNumber delta) it stayed degraded for.
    if (m_violation != m_prevViolation) {
        Logger::instance().log(LogType::NMPC_EVENT,
            std::to_string(m_trackingNumber) + "," + std::to_string(Logger::instance().nowMilliseconds()) + ","
            + std::to_string(Logger::nowWallTimeMs()) + ","
            + (m_violation ? "VIOLATION entered" : "VIOLATION cleared"));
        m_prevViolation = m_violation;
    }
}

double MpcController::lastSolveMs() const {
    return m_lastSolveMs;
}

MpcController::DebugInfo MpcController::getDebugInfo() const {
    std::lock_guard lock(m_solveMutex);

    DebugInfo info;
    info.launched = m_launched;
    info.inFlight = m_inFlight;
    info.endedTraj = m_endedTraj;
    info.violation = m_violation;
    info.lastSolveMs = m_lastSolveMs;
    info.trackingNumber = m_trackingNumber;
    info.trajectoryIndex = m_lastIdxTraj;
    info.trajectoryTotal = m_numTrajectoryPoints;
    info.lastFlag = m_lastFlag;
    info.lastMaxConstraintViolation = m_lastMaxConstraintViolation;
    info.backendName = m_backend->name();
    return info;
}

std::vector<MpcController::TrajectoryPointView> MpcController::getTrajectoryForVehicle(const int vehicleIndex) const {
    std::lock_guard lock(m_solveMutex);  // same guard getDebugInfo() uses

    int offset, blockSize;
    if (vehicleIndex >= 0 && vehicleIndex < m_config.numUavs) {
        offset = vehicleIndex * kUavBlockSize;
        blockSize = kUavBlockSize;
    } else if (hasPayload() && vehicleIndex == m_config.numUavs) {
        offset = kUavBlockSize * m_config.numUavs;
        blockSize = kPayloadBlockSize;
    } else {
        return {};
    }

    std::vector<TrajectoryPointView> points;
    points.reserve(m_numTrajectoryPoints);
    for (size_t i = 0; i < m_numTrajectoryPoints; ++i) {
        const size_t rowStart = i * m_refStride + offset;
        TrajectoryPointView p;
        p.north = m_referenceTrajectory[rowStart + 0];
        p.east  = m_referenceTrajectory[rowStart + 1];
        p.down  = m_referenceTrajectory[rowStart + 2];
        p.vx    = m_referenceTrajectory[rowStart + 3];
        p.vy    = m_referenceTrajectory[rowStart + 4];
        p.vz    = m_referenceTrajectory[rowStart + 5];
        if (blockSize == kUavBlockSize) {
            p.roll  = grs::radToDeg(m_referenceTrajectory[rowStart + 6]);
            p.pitch = grs::radToDeg(m_referenceTrajectory[rowStart + 7]);
        }
        points.push_back(p);
    }
    return points;
}

void MpcController::m_initializeSolverIO() {
    // Inputs
    m_x0.assign(m_backend->inputSize(0), 0.0);
    m_p.assign(m_backend->inputSize(1), 0.0);
    m_lbx.assign(m_backend->inputSize(2), 0.0);
    m_ubx.assign(m_backend->inputSize(3), 0.0);
    m_lbg.assign(m_backend->inputSize(4), 0.0);
    m_ubg.assign(m_backend->inputSize(5), 0.0);
    m_lam_x0.assign(m_backend->inputSize(6), 0.0);
    m_lam_g0.assign(m_backend->inputSize(7), 0.0);

    // Outputs
    m_x.assign(m_backend->outputSize(0), 0.0);
    m_f.assign(m_backend->outputSize(1), 0.0);
    m_g.assign(m_backend->outputSize(2), 0.0);
    m_lam_x.assign(m_backend->outputSize(3), 0.0);
    m_lam_g.assign(m_backend->outputSize(4), 0.0);
    m_lam_p.assign(m_backend->outputSize(5), 0.0);
}

void MpcController::m_bindSolverIO() {
    // Inputs
    // 0: x0, 1: p, 2: lbx, 3: ubx, 4: lbg, 5: ubg, 6: lam_x0, 7: lam_g0
    m_arg[0] = m_x0.data();
    m_arg[1] = m_p.data();
    m_arg[2] = m_lbx.data();
    m_arg[3] = m_ubx.data();
    m_arg[4] = m_lbg.data();
    m_arg[5] = m_ubg.data();
    m_arg[6] = m_lam_x0.data();
    m_arg[7] = m_lam_g0.data();

    // Outputs
    // 0: x, 1: f, 2: g, 3: lam_x, 4: lam_g, 5: lam_p
    m_res[0] = m_x.data();
    m_res[1] = m_f.data();
    m_res[2] = m_g.data();
    m_res[3] = m_lam_x.data();
    m_res[4] = m_lam_g.data();
    m_res[5] = m_lam_p.data();
}

double MpcController::m_computeReferenceCost(const size_t idx) const {
    const size_t refOffset = idx * m_refStride;

    double cost = 0.0;

    const double refNorth = m_referenceTrajectory[refOffset + 0];
    const double refEast = m_referenceTrajectory[refOffset + 1];

    const double dn = m_initialStates[0] - refNorth;
    const double de = m_initialStates[1]  - refEast;

    cost += dn*dn + de*de;

    return cost;
}

void MpcController::m_shiftSolution() {

    const size_t nx     = m_config.nx;
    const size_t nu     = m_config.nu;
    const size_t N      = m_config.N;
    const size_t stride = nx + nu;

    const size_t shift = m_pendingSteps;

    // -------------------------------------------------
    // 1. Shift all complete stages
    // -------------------------------------------------
    for (size_t k = 0; k < N - shift; ++k) {
        const size_t dst = k * stride;
        const size_t src = (k + shift) * stride;

        std::copy_n(m_x.begin() + src, stride, m_x0.begin() + dst);
    }

    // -------------------------------------------------
    // 2. Repeat last available stage
    // -------------------------------------------------
    const size_t lastValidStage = N - shift;

    for (size_t k = N - shift; k < N; ++k) {
        const size_t dst = k * stride;
        const size_t src = lastValidStage * stride;

        std::copy_n(m_x.begin() + src, stride, m_x0.begin() + dst);
    }

    // -------------------------------------------------
    // 3. Copy terminal state x_N
    // -------------------------------------------------
    const size_t xN_src = N * stride;
    const size_t xN_dst = N * stride;

    std::copy_n(m_x.begin() + xN_src, nx, m_x0.begin() + xN_dst);

    // -------------------------------------------------
    // 4. Re-anchor initial state with measurement
    // -------------------------------------------------
    for (size_t i = 0; i < nx; ++i) {
        m_x0[i] = m_initialStates[i] * m_config.invScalesStates[i];
    }

    std::ranges::fill(m_lam_x0, 0.0);
    std::ranges::fill(m_lam_g0, 0.0);
}

void MpcController::m_packBounds() {
    // This NLP's decision-variable bounds tiling is m_backend's concern now
    // -- see solverBackend.h for why packParameters()/packBounds() moved
    // off this class.
    m_backend->packBounds(m_config, m_lbx, m_ubx);

    assert(m_lbx.size() == static_cast<size_t>(m_backend->inputSize(2)));
    assert(m_ubx.size() == static_cast<size_t>(m_backend->inputSize(3)));
}

void MpcController::m_packInitialGuess() {

    // N * (nx+nu) + xN
    // ref: [x0 u0 x1 u1 ... xN uN]
    const size_t count = m_config.N * m_refStride + m_config.nx;
    const size_t offsetRef = m_lastIdxTraj * m_refStride;

    std::memcpy(m_x0.data(), m_referenceTrajectory.data() + offsetRef, count * sizeof(double));

    std::ranges::copy(m_initialStates, m_x0.begin());

    for (size_t k = 0; k < m_config.N + 1; ++k)
    {
        const size_t xOffset = k * m_refStride;
        for (size_t i = 0; i < m_config.nx; ++i) {
            m_x0[xOffset + i] *= m_config.invScalesStates[i];
        }

        const size_t uOffset = k * m_refStride + m_config.nx;
        for (size_t i = 0; i < m_config.nu; ++i) {
            m_x0[uOffset + i] *= m_config.invScalesControls[i];
        }
    }


    assert(m_x0.size() == static_cast<size_t>(m_backend->inputSize(0)));
}

void MpcController::m_packParameters() {
    // This NLP's parameter-vector ordering (wind, weight, U_prev, L0, ...)
    // is m_backend's concern now -- see solverBackend.h for why
    // packParameters()/packBounds() moved off this class.
    const size_t offsetRef = m_lastIdxTraj * m_refStride;
    m_backend->packParameters(m_config, m_initialStates, m_referenceTrajectory, offsetRef, m_uPrev, m_p);

    assert(m_p.size() == static_cast<size_t>(m_backend->inputSize(1)));
}

std::map<uint8_t, uavCommandsFlags> MpcController::m_extractControls() const {

    std::map<uint8_t, uavCommandsFlags> out;

    // Extract u0 for each UAV in order
    for (int sysId = 1; sysId <= m_config.numUavs; ++sysId) {
        uavCommandsFlags cmd;

        const int offset = m_config.nx + (m_config.nu / m_config.numUavs) * (sysId - 1);

        // Controls per UAV are [T, roll, pitch], yaw is always 0
        cmd.commands.sysId = static_cast<uint8_t>(sysId);

        if (m_violation) {
            // Fall back to the planned open-loop control
            const size_t ctrlOffset = m_lastIdxTraj * m_refStride + offset;
            cmd.commands.thrust      = static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 0));
            cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 1)));
            cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 2)));
            cmd.commands.yawDegree   = 0.0;
        } else {
            cmd.commands.thrust      = static_cast<float>(m_x[offset + 0] * m_config.scalesControls[0]);
            cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_x[offset + 1] * m_config.scalesControls[1]));
            cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_x[offset + 2] * m_config.scalesControls[2]));
            cmd.commands.yawDegree   = 0.0;
        }

        cmd.F1Command = true;   // Should move?
        cmd.F2Command = false;  // End simulation?
        cmd.F3Command = false;  // Launch?

        if (m_launched) {
            cmd.F3Command = true;
        }

        if (m_endedTraj) {
            cmd.F2Command = true;
        }

        out[static_cast<uint8_t>(sysId)] = cmd;

        std::ostringstream msg;
        msg << std::fixed << std::setprecision(4) << m_trackingNumber << "," << Logger::instance().nowMilliseconds() << "," << Logger::nowWallTimeMs() << "," << m_lastSolveMs << ",";
        msg << cmd.commands.thrust << "," << cmd.commands.rollDegree << "," << cmd.commands.pitchDegree << "," << cmd.commands.yawDegree << "," << m_lastIdxTraj;

        if (m_violation) {
            msg << ", INVALID SOL";
        }
        Logger::instance().log(LogType::CONTROLS, msg.str());
    }

    return out;
}

bool MpcController::m_solutionIsValid(const int flag) {
    m_violation = false;

    if (flag != 0) {
        // Previously fell through without setting m_violation -- meaning a
        // raw solver failure (as opposed to a feasibility/NaN issue caught
        // below) never triggered m_extractControls()'s fallback to the
        // planned open-loop control, and would have extracted m_x as if it
        // held a valid solution even though the solve itself failed.
        m_violation = true;
        m_lastMaxConstraintViolation = -1.0; // not evaluated -- solver itself failed
        return false;
    }

    constexpr double feas_tol = 5e-4;

    // Check constraints
    double max_violation = 0.0;

    for (size_t i = 0; i < m_g.size(); ++i) {
        double v_low  = m_lbg[i] - m_g[i];
        double v_high = m_g[i] - m_ubg[i];
        double violation = std::max({0.0, v_low, v_high});
        max_violation = std::max(max_violation, violation);
    }

    m_lastMaxConstraintViolation = max_violation;

    if (max_violation > feas_tol) {
        m_violation = true;
        return false;
    }

    // Check decision variables
    for (int i = 0; i < m_config.nx; ++i)
        if (!std::isfinite(m_x[i])) {
            m_violation = true;
            return false;
        }

    // Check objective
    if (!std::isfinite(m_f[0])) {
        m_violation = true;
        return false;
    }

    return true;
}

double MpcController::m_unwrapYaw(const uint8_t sysId, const double yawRadWrapped) {

    auto& s = m_yawStates[sysId];

    if (!s.initialized) {
        s.prev = yawRadWrapped;
        s.unwrapped = yawRadWrapped;
        s.initialized = true;
        return s.unwrapped;
    }

    double delta = yawRadWrapped - s.prev;

    // Wrap delta to [-pi, pi]
    if (delta > M_PI)
        delta -= 2.0 * M_PI;
    else if (delta < -M_PI)
        delta += 2.0 * M_PI;

    s.unwrapped += delta;
    s.prev = yawRadWrapped;

    return s.unwrapped;
}

void MpcController::m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates, std::vector<double>& unpackStates) {

    size_t offset = 0;

    size_t connectedUavs = 0;
    for (const auto& [sysId, states] : latestStates) {
        if (sysId <= m_config.numUavs) {
            ++connectedUavs;
        }
    }
    const bool usePayloadTelemetry = hasPayload() && connectedUavs >= 2;

    for (const auto& [sysId, states] : latestStates) {
        if (sysId <= m_config.numUavs) {

            double speed = std::sqrt(states.northMeterSecond*states.northMeterSecond + states.eastMeterSecond*states.eastMeterSecond + states.downMeterSecond*states.downMeterSecond);

            double north = states.northMeter;
            double east = states.eastMeter;
            double down = states.downMeter;

            double vNorth = states.northMeterSecond;
            double vEast = states.eastMeterSecond;
            double vDown = states.downMeterSecond;

            if (speed > 12.0) {
                m_inFlight = true;
            }

            if (!m_launched || !m_inFlight) {
                const size_t blockOffset = static_cast<size_t>(sysId - 1) * kUavBlockSize;
                north  = m_referenceTrajectory.at(blockOffset + 0);
                east   = m_referenceTrajectory.at(blockOffset + 1);
                down   = m_referenceTrajectory.at(blockOffset + 2);
                vNorth = m_referenceTrajectory.at(blockOffset + 3);
                vEast  = m_referenceTrajectory.at(blockOffset + 4);
                vDown  = m_referenceTrajectory.at(blockOffset + 5);
            }

            unpackStates.at(offset++) = north;
            unpackStates.at(offset++) = east;
            unpackStates.at(offset++) = down;
            unpackStates.at(offset++) = vNorth;
            unpackStates.at(offset++) = vEast;
            unpackStates.at(offset++) = vDown;
            unpackStates.at(offset++) = grs::degToRad(states.rollDegree);
            unpackStates.at(offset++) = grs::degToRad(states.pitchDegree);


        } else if (usePayloadTelemetry) {
            // Payload
            unpackStates.at(offset++) = states.northMeter;
            unpackStates.at(offset++) = states.eastMeter;
            unpackStates.at(offset++) = states.downMeter;
            unpackStates.at(offset++) = states.northMeterSecond;
            unpackStates.at(offset++) = states.eastMeterSecond;
            unpackStates.at(offset++) = states.downMeterSecond;
        }
    }

    assert(offset == unpackStates.size());

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4) << m_trackingNumber << "," << Logger::instance().nowMilliseconds() << "," << Logger::nowWallTimeMs() << ",";
    for (auto&& x : unpackStates) {
        msg << x << ",";
    }
    Logger::instance().log(LogType::STATES, msg.str());
}
