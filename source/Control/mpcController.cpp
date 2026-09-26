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

MpcController::MpcController(const solverConfig& config)
    : m_config(config)
    , m_solver(Nlpsol::Problem::Nmpc, config.numUavs)
{
    m_refStride = m_config.nx + m_config.nu;
    m_layout = {m_config.numUavs, hasPayload()};
    assert(m_layout.size() == static_cast<size_t>(m_config.nx));

    m_initialStates.assign(m_config.nx, 0.0);
    m_uPrev.assign(m_config.nu, 0.0);
    m_windEst.assign(m_config.np, 0.0);
    m_dEst.assign(m_config.nd, 0.0);

    m_packBounds();
    m_packInequalityBounds();
}

MpcController::~MpcController() = default;

void MpcController::setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) {
    std::lock_guard lock(m_disturbanceMutex);
    assert(wind.size() == static_cast<size_t>(m_config.np));
    assert(d.size() == static_cast<size_t>(m_config.nd));
    m_windEst = wind;
    m_dEst = d;
}

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

        for (int i = 0; i < m_refStride; i++) {
            std::getline(ss, field, ',');
            m_referenceTrajectory.push_back(std::stod(field));
        }
    }

    m_onReferenceTrajectoryChanged();
    LOG_INFO("Trajectory loaded from file, number of points = " + std::to_string(m_numTrajectoryPoints));
}

void MpcController::saveTrajectory(const std::string& file) const {
    std::lock_guard lock(m_solveMutex);

    if (m_referenceTrajectory.empty())
        throw std::runtime_error("saveTrajectory: no trajectory loaded/generated yet");

    std::ofstream fileStream(file);
    if (!fileStream.is_open())
        throw std::runtime_error("saveTrajectory: cannot open file for writing: " + file);

    // Full double precision, so loadTrajectory() reads back the same values.
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
    std::lock_guard lock(m_solveMutex);

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

    m_unpackLatestStates(latestStates);

    // Warm start from the shifted previous solution, cold start the first time.
    if (m_lastSolveMs <= 0.0) {
        m_packInitialGuess();
    } else {
        m_shiftSolution();
    }

    if (m_launched) {
        size_t idx = m_lastIdxTraj;

        if (m_config.referenceIndexing == solverConfig::ReferenceIndexing::Time) {
            // One sample per solve, starting at sample 0 at launch.
            if (m_solvesSinceLaunch > 0 && idx + 1 < m_endIdxTraj) {
                ++idx;
            }
            ++m_solvesSinceLaunch;
        } else {
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
        }
        m_pendingSteps = idx - m_lastIdxTraj;
        m_lastIdxTraj = idx;

        if (m_endIdxTraj == 0 || idx +1 >= m_endIdxTraj) {
            m_endedTraj = true;
        }
    }

    m_packParameters();

    {
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs, false);
        const int flag = m_solver.solve();
        m_lastFlag = flag;

        const auto converged = m_solutionIsValid(flag);
        (void)converged;
    }

    auto controls = m_extractControls();

    // U_prev for the next solve: the first-stage control, all UAVs. Kept unchanged when the solution was rejected.
    if (!m_violation) {
        const int offset = m_config.nx;
        for (int i = 0; i < m_config.nu; ++i) {
            m_uPrev[i] = m_solver.x[offset + i] * m_config.scalesControls[i];
        }
    }

    m_logTransitions();

    m_trackingNumber += 1;
    return controls;
}

void MpcController::m_logTransitions() {
    if (m_telemetryComplete != m_prevTelemetryComplete) {
        Logger::instance().log(LogType::NMPC_EVENT,
            std::to_string(m_trackingNumber) + "," + std::to_string(Logger::instance().nowMilliseconds()) + ","
            + std::to_string(Logger::nowWallTimeMs()) + ","
            + (m_telemetryComplete ? "TELEMETRY complete" : "TELEMETRY incomplete, missing vehicles keep their last state"));
        m_prevTelemetryComplete = m_telemetryComplete;
    }

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
    info.backendName = m_solver.name();
    return info;
}

std::vector<MpcController::TrajectoryPointView> MpcController::getTrajectoryForVehicle(const int vehicleIndex) const {
    std::lock_guard lock(m_solveMutex);

    int offset, blockSize;
    if (vehicleIndex >= 0 && vehicleIndex < m_config.numUavs) {
        offset = static_cast<int>(m_layout.uavOffset(vehicleIndex));
        blockSize = grs::control::kUavBlockSize;
    } else if (hasPayload() && vehicleIndex == m_config.numUavs) {
        offset = static_cast<int>(m_layout.payloadOffset());
        blockSize = grs::control::kPayloadBlockSize;
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
        if (blockSize == grs::control::kUavBlockSize) {
            p.roll  = grs::radToDeg(m_referenceTrajectory[rowStart + 6]);
            p.pitch = grs::radToDeg(m_referenceTrajectory[rowStart + 7]);
        }
        points.push_back(p);
    }
    return points;
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

    // Shift the stages by the reference progress.
    for (size_t k = 0; k < N - shift; ++k) {
        const size_t dst = k * stride;
        const size_t src = (k + shift) * stride;

        std::copy_n(m_solver.x.begin() + src, stride, m_solver.x0.begin() + dst);
    }

    // Fill the tail with the last shifted stage.
    const size_t lastValidStage = N - shift;

    for (size_t k = N - shift; k < N; ++k) {
        const size_t dst = k * stride;
        const size_t src = lastValidStage * stride;

        std::copy_n(m_solver.x.begin() + src, stride, m_solver.x0.begin() + dst);
    }

    // Keep the terminal state.
    const size_t xN_src = N * stride;
    const size_t xN_dst = N * stride;

    std::copy_n(m_solver.x.begin() + xN_src, nx, m_solver.x0.begin() + xN_dst);

    // Initial state from the measurement.
    for (size_t i = 0; i < nx; ++i) {
        m_solver.x0[i] = m_initialStates[i] * m_config.invScalesStates[i];
    }

    std::ranges::fill(m_solver.lamX0, 0.0);
    std::ranges::fill(m_solver.lamG0, 0.0);
}

void MpcController::m_packBounds() {
    auto lb = m_solver.lbx.begin();
    auto ub = m_solver.ubx.begin();
    for (int k = 0; k <= m_config.N; ++k) {
        for (int i = 0; i < m_config.nx; ++i) {
            *lb++ = m_config.lbxStates[i] * m_config.invScalesStates[i];
            *ub++ = m_config.ubxStates[i] * m_config.invScalesStates[i];
        }
        if (k == m_config.N) break;
        for (int i = 0; i < m_config.nu; ++i) {
            *lb++ = m_config.lbxControls[i] * m_config.invScalesControls[i];
            *ub++ = m_config.ubxControls[i] * m_config.invScalesControls[i];
        }
    }
    assert(lb == m_solver.lbx.end());
}

void MpcController::m_packInequalityBounds() {
    const auto nx = static_cast<size_t>(m_config.nx);
    const auto N = static_cast<size_t>(m_config.N);
    const auto alphaRowsPerStage = static_cast<size_t>(m_config.numUavs);
    const double alphaMax = m_config.alphaMax;

    size_t idx = nx; // skip the leading nx initial-condition equality rows
    for (size_t k = 0; k < N; ++k) {
        idx += nx; // skip this stage's nx dynamics equality rows
        for (size_t a = 0; a < alphaRowsPerStage; ++a) {
            m_solver.lbg[idx] = -alphaMax;
            m_solver.ubg[idx] = alphaMax;
            ++idx;
        }
    }

    assert(idx == m_solver.lbg.size());
    assert(idx == m_solver.ubg.size());
}

void MpcController::m_packInitialGuess() {
    // Reference window [x0 u0 ... x(N-1) u(N-1) xN], then the measured x0.
    const size_t count = m_config.N * m_refStride + m_config.nx;
    const size_t offsetRef = m_lastIdxTraj * m_refStride;

    std::memcpy(m_solver.x0.data(), m_referenceTrajectory.data() + offsetRef, count * sizeof(double));

    std::ranges::copy(m_initialStates, m_solver.x0.begin());

    // Scale: N+1 state blocks, N control blocks.
    for (size_t k = 0; k < m_config.N + 1; ++k)
    {
        const size_t xOffset = k * m_refStride;
        for (size_t i = 0; i < m_config.nx; ++i) {
            m_solver.x0[xOffset + i] *= m_config.invScalesStates[i];
        }

        if (k == static_cast<size_t>(m_config.N)) break;

        const size_t uOffset = k * m_refStride + m_config.nx;
        for (size_t i = 0; i < m_config.nu; ++i) {
            m_solver.x0[uOffset + i] *= m_config.invScalesControls[i];
        }
    }
}

void MpcController::m_packParameters() {
    auto dst = std::ranges::copy(m_initialStates, m_solver.p.begin()).out;
    dst = std::copy_n(m_referenceTrajectory.begin() + m_lastIdxTraj * m_refStride, m_config.N * m_refStride + m_config.nx, dst);
    {
        std::lock_guard lock(m_disturbanceMutex);
        dst = std::ranges::copy(m_windEst, dst).out;
        dst = std::ranges::copy(m_dEst, dst).out;
    }
    dst = std::ranges::copy(m_config.weight, dst).out;
    dst = std::ranges::copy(m_uPrev, dst).out;
    dst = std::fill_n(dst, m_config.nL0, m_config.tetherL0);
    assert(dst == m_solver.p.end());
}

std::map<uint8_t, uavCommandsFlags> MpcController::m_extractControls() const {

    std::map<uint8_t, uavCommandsFlags> out;

    for (int sysId = 1; sysId <= m_config.numUavs; ++sysId) {
        uavCommandsFlags cmd;

        const int perUavNu = m_config.nu / m_config.numUavs;
        const int offset = m_config.nx + perUavNu * (sysId - 1);
        // This UAV's block in the joint scalesControls.
        const int scaleOffset = perUavNu * (sysId - 1);

        // Controls per UAV are [T, roll, pitch], yaw is always 0
        cmd.commands.sysId = static_cast<uint8_t>(sysId);

        if (m_violation) {
            // Rejected solution: reference feedforward control.
            const size_t ctrlOffset = m_lastIdxTraj * m_refStride + offset;
            cmd.commands.thrust      = static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 0));
            cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 1)));
            cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_referenceTrajectory.at(ctrlOffset + 2)));
            cmd.commands.yawDegree   = 0.0;
        } else {
            cmd.commands.thrust      = static_cast<float>(m_solver.x[offset + 0] * m_config.scalesControls[scaleOffset + 0]);
            cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_solver.x[offset + 1] * m_config.scalesControls[scaleOffset + 1]));
            cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_solver.x[offset + 2] * m_config.scalesControls[scaleOffset + 2]));
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
    const auto check = m_solver.check(flag);
    m_violation = !check.valid;
    m_lastMaxConstraintViolation = check.maxConstraintViolation;
    return check.valid;
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

void MpcController::m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates) {
    // A vehicle without telemetry this tick keeps its previous values.
    const auto fill = grs::control::fillStateVector(latestStates, m_layout, m_initialStates);
    m_telemetryComplete = fill.complete(m_layout);

    // In flight once any UAV exceeds 12 m/s (catapult launch done).
    for (int i = 0; i < m_config.numUavs; ++i) {
        const size_t o = m_layout.uavOffset(i);
        const double vn = m_initialStates[o + 3], ve = m_initialStates[o + 4], vd = m_initialStates[o + 5];
        if (fill.uav[i] && std::sqrt(vn * vn + ve * ve + vd * vd) > 12.0) {
            m_inFlight = true;
        }
    }

    // Pre-flight: UAV position/velocity from the reference's first sample
    // (attitude stays measured), so the solver starts from a consistent state
    // while the aircraft sit on the launchers.
    if (!m_launched || !m_inFlight) {
        for (int i = 0; i < m_config.numUavs; ++i) {
            const size_t o = m_layout.uavOffset(i);
            for (size_t k = 0; k < 6; ++k) {
                m_initialStates[o + k] = m_referenceTrajectory.at(o + k);
            }
        }
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4) << m_trackingNumber << "," << Logger::instance().nowMilliseconds() << "," << Logger::nowWallTimeMs() << ",";
    for (auto&& x : m_initialStates) {
        msg << x << ",";
    }
    Logger::instance().log(LogType::STATES, msg.str());
}
