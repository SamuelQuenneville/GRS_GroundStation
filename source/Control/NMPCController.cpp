/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "NMPCController.h"

NMPCController::NMPCController(const solverConfig& config)
    : m_config(config)
    , m_iw(solver_SZ_IW)
    , m_w(solver_SZ_W)
{
    m_refStride = m_config.nx + m_config.nu;
    m_endIdxTraj = m_numTrajectoryPoints - m_config.N;

    m_initialStates.resize(m_config.nx);

    m_initializeSolverIO();
    m_packBounds();

    m_arg.resize(solver_n_in());
    m_res.resize(solver_n_out());
    m_bindSolverIO();

    m_mem = solver_checkout();
    solver_init_mem(m_mem);
}

NMPCController::~NMPCController() {
    solver_release(m_mem);
    m_mem = -1;
}

void NMPCController::initLaunch() {
    m_launched = true;
    m_timeAtLaunched = std::chrono::steady_clock::now();
}

void NMPCController::loadTrajectory(const std::string& file) {

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

    m_numTrajectoryPoints = m_referenceTrajectory.size() / m_refStride;
    LOG_INFO("Trajectory loaded, number of points = " + std::to_string(m_numTrajectoryPoints));
}

std::map<uint8_t, uavCommandsFlags> NMPCController::solve(const std::map<uint8_t, uavStates>& latestStates) {

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
    }

    if (m_lastIdxTraj == m_endIdxTraj) {
        m_endedTraj = true;
    }

    m_packParameters();
    m_bindSolverIO();


    // Solve
    {
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs, false);
        const int flag = solver(m_arg.data(), m_res.data(), m_iw.data(), m_w.data(), m_mem);

        const auto converged = m_solutionIsValid(flag);
    }

    // Extract and return u0 for each UAV
    auto controls = m_extractControls();

    m_trackingNumber += 1;
    return controls;
}

double NMPCController::lastSolveMs() const {
    return m_lastSolveMs;
}

NMPCController::DebugInfo NMPCController::getDebugInfo() const {
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
    return info;
}

std::vector<NMPCController::TrajectoryPointView> NMPCController::getTrajectoryForVehicle(const int vehicleIndex) const {
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

void NMPCController::m_initializeSolverIO() {
    // Inputs
    m_x0.assign(solver_sparsity_in(0)[0], 0.0);
    m_p.assign(solver_sparsity_in(1)[0], 0.0);
    m_lbx.assign(solver_sparsity_in(2)[0], 0.0);
    m_ubx.assign(solver_sparsity_in(3)[0], 0.0);
    m_lbg.assign(solver_sparsity_in(4)[0], 0.0);
    m_ubg.assign(solver_sparsity_in(5)[0], 0.0);
    m_lam_x0.assign(solver_sparsity_in(6)[0], 0.0);
    m_lam_g0.assign(solver_sparsity_in(7)[0], 0.0);

    // Outputs
    m_x.assign(solver_sparsity_out(0)[0], 0.0);
    m_f.assign(solver_sparsity_out(1)[0], 0.0);
    m_g.assign(solver_sparsity_out(2)[0], 0.0);
    m_lam_x.assign(solver_sparsity_out(3)[0], 0.0);
    m_lam_g.assign(solver_sparsity_out(4)[0], 0.0);
    m_lam_p.assign(solver_sparsity_out(5)[0], 0.0);
}

void NMPCController::m_bindSolverIO() {
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

double NMPCController::m_computeReferenceCost(const size_t idx) const {
    const size_t refOffset = idx * m_refStride;

    double cost = 0.0;

    const double refNorth = m_referenceTrajectory[refOffset + 0];
    const double refEast = m_referenceTrajectory[refOffset + 1];

    const double dn = m_initialStates[0] - refNorth;
    const double de = m_initialStates[1]  - refEast;

    cost += dn*dn + de*de;

    return cost;
}

void NMPCController::m_shiftSolution() {

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

void NMPCController::m_packBounds() {

    std::vector<double> lbxStates(m_config.nx);
    std::vector<double> ubxStates(m_config.nx);
    for (size_t i = 0; i < m_config.nx; ++i) {
        lbxStates[i] = m_config.lbxStates[i] * m_config.invScalesStates[i];
        ubxStates[i] = m_config.ubxStates[i] * m_config.invScalesStates[i];
    }

    std::vector<double> lbxControls(m_config.nu);
    std::vector<double> ubxControls(m_config.nu);
    for (size_t i = 0; i < m_config.nu; ++i) {
        lbxControls[i] = m_config.lbxControls[i] * m_config.invScalesControls[i];
        ubxControls[i] = m_config.ubxControls[i] * m_config.invScalesControls[i];
    }

    size_t offset = 0;

    for (size_t k = 0; k <= static_cast<size_t>(m_config.N); ++k) {

        std::ranges::copy(lbxStates, m_lbx.begin() + static_cast<std::ptrdiff_t>(offset));
        std::ranges::copy(ubxStates, m_ubx.begin() + static_cast<std::ptrdiff_t>(offset));

        offset += lbxStates.size();

        if (k < static_cast<size_t>(m_config.N)) {
            std::ranges::copy(lbxControls, m_lbx.begin() + static_cast<std::ptrdiff_t>(offset));
            std::ranges::copy(ubxControls, m_ubx.begin() + static_cast<std::ptrdiff_t>(offset));

            offset += lbxControls.size();
        }
    }

    assert(offset == m_lbx.size());
    assert(m_lbx.size() == solver_sparsity_in(2)[0]);
    assert(m_ubx.size() == solver_sparsity_in(3)[0]);
}

void NMPCController::m_packInitialGuess() {

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


    assert(m_x0.size() == solver_sparsity_in(0)[0]);
}

void NMPCController::m_packParameters() {

    size_t offset = 0;

    const size_t nx = m_config.nx;
    const size_t stride = m_refStride;
    const size_t N = m_config.N;

    double* p = m_p.data();

    // x_initial
    std::memcpy(p + offset, m_initialStates.data(), nx * sizeof(double));
    offset += nx;

    const size_t count = N * stride + nx;
    const size_t offsetRef = m_lastIdxTraj * stride;

    // copy [x0 u0 ... xN-1 uN-1 xN]
    std::memcpy(p + offset, m_referenceTrajectory.data() + offsetRef, count * sizeof(double));
    offset += count;

    // p_ref (wind)
    double wind[3] = {0.0, 0.0, 0.0}; // TODO Wind could come from an estimator later on
    std::memcpy(p + offset, wind, sizeof(wind));
    offset += 3;

    // cost function weight
    std::memcpy(p + offset, m_config.weight.data(), m_config.weight.size() * sizeof(double));
    offset += m_config.weight.size();

    assert(offset == m_p.size());
    assert(m_p.size() == solver_sparsity_in(1)[0]);
}

std::map<uint8_t, uavCommandsFlags> NMPCController::m_extractControls() const {

    std::map<uint8_t, uavCommandsFlags> out;

    // Extract u0 for each UAV in order
    for (int sysId = 1; sysId <= m_config.numUavs; ++sysId) {
        uavCommandsFlags cmd;

        const int offset = m_config.nx + (m_config.nu / m_config.numUavs) * (sysId - 1);

        // Controls per UAV are [T, roll, pitch], yaw is always 0
        cmd.commands.sysId = static_cast<uint8_t>(sysId);

        if (m_violation) {
            cmd.commands.thrust      = 14.0;
            cmd.commands.rollDegree  = 10.0;
            cmd.commands.pitchDegree = 4.0;
            cmd.commands.yawDegree   = 0.0;
        } else {
            cmd.commands.thrust      = static_cast<float>(m_x[offset + 0] * m_config.scalesControls[0]);
            cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_x[offset + 1]));
            cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_x[offset + 2]));
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

bool NMPCController::m_solutionIsValid(const int flag) {
    if (flag != 0)
        return false;

    constexpr double feas_tol = 5e-4;

    // Check constraints
    double max_violation = 0.0;

    for (int i = 0; i < m_g.size(); ++i) {
        double v_low  = m_lbg[i] - m_g[i];
        double v_high = m_g[i] - m_ubg[i];
        double violation = std::max({0.0, v_low, v_high});
        max_violation = std::max(max_violation, violation);
    }

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

double NMPCController::m_unwrapYaw(const uint8_t sysId, const double yawRadWrapped) {

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

void NMPCController::m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates, std::vector<double>& unpackStates) {

    size_t offset = 0;

    // Payload is last sysId -- look if we are tethered to the ground (no payload)
    size_t numberOfUavs = latestStates.size() - 1;

    if (numberOfUavs == 0) {
        numberOfUavs += 1;
    }

    for (const auto&[sysId, states] : latestStates) {
        if (sysId <= numberOfUavs) {

            double speed = std::sqrt(states.northMeterSecond*states.northMeterSecond + states.eastMeterSecond*states.eastMeterSecond + states.downMeterSecond*states.downMeterSecond);

            double north = states.northMeter;
            double east = states.eastMeter;
            double down = states.downMeter;

            double vNorth = states.northMeterSecond;
            double vEast = states.eastMeterSecond;
            double vDown = states.downMeterSecond;

            if (speed > 13.0) {
                m_inFlight = true;
            }

            if (!m_launched || !m_inFlight) {
                north = 29.9681;
                east = 0.0;
                down = -1.382;

                vNorth = -0.05;
                vEast = -11.94;
                vDown = -1.115;
            }

            unpackStates.at(offset++) = north;
            unpackStates.at(offset++) = east;
            unpackStates.at(offset++) = down;
            unpackStates.at(offset++) = vNorth;
            unpackStates.at(offset++) = vEast;
            unpackStates.at(offset++) = vDown;
            unpackStates.at(offset++) = grs::degToRad(states.rollDegree);
            unpackStates.at(offset++) = grs::degToRad(states.pitchDegree);


        } else {
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
