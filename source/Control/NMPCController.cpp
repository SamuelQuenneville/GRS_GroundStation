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

    LOG_INFO("Trajectory loaded");
}

std::map<uint8_t, uavCommandsFlags> NMPCController::solve(const std::map<uint8_t, uavStates>& latestStates) {

    std::lock_guard lock(m_solveMutex);

    // Shift solution or pack initial guess
    if (m_lastSolveMs <= 0.0) {
        m_packInitialGuess();
    } else {
        m_shiftSolution();
    }

    // If the aircraft were launched we move along the ref traj
    if (m_launched) {
        m_idxTraj += 1;
    }

    m_packParameters(latestStates);
    m_bindSolverIO();


    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4);
    for (auto&& x : m_x0) {
        msg << x << ",";
    }
    Logger::instance().log(LogType::MPC_ARG_X0, msg.str());


    // Solve
    {
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs, false);
        const int flag = solver(m_arg.data(), m_res.data(), m_iw.data(), m_w.data(), m_mem);

        const auto converged = m_solutionIsValid(flag);
    }


    std::ostringstream msg2;
    msg2 << std::fixed << std::setprecision(4);
    for (auto&& x : m_x) {
        msg2 << x << ",";
    }
    Logger::instance().log(LogType::MPC_RES_X, msg2.str());


    // Extract and return u0 for each UAV
    auto controls = m_extractControls();
    return controls;
}

double NMPCController::lastSolveMs() const {
    return m_lastSolveMs;
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

void NMPCController::m_shiftSolution() {
    // TODO Warm-start: shift the solution and repeat last

    // Initial guess is last result, no shifting
    std::ranges::copy(m_x, m_x0.begin());

    // Warm-start: store multipliers for next iteration
    std::ranges::copy(m_lam_x, m_lam_x0.begin());
    std::ranges::copy(m_lam_g, m_lam_g0.begin());
}

void NMPCController::m_packBounds() {

    size_t offset = 0;

    for (size_t k = 0; k <= static_cast<size_t>(m_config.N); ++k) {

        std::ranges::copy(m_config.lbxStates, m_lbx.begin() + static_cast<std::ptrdiff_t>(offset));
        std::ranges::copy(m_config.ubxStates, m_ubx.begin() + static_cast<std::ptrdiff_t>(offset));

        offset += m_config.lbxStates.size();

        if (k < static_cast<size_t>(m_config.N)) {
            std::ranges::copy(m_config.lbxControls, m_lbx.begin() + static_cast<std::ptrdiff_t>(offset));
            std::ranges::copy(m_config.ubxControls, m_ubx.begin() + static_cast<std::ptrdiff_t>(offset));

            offset += m_config.lbxControls.size();
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
    const size_t offsetRef = m_idxTraj * m_refStride;

    std::memcpy(m_x0.data(), m_referenceTrajectory.data() + offsetRef, count * sizeof(double));

    assert(m_x0.size() == solver_sparsity_in(0)[0]);
}

void NMPCController::m_packParameters(const std::map<uint8_t, uavStates>& latestStates) {

    size_t offset = 0;

    const size_t nx = m_config.nx;
    const size_t stride = m_refStride;
    const size_t N = m_config.N;

    double* p = m_p.data();

    // x_initial
    m_unpackLatestStates(latestStates, m_initialStates);
    std::memcpy(p + offset, m_initialStates.data(), nx * sizeof(double));
    offset += nx;

    const size_t count = N * stride + nx;
    const size_t offsetRef = m_idxTraj * stride;

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

        cmd.commands.thrust      = static_cast<float>(m_x[offset + 0]);
        cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_x[offset + 1]));
        cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_x[offset + 2]));
        cmd.commands.yawDegree   = 0.0;

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
        msg << std::fixed << std::setprecision(4) << Logger::instance().nowMilliseconds() << "," << m_lastSolveMs << ",";
        msg << cmd.commands.thrust << "," << cmd.commands.rollDegree << "," << cmd.commands.pitchDegree << "," << cmd.commands.yawDegree;

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

    constexpr double feas_tol = 1e-5;

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
            double gamma = std::asin(-states.downMeterSecond / speed);

            if (speed < 12.0) {
                speed = 12.0;
                gamma = 0.093;
            }

            const double yawContinuous = m_unwrapYaw(sysId, grs::degToRad(states.yawDegree));

            unpackStates.at(offset++) = states.northMeter;
            unpackStates.at(offset++) = states.eastMeter;
            unpackStates.at(offset++) = states.downMeter;
            unpackStates.at(offset++) = speed;
            unpackStates.at(offset++) = yawContinuous;
            unpackStates.at(offset++) = grs::degToRad(states.rollDegree);
            unpackStates.at(offset++) = grs::degToRad(states.pitchDegree);

            if (m_config.nx == 8) {
               unpackStates.at(offset++) = gamma;
            }

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
    msg << std::fixed << std::setprecision(4) << Logger::instance().nowMilliseconds() << ",";
    for (auto&& x : unpackStates) {
        msg << x << ",";
    }
    Logger::instance().log(LogType::STATES, msg.str());
}
