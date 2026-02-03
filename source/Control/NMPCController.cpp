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
    m_initalizeSolverIO();
    m_packBounds();
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

        referencePoint pt;
        pt.statesRef.resize(m_config.nx);
        pt.controlsRef.resize(m_config.nu);

        // first nx fields : states
        for (int i = 0; i < m_config.nx; i++) {
            std::getline(ss, field, ',');
            pt.statesRef[i] = std::stod(field);
        }

        // next nu fields : controls
        for (int i = 0; i < m_config.nu; i++) {
            std::getline(ss, field, ',');
            pt.controlsRef[i] = std::stod(field);
        }

        m_referenceTrajectory.push_back(std::move(pt));
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
    m_bindSolverIO();                   // casadi solver() mess with lbx and ubx

    // Solve
    dumpSolverArgsToFile(m_arg);
    {
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs);
        solver(m_arg.data(), m_res.data(), m_iw.data(), m_w.data(), m_mem);
    }

    // Extract and return u0 for each UAV
    return m_extractControls();
}

double NMPCController::lastSolveMs() const {
    return m_lastSolveMs;
}

void NMPCController::dumpSolverArgsToFile(const std::vector<const casadi_real*>& arg) {

    for (size_t i = 0; i < arg.size(); ++i) {

        if (!arg[i]) continue;

        const size_t sz = solver_sparsity_in(i)[0];

        std::ofstream f("mpc_arg" + std::to_string(i) + ".txt", std::ios::out | std::ios::app);

        for (size_t k = 0; k < sz; ++k) {
            f << arg[i][k];
            if (k + 1 < sz) f << ' ';
        }
        f << '\n';
    }
}

void NMPCController::m_initalizeSolverIO() {
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
    m_arg.resize(solver_n_in());
    m_res.resize(solver_n_out());

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

    m_lbx.clear();
    m_ubx.clear();

    for (size_t k = 0; k <= static_cast<size_t>(m_config.N); k++) {
        m_lbx.insert(m_lbx.end(), m_config.lbxStates.begin(), m_config.lbxStates.end());
        m_ubx.insert(m_ubx.end(), m_config.ubxStates.begin(), m_config.ubxStates.end());

        if (k < static_cast<size_t>(m_config.N)) {
            m_lbx.insert(m_lbx.end(), m_config.lbxControls.begin(), m_config.lbxControls.end());
            m_ubx.insert(m_ubx.end(), m_config.ubxControls.begin(), m_config.ubxControls.end());
        }
    }

    assert(m_lbx.size() == solver_sparsity_in(2)[0]);
    assert(m_ubx.size() == solver_sparsity_in(3)[0]);
}

void NMPCController::m_packInitialGuess() {

    m_x0.clear();

    // reference over horizon
    for (int idx = 0; idx < m_config.N; ++idx) {

        const auto& [statesRef, controlsRef] = m_referenceTrajectory[idx];

        // x_ref
        m_x0.insert(m_x0.end(), statesRef.begin(), statesRef.end());

        // u_ref
        m_x0.insert(m_x0.end(), controlsRef.begin(), controlsRef.end());
    }

    // x_ref N
    const auto& [statesRef, _] = m_referenceTrajectory[m_config.N];
    m_x0.insert(m_x0.end(), statesRef.begin(), statesRef.end());

    assert(m_x0.size() == solver_sparsity_in(0)[0]);
}

void NMPCController::m_packParameters(const std::map<uint8_t, uavStates>& latestStates) {

    m_p.clear();

    // reference over horizon
    int idx = 0;
    for (int i = 0; i < m_config.N; ++i) {

        idx = m_idxTraj + i;
        if (idx >= static_cast<int>(m_referenceTrajectory.size()) - 1) {
            idx = static_cast<int>(m_referenceTrajectory.size()) - 1;
            m_endedTraj = true;
        }

        const auto& [statesRef, controlsRef] = m_referenceTrajectory[idx];

        // x_ref(i)
        m_p.insert(m_p.end(), statesRef.begin(), statesRef.end());

        // u_ref(i)
        m_p.insert(m_p.end(), controlsRef.begin(), controlsRef.end());

        // p_ref(i) (wind)
        m_p.insert(m_p.end(), {0.0, 0.0, 0.0});
    }

    // terminal state x_ref(N)
    const auto& [statesRef, _] = m_referenceTrajectory[idx + 1];
    m_p.insert(m_p.end(), statesRef.begin(), statesRef.end());

    // x_initial (latestStates)
    auto initialStates = m_unpackLatestStates(latestStates);
    m_p.insert(m_p.end(), initialStates.begin(), initialStates.end());

    assert(m_p.size() == solver_sparsity_in(1)[0]);
}

std::map<uint8_t, uavCommandsFlags> NMPCController::m_extractControls() const {

    std::map<uint8_t, uavCommandsFlags> out;

    // offset = expected states
    const int offset_u0 = m_config.nx * (m_config.N + 1);

    // Extract u0 for each UAV in order
    int u_idx = offset_u0;
    for (int sysId = 1; sysId <= m_config.numUavs; ++sysId) {
        uavCommandsFlags cmd;

        // Controls per UAV are [T, roll, pitch], yaw is always 0
        cmd.commands.sysId = static_cast<uint8_t>(sysId);

        cmd.commands.thrust      = static_cast<float>(m_x[u_idx + 0]);
        cmd.commands.rollDegree  = static_cast<float>(m_x[u_idx + 1]) * 180.0f / M_PIf;
        cmd.commands.pitchDegree = static_cast<float>(m_x[u_idx + 2]) * 180.0f / M_PIf;
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
        u_idx += m_config.nu;
    }

    LOG_INFO("SOLVED");
    return out;
}

std::vector<double> NMPCController::m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates) const {

    // Payload is last sysId -- look if we are tethered to the ground (no payload)
    size_t numberOfUavs = latestStates.size() - 1;

    if (numberOfUavs == 0) {
        numberOfUavs += 1;
    }

    std::vector<double> unpackStates;

    for (const auto&[sysId, states] : latestStates) {
        if (sysId <= numberOfUavs) {

            float speed = 12.0f;
            if (m_launched) {
                speed = std::sqrt(states.northMeterSecond*states.northMeterSecond + states.eastMeterSecond*states.eastMeterSecond);
            }
            unpackStates.insert(unpackStates.end(),
                {states.northMeter,
                   states.eastMeter,
                   states.downMeter,
                   speed,
                   grs::degToRad(states.yawDegree),
                   grs::degToRad(states.rollDegree),
                   grs::degToRad(states.pitchDegree)});
        } else {
            unpackStates.insert(unpackStates.end(),
                {states.northMeter,
                   states.eastMeter,
                   states.downMeter,
                   states.northMeterSecond,
                   states.eastMeterSecond,
                   states.downMeterSecond});
        }
    }

    return unpackStates;
}
