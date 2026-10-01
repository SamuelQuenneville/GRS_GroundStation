/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "mpcController.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

MpcController::MpcController(const solverConfig& config)
    : m_config(config)
    , m_solver(config.controller == solverConfig::Controller::Lmpc ? GeneratedFunction::Id::Lmpc : GeneratedFunction::Id::Nmpc,
               config.numUavs)
{
    m_refStride = m_config.nx + m_config.nu;
    m_nz = m_config.nx + m_config.nu;
    m_solStride = m_nz + m_config.nu;

    const size_t nDecision = (m_config.N + 1) * m_nz + m_config.N * m_config.nu;
    if (m_solver.x0.size() != nDecision) {
        throw std::runtime_error(std::string(m_solver.name()) + " has " + std::to_string(m_solver.x0.size())
            + " decision variables, the augmented formulation [x; up] needs " + std::to_string(nDecision)
            + " (regenerate it with generate_solvers.m)");
    }

    // Layout of m_packParameters(). A mismatch means SolverConfiguration does
    // not describe the generated solver.
    size_t expected = 2 * m_config.nx + m_config.N * m_refStride + m_config.np + m_config.nd
        + m_config.weight.size() + m_config.nu + m_config.nL0;
    if (m_config.controller == solverConfig::Controller::Lmpc) {
        m_linearization.emplace(GeneratedFunction::Id::LmpcLinearization, m_config.numUavs);
        if (m_linearization->inputSize(0) != m_config.N * m_refStride) {
            throw std::runtime_error(std::string(m_linearization->name()) + " was generated for another N, nx or nu");
        }
        expected += m_linearization->outputSize(0);
    }
    if (m_solver.p.size() != expected) {
        throw std::runtime_error(std::string(m_solver.name()) + " takes " + std::to_string(m_solver.p.size())
            + " parameters, SolverConfiguration describes " + std::to_string(expected));
    }

    m_layout = {m_config.numUavs, hasPayload()};
    assert(m_layout.size() == static_cast<size_t>(m_config.nx));

    m_initialStates.assign(m_config.nx, 0.0);
    m_referenceWindow.assign(m_config.N * m_refStride + m_config.nx, 0.0);
    m_uPrev.assign(m_config.nu, 0.0);
    m_linPoint.assign(m_config.N * m_refStride, 0.0);
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
    m_planIdx.reset();
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

std::map<uint8_t, uavCommandsFlags> MpcController::solve(const std::map<uint8_t, uavStates>& latestStates, const double time) {

    std::lock_guard lock(m_solveMutex);

    m_unpackLatestStates(latestStates);

    m_updateReference(time);

    // Warm start from the last accepted plan shifted to the current
    // reference sample, cold start from the reference when it is too old.
    const auto N = static_cast<size_t>(m_config.N);
    m_planAge = m_planIdx ? m_lastIdxTraj - *m_planIdx : N;
    const bool fromPlan = m_planAge < N;
    if (fromPlan) {
        m_shiftSolution(m_planAge);
    } else {
        m_packInitialGuess();
    }

    {
        // Includes the LMPC linearization, part of the online cost.
        PROFILE_SCOPE_OUT("casadi_solve", &m_lastSolveMs, false);
        m_packParameters(fromPlan);
        m_lastStatus = m_solver.solve();

        const auto check = m_solver.check(m_lastStatus);
        m_violation = !check.valid;
        m_lastMaxConstraintViolation = check.maxConstraintViolation;
    }

    if (!m_violation) {
        m_plan = m_solver.x;
        m_planIdx = m_lastIdxTraj;
        m_planAge = 0;
    }

    // Applied control: the plan at its age, else the reference feedforward.
    // Also U_prev for the next solve.
    if (m_planAge < N) {
        const size_t offset = m_planAge * m_solStride + m_nz;
        for (int i = 0; i < m_config.nu; ++i) {
            m_uPrev[i] = m_plan[offset + i] * m_config.scalesControls[i];
        }
    } else {
        std::copy_n(m_referenceWindow.begin() + m_config.nx, m_config.nu, m_uPrev.begin());
    }

    auto controls = m_extractControls();

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
    info.lastFlag = m_lastStatus.flag;
    info.lastFatrop = m_lastStatus.fatrop;
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

void MpcController::m_updateReference(const double time) {
    if (m_launched && !m_launchTime) {
        m_launchTime = time;
    }

    // Reference position in samples, clamped to the last full window.
    const double last = m_endIdxTraj > 0 ? static_cast<double>(m_endIdxTraj - 1) : 0.0;
    double s = m_launchTime ? std::clamp((time - *m_launchTime) / m_config.dt, 0.0, last) : 0.0;
    if (std::abs(s - std::round(s)) < 1e-6) {
        s = std::round(s); // ticks exactly on the sample grid use the samples as they are
    }
    m_referenceTime = s * m_config.dt;
    m_lastIdxTraj = static_cast<size_t>(s);
    if (m_launched && m_lastIdxTraj + 1 >= m_endIdxTraj) {
        m_endedTraj = true;
    }
    if (m_endIdxTraj == 0) {
        return; // no reference long enough for a window
    }

    // Window [x u] x N, then x, linearly interpolated between samples.
    const double a = s - static_cast<double>(m_lastIdxTraj);
    const double* r0 = m_referenceTrajectory.data() + m_lastIdxTraj * m_refStride;
    const double* r1 = a > 0.0 ? r0 + m_refStride : r0; // a > 0 only below `last`: r1's window is in range
    for (size_t j = 0; j < m_referenceWindow.size(); ++j) {
        m_referenceWindow[j] = (1.0 - a) * r0[j] + a * r1[j];
    }
}

void MpcController::m_shiftSolution(const size_t shift) {
    const size_t N = m_config.N;
    const size_t stride = m_solStride;

    // Stage k takes the plan's stage k + shift: z up to the terminal one,
    // u up to the last one (shift_primal.m). The terminal z stays.
    for (size_t k = 0; k < N; ++k) {
        const size_t zSrc = std::min(k + shift, N) * stride;
        const size_t uSrc = std::min(k + shift, N - 1) * stride + m_nz;
        std::copy_n(m_plan.begin() + zSrc, m_nz, m_solver.x0.begin() + k * stride);
        std::copy_n(m_plan.begin() + uSrc, m_config.nu, m_solver.x0.begin() + k * stride + m_nz);
    }
    std::copy_n(m_plan.begin() + N * stride, m_nz, m_solver.x0.begin() + N * stride);

    m_packFirstStage();
}

void MpcController::m_packFirstStage() {
    for (int i = 0; i < m_config.nx; ++i) {
        m_solver.x0[i] = m_initialStates[i] * m_config.invScalesStates[i];
    }
    for (int j = 0; j < m_config.nu; ++j) {
        m_solver.x0[m_config.nx + j] = m_uPrev[j] * m_config.invScalesControls[j];
    }
}

void MpcController::m_packBounds() {
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto lb = m_solver.lbx.begin();
    auto ub = m_solver.ubx.begin();
    for (int k = 0; k <= m_config.N; ++k) {
        for (int i = 0; i < m_config.nx; ++i) {
            *lb++ = k == 0 ? -inf : m_config.lbxStates[i] * m_config.invScalesStates[i];
            *ub++ = k == 0 ? inf : m_config.ubxStates[i] * m_config.invScalesStates[i];
        }
        lb = std::fill_n(lb, m_config.nu, -inf);
        ub = std::fill_n(ub, m_config.nu, inf);
        if (k == m_config.N) break;
        for (int i = 0; i < m_config.nu; ++i) {
            *lb++ = m_config.lbxControls[i] * m_config.invScalesControls[i];
            *ub++ = m_config.ubxControls[i] * m_config.invScalesControls[i];
        }
    }
    assert(lb == m_solver.lbx.end());
}

void MpcController::m_packInequalityBounds() {
    const auto N = static_cast<size_t>(m_config.N);
    const auto alphaRowsPerStage = static_cast<size_t>(m_config.numUavs);
    const double alphaMax = m_config.alphaMax;

    size_t idx = m_nz; // skip the initial-condition equality rows
    for (size_t k = 0; k < N; ++k) {
        idx += m_nz; // skip this stage's dynamics equality rows
        for (size_t a = 0; a < alphaRowsPerStage; ++a) {
            m_solver.lbg[idx] = -alphaMax;
            m_solver.ubg[idx] = alphaMax;
            ++idx;
        }
    }

    if (idx != m_solver.lbg.size()) {
        throw std::runtime_error(std::string(m_solver.name()) + " has " + std::to_string(m_solver.lbg.size())
            + " constraint rows, expected " + std::to_string(idx));
    }
}

void MpcController::m_packInitialGuess() {
    const size_t nx = m_config.nx;
    const size_t nu = m_config.nu;
    const size_t N = m_config.N;

    for (size_t k = 0; k <= N; ++k) {
        const double* ref = m_referenceWindow.data() + k * m_refStride; // x_k, u_k
        double* z = m_solver.x0.data() + k * m_solStride;
        for (size_t i = 0; i < nx; ++i) {
            z[i] = ref[i] * m_config.invScalesStates[i];
        }
        if (k > 0) {
            const double* uPrevRef = ref - m_refStride + nx; // u_(k-1)
            for (size_t j = 0; j < nu; ++j) {
                z[nx + j] = uPrevRef[j] * m_config.invScalesControls[j];
            }
        }
        if (k == N) break;
        for (size_t j = 0; j < nu; ++j) {
            z[m_nz + j] = ref[nx + j] * m_config.invScalesControls[j];
        }
    }

    m_packFirstStage();
}

void MpcController::m_packParameters(const bool aboutPlan) {
    double* dst = std::ranges::copy(m_initialStates, m_solver.p.data()).out;
    dst = std::ranges::copy(m_referenceWindow, dst).out;
    const double* wind = dst;
    {
        std::lock_guard lock(m_disturbanceMutex);
        dst = std::ranges::copy(m_windEst, dst).out;
        dst = std::ranges::copy(m_dEst, dst).out;
    }
    const double* d = wind + m_config.np;
    dst = std::ranges::copy(m_config.weight, dst).out;
    dst = std::ranges::copy(m_uPrev, dst).out;
    const double* L0 = dst;
    dst = std::fill_n(dst, m_config.nL0, m_config.tetherL0);
    if (m_linearization) {
        const double* point = m_referenceWindow.data();
        if (aboutPlan) {
            // The warm start, unscaled [x u] x N: the shifted plan from the measured state.
            for (int k = 0; k < m_config.N; ++k) {
                const double* z = m_solver.x0.data() + k * m_solStride;
                double* xu = m_linPoint.data() + k * m_refStride;
                for (int i = 0; i < m_config.nx; ++i) {
                    xu[i] = z[i] * m_config.scalesStates[i];
                }
                for (int j = 0; j < m_config.nu; ++j) {
                    xu[m_config.nx + j] = z[m_nz + j] * m_config.scalesControls[j];
                }
            }
            point = m_linPoint.data();
        }
        m_linearization->eval({point, wind, d, L0}, {dst});
        dst += m_linearization->outputSize(0);
    }
    assert(dst == m_solver.p.data() + m_solver.p.size());
}

std::map<uint8_t, uavCommandsFlags> MpcController::m_extractControls() const {

    std::map<uint8_t, uavCommandsFlags> out;

    for (int sysId = 1; sysId <= m_config.numUavs; ++sysId) {
        uavCommandsFlags cmd;

        // Controls per UAV are [T, roll, pitch], yaw is always 0
        const int offset = m_config.nu / m_config.numUavs * (sysId - 1);
        cmd.commands.sysId       = static_cast<uint8_t>(sysId);
        cmd.commands.thrust      = static_cast<float>(m_uPrev[offset + 0]);
        cmd.commands.rollDegree  = grs::radToDeg(static_cast<float>(m_uPrev[offset + 1]));
        cmd.commands.pitchDegree = grs::radToDeg(static_cast<float>(m_uPrev[offset + 2]));
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
        msg << std::fixed << std::setprecision(4) << m_trackingNumber << "," << Logger::instance().nowMilliseconds() << "," << Logger::nowWallTimeMs() << "," << m_lastSolveMs << ",";
        msg << cmd.commands.thrust << "," << cmd.commands.rollDegree << "," << cmd.commands.pitchDegree << "," << cmd.commands.yawDegree << "," << m_lastIdxTraj;
        msg << "," << m_lastStatus.fatrop.iterations << "," << m_lastStatus.fatrop.returnCode << "," << m_referenceTime;

        if (m_violation) {
            msg << ", INVALID SOL, " << (m_planAge < static_cast<size_t>(m_config.N) ? "plan age " + std::to_string(m_planAge) : "reference");
        }
        Logger::instance().log(LogType::CONTROLS, msg.str());
    }

    return out;
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
