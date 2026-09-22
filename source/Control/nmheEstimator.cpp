/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "nmheEstimator.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "Util/profilingTimer.h"

NmheEstimator::NmheEstimator(const estimatorConfig& config, std::unique_ptr<EstimatorBackend> backend)
    : m_config(config)
    , m_backend(std::move(backend))
    , m_iw(m_backend->workIntSize())
    , m_w(m_backend->workRealSize())
{
    m_windEst.assign(m_config.np, 0.0);
    m_dEst.assign(m_config.nd, 0.0);

    m_initializeSolverIO();

    // Bounds are the same every solve (tiled from config, no state
    // dependency) -- packed once here, same as MpcController::m_packBounds()
    // does for the NMPC side.
    m_backend->packBounds(m_config, m_lbx, m_ubx);

    // g is purely the M stage-linking dynamics equalities (ng=0 per stage
    // beyond that -- see nmhe-fatrop-stage-structure.md), so lbg=ubg=0 for
    // every row, for the whole run. Left at m_initializeSolverIO()'s
    // zero-init, never repacked -- unlike NMPC's alpha inequality rows,
    // there is no non-zero g bound anywhere in this NLP.
    m_arg.resize(8);
    m_res.resize(6);
    m_bindSolverIO();
}

NmheEstimator::~NmheEstimator() = default;

void NmheEstimator::addSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) {
    std::lock_guard lock(m_solveMutex);

    if (!m_stateWindow.empty()) {
        m_controlWindow.push_back(appliedControl);
        if (m_controlWindow.size() > static_cast<size_t>(m_config.M)) {
            m_controlWindow.pop_front();
        }
    }

    m_stateWindow.push_back(measuredState);
    if (m_stateWindow.size() > static_cast<size_t>(m_config.M) + 1) {
        m_stateWindow.pop_front();
    }

    m_windowFull = m_stateWindow.size() == static_cast<size_t>(m_config.M) + 1
        && m_controlWindow.size() == static_cast<size_t>(m_config.M);
}

bool NmheEstimator::estimate() {
    std::lock_guard lock(m_solveMutex);

    if (!m_windowFull) {
        return false;
    }

    // Flatten the sliding windows, oldest to newest -- EstimatorBackend::
    // packParameters()'s expected layout.
    std::vector<double> measurementWindow;
    measurementWindow.reserve(m_stateWindow.size() * m_config.nx);
    for (const auto& s : m_stateWindow) {
        measurementWindow.insert(measurementWindow.end(), s.begin(), s.end());
    }

    std::vector<double> controlWindow;
    controlWindow.reserve(m_controlWindow.size() * m_config.nu);
    for (const auto& u : m_controlWindow) {
        controlWindow.insert(controlWindow.end(), u.begin(), u.end());
    }

    // m_windEst/m_dEst still hold the PREVIOUS solve's estimate here --
    // that's deliberately what's fed in as this solve's arrival-cost
    // prior, before being overwritten below with this solve's own result.
    m_backend->packParameters(m_config, measurementWindow, controlWindow, m_windEst, m_dEst, m_p);
    m_packInitialGuess();
    m_bindSolverIO();

    int flag = 0;
    {
        PROFILE_SCOPE_OUT("nmhe_solve", &m_lastSolveMs, false);
        flag = m_backend->solve(m_arg.data(), m_res.data(), m_iw.data(), m_w.data());
        m_lastFlag = flag;
    }

    const bool valid = m_solutionIsValid(flag);

    if (valid) {
        // Wind/d are identical across every stage by construction (identity
        // "dynamics" tie them together, see nmhe-fatrop-stage-structure.md),
        // so any stage's block is the estimate -- the last one (index M) is
        // read here for no reason beyond simple indexing.
        const size_t nxi = static_cast<size_t>(m_config.nxi);
        const size_t lastStageOffset = static_cast<size_t>(m_config.M) * nxi;
        const size_t windOffset = lastStageOffset + m_config.nx;
        const size_t dOffset = windOffset + m_config.np;

        for (int i = 0; i < m_config.np; ++i) {
            const double scale = m_config.windScale.empty() ? 1.0 : m_config.windScale[i % m_config.windScale.size()];
            m_windEst[i] = m_x[windOffset + i] * scale;
        }
        for (int i = 0; i < m_config.nd; ++i) {
            const double scale = m_config.dScale.empty() ? 1.0 : m_config.dScale[i % m_config.dScale.size()];
            m_dEst[i] = m_x[dOffset + i] * scale;
        }
    }
    // On a violation, m_windEst/m_dEst are deliberately left unchanged --
    // same zero-order-hold-the-last-good-estimate fallback philosophy as
    // MpcController's m_extractControls() falling back to the planned
    // open-loop control on m_violation, just applied to the estimate
    // instead of the command.

    return valid;
}

const std::vector<double>& NmheEstimator::windEstimate() const {
    std::lock_guard lock(m_solveMutex);
    return m_windEst;
}

const std::vector<double>& NmheEstimator::dEstimate() const {
    std::lock_guard lock(m_solveMutex);
    return m_dEst;
}

Estimator::DebugInfo NmheEstimator::getDebugInfo() const {
    std::lock_guard lock(m_solveMutex);

    DebugInfo info;
    info.windowFull = m_windowFull;
    info.sampleCount = m_stateWindow.size();
    info.lastSolveMs = m_lastSolveMs;
    info.lastFlag = m_lastFlag;
    info.lastMaxConstraintViolation = m_lastMaxConstraintViolation;
    info.backendName = m_backend->name();
    return info;
}

void NmheEstimator::m_initializeSolverIO() {
    m_x0.assign(m_backend->inputSize(0), 0.0);
    m_p.assign(m_backend->inputSize(1), 0.0);
    m_lbx.assign(m_backend->inputSize(2), 0.0);
    m_ubx.assign(m_backend->inputSize(3), 0.0);
    m_lbg.assign(m_backend->inputSize(4), 0.0);
    m_ubg.assign(m_backend->inputSize(5), 0.0);
    m_lam_x0.assign(m_backend->inputSize(6), 0.0);
    m_lam_g0.assign(m_backend->inputSize(7), 0.0);

    m_x.assign(m_backend->outputSize(0), 0.0);
    m_f.assign(m_backend->outputSize(1), 0.0);
    m_g.assign(m_backend->outputSize(2), 0.0);
    m_lam_x.assign(m_backend->outputSize(3), 0.0);
    m_lam_g.assign(m_backend->outputSize(4), 0.0);
    m_lam_p.assign(m_backend->outputSize(5), 0.0);
}

void NmheEstimator::m_bindSolverIO() {
    m_arg[0] = m_x0.data();
    m_arg[1] = m_p.data();
    m_arg[2] = m_lbx.data();
    m_arg[3] = m_ubx.data();
    m_arg[4] = m_lbg.data();
    m_arg[5] = m_ubg.data();
    m_arg[6] = m_lam_x0.data();
    m_arg[7] = m_lam_g0.data();

    m_res[0] = m_x.data();
    m_res[1] = m_f.data();
    m_res[2] = m_g.data();
    m_res[3] = m_lam_x.data();
    m_res[4] = m_lam_g.data();
    m_res[5] = m_lam_p.data();
}

void NmheEstimator::m_packInitialGuess() {
    // Cold-started every solve (no warm start carried between solves,
    // matching run_nmhe.m's own convention -- see gcs-sitl-integration-
    // plan.md's Phase 4 status): x-part gets the ACTUAL measurement for
    // that stage (a real value we already have, better than zero), wind/d
    // parts get the current prior tiled across all M+1 stages.
    const size_t nxi = static_cast<size_t>(m_config.nxi);

    size_t stage = 0;
    for (const auto& s : m_stateWindow) {
        const size_t offset = stage * nxi;
        for (int i = 0; i < m_config.nx; ++i) {
            const double scale = m_config.invXScale.empty() ? 1.0 : m_config.invXScale[i % m_config.invXScale.size()];
            m_x0[offset + i] = s[i] * scale;
        }
        for (int i = 0; i < m_config.np; ++i) {
            const double scale = m_config.windScale.empty() ? 1.0 : 1.0 / m_config.windScale[i % m_config.windScale.size()];
            m_x0[offset + m_config.nx + i] = m_windEst[i] * scale;
        }
        for (int i = 0; i < m_config.nd; ++i) {
            const double scale = m_config.dScale.empty() ? 1.0 : 1.0 / m_config.dScale[i % m_config.dScale.size()];
            m_x0[offset + m_config.nx + m_config.np + i] = m_dEst[i] * scale;
        }
        ++stage;
    }

    assert(stage == static_cast<size_t>(m_config.M) + 1);
    assert(m_x0.size() == static_cast<size_t>(m_backend->inputSize(0)));
}

bool NmheEstimator::m_solutionIsValid(const int flag) {
    if (flag != 0) {
        m_lastMaxConstraintViolation = -1.0; // not evaluated -- solver itself failed
        return false;
    }

    constexpr double feas_tol = 5e-4;

    double max_violation = 0.0;
    for (size_t i = 0; i < m_g.size(); ++i) {
        const double v_low  = m_lbg[i] - m_g[i];
        const double v_high = m_g[i] - m_ubg[i];
        const double violation = std::max({0.0, v_low, v_high});
        max_violation = std::max(max_violation, violation);
    }
    m_lastMaxConstraintViolation = max_violation;

    if (max_violation > feas_tol) {
        return false;
    }

    for (double xi : m_x) {
        if (!std::isfinite(xi)) {
            return false;
        }
    }

    return std::isfinite(m_f[0]);
}
