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
    , m_io(*m_backend)
{
    m_windEst.assign(m_config.np, 0.0);
    m_dEst.assign(m_config.nd, 0.0);

    // Bounds are the same every solve (tiled from config, no state
    // dependency) -- packed once here, same as MpcController::m_packBounds()
    // does for the NMPC side.
    m_backend->packBounds(m_config, m_io.lbx, m_io.ubx);

    // g is purely the M stage-linking dynamics equalities (no path
    // constraints), so lbg = ubg = 0 for every row: NlpsolIo's zero-fill is
    // final, unlike the NMPC's alpha inequality rows.
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
    m_backend->packParameters(m_config, measurementWindow, controlWindow, m_windEst, m_dEst, m_io.p);
    m_packInitialGuess();

    int flag = 0;
    {
        PROFILE_SCOPE_OUT("nmhe_solve", &m_lastSolveMs, false);
        flag = m_io.solve(*m_backend);
        m_lastFlag = flag;
    }

    const auto check = m_io.check(flag);
    m_lastMaxConstraintViolation = check.maxConstraintViolation;
    const bool valid = check.valid;

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
            m_windEst[i] = m_io.x[windOffset + i] * scale;
        }
        for (int i = 0; i < m_config.nd; ++i) {
            const double scale = m_config.dScale.empty() ? 1.0 : m_config.dScale[i % m_config.dScale.size()];
            m_dEst[i] = m_io.x[dOffset + i] * scale;
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
            m_io.x0[offset + i] = s[i] * scale;
        }
        for (int i = 0; i < m_config.np; ++i) {
            const double scale = m_config.windScale.empty() ? 1.0 : 1.0 / m_config.windScale[i % m_config.windScale.size()];
            m_io.x0[offset + m_config.nx + i] = m_windEst[i] * scale;
        }
        for (int i = 0; i < m_config.nd; ++i) {
            const double scale = m_config.dScale.empty() ? 1.0 : 1.0 / m_config.dScale[i % m_config.dScale.size()];
            m_io.x0[offset + m_config.nx + m_config.np + i] = m_dEst[i] * scale;
        }
        ++stage;
    }

    assert(stage == static_cast<size_t>(m_config.M) + 1);
    assert(m_io.x0.size() == static_cast<size_t>(m_backend->inputSize(0)));
}
