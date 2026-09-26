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
#include <limits>

#include "Util/profilingTimer.h"

namespace {

// Entry i of a scale vector, repeated if shorter; 1 when empty.
double scaleAt(const std::vector<double>& scale, const int i) {
    return scale.empty() ? 1.0 : scale[i % scale.size()];
}

} // namespace

NmheEstimator::NmheEstimator(const estimatorConfig& config)
    : m_config(config)
    , m_solver(Nlpsol::Problem::Nmhe, config.numUavs)
{
    m_windEst.assign(m_config.np, 0.0);
    m_dEst.assign(m_config.nd, 0.0);

    m_packBounds();
    // g holds only the dynamics equalities: lbg = ubg = 0 as constructed.
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

    // m_windEst/m_dEst still hold the previous estimate: this solve's
    // arrival-cost prior.
    m_packParameters();
    m_packInitialGuess();

    int flag = 0;
    {
        PROFILE_SCOPE_OUT("nmhe_solve", &m_lastSolveMs, false);
        flag = m_solver.solve();
        m_lastFlag = flag;
    }

    const auto check = m_solver.check(flag);
    m_lastMaxConstraintViolation = check.maxConstraintViolation;
    const bool valid = check.valid;

    if (valid) {
        // Wind and d are constant over the window (identity dynamics), so
        // any stage holds the estimate; read the last one.
        const size_t nxi = static_cast<size_t>(m_config.nxi);
        const size_t lastStageOffset = static_cast<size_t>(m_config.M) * nxi;
        const size_t windOffset = lastStageOffset + m_config.nx;
        const size_t dOffset = windOffset + m_config.np;

        for (int i = 0; i < m_config.np; ++i) {
            m_windEst[i] = m_solver.x[windOffset + i] * scaleAt(m_config.windScale, i);
        }
        for (int i = 0; i < m_config.nd; ++i) {
            m_dEst[i] = m_solver.x[dOffset + i] * scaleAt(m_config.dScale, i);
        }
    }

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
    info.backendName = m_solver.name();
    return info;
}

void NmheEstimator::m_packInitialGuess() {
    // Cold start: each stage's measurement, and the prior for wind and d.
    const size_t nxi = static_cast<size_t>(m_config.nxi);

    size_t stage = 0;
    for (const auto& s : m_stateWindow) {
        const size_t offset = stage * nxi;
        for (int i = 0; i < m_config.nx; ++i) {
            m_solver.x0[offset + i] = s[i] * scaleAt(m_config.invXScale, i);
        }
        for (int i = 0; i < m_config.np; ++i) {
            m_solver.x0[offset + m_config.nx + i] = m_windEst[i] / scaleAt(m_config.windScale, i);
        }
        for (int i = 0; i < m_config.nd; ++i) {
            m_solver.x0[offset + m_config.nx + m_config.np + i] = m_dEst[i] / scaleAt(m_config.dScale, i);
        }
        ++stage;
    }

    assert(stage == static_cast<size_t>(m_config.M) + 1);
}

void NmheEstimator::m_packBounds() {
    constexpr double inf = std::numeric_limits<double>::infinity();
    const double dBound[] = {m_config.dFMax, m_config.dFMax, m_config.dFMax, m_config.bAttMax, m_config.bAttMax};

    std::vector<double> ub(m_config.nxi, inf);
    for (int i = 0; i < m_config.np; ++i) {
        ub[m_config.nx + i] = m_config.windMax / scaleAt(m_config.windScale, i);
    }
    for (int i = 0; i < m_config.nd; ++i) {
        ub[m_config.nx + m_config.np + i] = dBound[i % 5] / scaleAt(m_config.dScale, i);
    }

    for (int k = 0; k <= m_config.M; ++k) {
        for (int i = 0; i < m_config.nxi; ++i) {
            m_solver.lbx[k * m_config.nxi + i] = -ub[i];
            m_solver.ubx[k * m_config.nxi + i] = ub[i];
        }
    }
}

void NmheEstimator::m_packParameters() {
    auto dst = m_solver.p.begin();
    for (size_t k = 0; k < m_stateWindow.size(); ++k) {
        dst = std::ranges::copy(m_stateWindow[k], dst).out;
        if (k < m_controlWindow.size()) {
            dst = std::ranges::copy(m_controlWindow[k], dst).out;
        }
    }
    for (const auto* v : {&m_windEst, &m_dEst, &m_config.wMeas, &m_config.wWindPrior, &m_config.wDPrior}) {
        dst = std::ranges::copy(*v, dst).out;
    }
    dst = std::fill_n(dst, m_config.nL0, m_config.tetherL0);
    assert(dst == m_solver.p.end());
}
