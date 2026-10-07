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
#include <stdexcept>
#include <string>

#include "Util/profilingTimer.h"

namespace {

// Entry i of a scale vector, repeated if shorter; 1 when empty.
double scaleAt(const std::vector<double>& scale, const int i) {
    return scale.empty() ? 1.0 : scale[i % scale.size()];
}

} // namespace

NmheEstimator::NmheEstimator(const estimatorConfig& config)
    : m_config(config)
    , m_solver(GeneratedFunction::Id::Nmhe, config.numUavs)
{
    // Layout of m_packParameters() and of the decision vector. A mismatch
    // means EstimatorConfiguration does not describe the generated NMHE.
    auto require = [this](const bool ok, const std::string& what) {
        if (!ok) throw std::runtime_error(std::string(m_solver.name()) + ": " + what + " (EstimatorConfiguration vs generated NMHE)");
    };
    const auto& c = m_config;
    require(c.nxi == c.nx + c.np + c.nd, "NXI != NX + NP + ND");
    require(c.wMeas.size() == static_cast<size_t>(c.nx) && c.wWindPrior.size() == static_cast<size_t>(c.np)
            && c.wDPrior.size() == static_cast<size_t>(c.nd), "W_MEAS/W_WINDP/W_DP sizes");
    require(m_solver.x0.size() == (c.M + 1) * c.nxi,
            "takes " + std::to_string(m_solver.x0.size()) + " decision variables, expected (M+1)*NXI = " + std::to_string((c.M + 1) * c.nxi));
    const size_t np = (c.M + 1) * c.nx + c.M * c.nu + 2 * (c.np + c.nd) + c.nx + c.nL0;
    require(m_solver.p.size() == np,
            "takes " + std::to_string(m_solver.p.size()) + " parameters, expected " + std::to_string(np));

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
    ++m_samplesSinceSolve;

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
    m_samplesSinceSolve = 0;

    {
        PROFILE_SCOPE_OUT("nmhe_solve", &m_lastSolveMs, false);
        m_lastStatus = m_solver.solve();
    }

    const auto check = m_solver.check(m_lastStatus);
    m_lastMaxConstraintViolation = check.maxConstraintViolation;
    const bool valid = check.valid;

    if (valid) {
        // Wind and d are constant over the window (identity dynamics), so
        // any stage holds the estimate; read the last one.
        const auto nxi = static_cast<size_t>(m_config.nxi);
        const size_t lastStageOffset = static_cast<size_t>(m_config.M) * nxi;
        const size_t windOffset = lastStageOffset + m_config.nx;
        const size_t dOffset = windOffset + m_config.np;

        for (int i = 0; i < m_config.np; ++i) {
            m_windEst[i] = m_solver.x[windOffset + i] * scaleAt(m_config.windScale, i);
        }
        for (int i = 0; i < m_config.nd; ++i) {
            m_dEst[i] = m_solver.x[dOffset + i] * scaleAt(m_config.dScale, i);
        }
        m_prevSolution = m_solver.x;
    } else {
        m_prevSolution.clear();
    }

    return valid;
}

void NmheEstimator::reset() {
    std::lock_guard lock(m_solveMutex);
    m_stateWindow.clear();
    m_controlWindow.clear();
    std::ranges::fill(m_windEst, 0.0);
    std::ranges::fill(m_dEst, 0.0);
    m_prevSolution.clear();
    m_samplesSinceSolve = 0;
    m_windowFull = false;
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
    info.lastFlag = m_lastStatus.flag;
    info.lastFatrop = m_lastStatus.fatrop;
    info.lastMaxConstraintViolation = m_lastMaxConstraintViolation;
    info.backendName = m_solver.name();
    return info;
}

void NmheEstimator::m_packInitialGuess() {
    const auto nxi = static_cast<size_t>(m_config.nxi);
    const auto M = static_cast<size_t>(m_config.M);
    const size_t shift = m_samplesSinceSolve;
    const bool warm = !m_prevSolution.empty() && shift <= M;

    for (size_t k = 0; k <= M; ++k) {
        double* xi = m_solver.x0.data() + k * nxi;
        if (warm) {
            // Stage k of the new window is stage k + shift of the previous
            // one; past its end, the previous last stage.
            std::copy_n(m_prevSolution.data() + std::min(k + shift, M) * nxi, nxi, xi);
        } else {
            for (int i = 0; i < m_config.np; ++i) {
                xi[m_config.nx + i] = m_windEst[i] / scaleAt(m_config.windScale, i);
            }
            for (int i = 0; i < m_config.nd; ++i) {
                xi[m_config.nx + m_config.np + i] = m_dEst[i] / scaleAt(m_config.dScale, i);
            }
        }
        // Measured x on every stage cold, on the new stages warm.
        if (!warm || k + shift > M) {
            const auto& s = m_stateWindow[k];
            for (int i = 0; i < m_config.nx; ++i) {
                xi[i] = s[i] * scaleAt(m_config.invXScale, i);
            }
        }
    }
}

void NmheEstimator::m_packBounds() {
    constexpr double inf = std::numeric_limits<double>::infinity();
    std::ranges::fill(m_solver.lbx, -inf);
    std::ranges::fill(m_solver.ubx, inf);
    for (int i = 0; i < m_config.np; ++i) {
        const double b = m_config.windBound[i] / scaleAt(m_config.windScale, i);
        m_solver.lbx[m_config.nx + i] = -b;
        m_solver.ubx[m_config.nx + i] = b;
    }
    for (int i = 0; i < m_config.nd; ++i) {
        const double b = m_config.dBound[i] / scaleAt(m_config.dScale, i);
        m_solver.lbx[m_config.nx + m_config.np + i] = -b;
        m_solver.ubx[m_config.nx + m_config.np + i] = b;
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
