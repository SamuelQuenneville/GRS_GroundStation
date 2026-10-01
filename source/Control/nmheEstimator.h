/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NMHEESTIMATOR_H
#define NMHEESTIMATOR_H

#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include "Definitions/controllerStructures.h"
#include "estimator.h"
#include "nlpsol.h"

class NmheEstimator final : public Estimator {
public:
    explicit NmheEstimator(const estimatorConfig& config);
    ~NmheEstimator() override;

    void addSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) override;
    bool estimate() override;

    [[nodiscard]] const std::vector<double>& windEstimate() const override;
    [[nodiscard]] const std::vector<double>& dEstimate() const override;

    [[nodiscard]] DebugInfo getDebugInfo() const override;

private:
    estimatorConfig m_config;
    Nlpsol m_solver;

    // Oldest first: M+1 states and M controls once full; controlWindow[k] goes from stateWindow[k] to stateWindow[k+1].
    std::deque<std::vector<double>> m_stateWindow;
    std::deque<std::vector<double>> m_controlWindow;

    // Last valid estimate, also the arrival-cost prior of the next solve.
    std::vector<double> m_windEst;
    std::vector<double> m_dEst;

    // Last valid solution (scaled), the warm start of the next solve, and
    // the samples added since the window it was solved on.
    std::vector<double> m_prevSolution;
    size_t m_samplesSinceSolve = 0;

    mutable std::mutex m_solveMutex;

    bool m_windowFull = false;
    double m_lastSolveMs = 0.0;
    Nlpsol::Status m_lastStatus;
    double m_lastMaxConstraintViolation = 0.0;

    // Warm start (shift_nmhe.m): the previous solution moved back by the
    // samples added since, the new stages with the measured x and the last
    // wind/d. Cold start (x measured, wind/d from the prior) without a
    // previous solution or when the whole window is new.
    void m_packInitialGuess();
    // Per stage [x; wind; d], scaled: wind and d bounded on the first stage
    // (windBound, dBound), everything else free.
    void m_packBounds();
    // P_optim (build_nmhe_*.m): [Xmeas_1 Uapp_1 ... Xmeas_M Uapp_M Xmeas_M+1; Wind_prior; D_prior; W_meas; W_windp; W_dp; L0].
    void m_packParameters();
};

#endif //NMHEESTIMATOR_H
