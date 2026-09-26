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

    mutable std::mutex m_solveMutex;

    bool m_windowFull = false;
    double m_lastSolveMs = 0.0;
    int m_lastFlag = 0;
    double m_lastMaxConstraintViolation = 0.0;

    void m_packInitialGuess();
    // Per stage [x; wind; d], scaled: x unbounded, wind by windMax, d by
    // dFMax (force) and bAttMax (attitude bias), per UAV.
    void m_packBounds();
    // P_optim (build_nmhe_*.m): [Xmeas_1 Uapp_1 ... Xmeas_M Uapp_M Xmeas_M+1; Wind_prior; D_prior; W_meas; W_windp; W_dp; L0].
    void m_packParameters();
};

#endif //NMHEESTIMATOR_H
