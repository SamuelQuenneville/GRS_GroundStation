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
#include <memory>
#include <mutex>
#include <vector>

#include "Definitions/controllerStructures.h"
#include "estimator.h"

// Backend-agnostic -- NmheEstimator talks only to this interface, never to
// a specific codegen'd solver's global symbols or parameter-vector layout
// (see estimatorBackend.h), same isolation MpcController/SolverBackend has.
#include "SolverBackend/estimatorBackend.h"
#include "SolverBackend/nlpsolIo.h"

class NmheEstimator final : public Estimator {
public:
    // backend is owned by this estimator for its whole lifetime -- build
    // it with createEstimatorBackend(config.numUavs)
    // (estimatorBackendFactory.h) and hand it in here.
    NmheEstimator(const estimatorConfig& config, std::unique_ptr<EstimatorBackend> backend);
    ~NmheEstimator() override;

    void addSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) override;
    bool estimate() override;

    [[nodiscard]] const std::vector<double>& windEstimate() const override;
    [[nodiscard]] const std::vector<double>& dEstimate() const override;

    [[nodiscard]] DebugInfo getDebugInfo() const override;

private:
    estimatorConfig m_config;
    std::unique_ptr<EstimatorBackend> m_backend;

    // Sliding window, oldest at front -- stateWindow holds M+1 samples once
    // full, controlWindow holds M (controlWindow[k] applied going from
    // stateWindow[k] to stateWindow[k+1]). Capped in addSample().
    std::deque<std::vector<double>> m_stateWindow;
    std::deque<std::vector<double>> m_controlWindow;

    // Current best estimate AND the arrival-cost prior fed into the next
    // solve -- see estimate()'s own comment on why these double as both.
    // Zero until the first successful solve (a reasonable cold-start prior,
    // matching run_nmhe.m's own cold-start convention).
    std::vector<double> m_windEst;
    std::vector<double> m_dEst;

    // Solver inputs/outputs/workspaces (see nlpsolIo.h). Declared after
    // m_backend, which sizes it.
    NlpsolIo m_io;

    mutable std::mutex m_solveMutex;

    bool m_windowFull = false;
    double m_lastSolveMs = 0.0;
    int m_lastFlag = 0;
    double m_lastMaxConstraintViolation = 0.0;

    void m_packInitialGuess();
};

#endif //NMHEESTIMATOR_H
