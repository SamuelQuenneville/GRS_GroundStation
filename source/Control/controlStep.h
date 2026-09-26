/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLSTEP_H
#define CONTROLSTEP_H

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "Definitions/controllerStructures.h"
#include "Definitions/vehicleStructures.h"
#include "controller.h"
#include "estimator.h"
#include "estimatorRunner.h"
#include "Configuration/configurationParser.h"
#include "SolverBackend/estimatorBackendFactory.h"
#include "SolverBackend/solverBackendFactory.h"

// One control-loop tick of the MPC control mode: hand the controller the
// newest finished NMHE estimate, push this tick's sample to the estimator,
// solve the NMPC. Pulled out of ControlInterface::m_controlLoop() so the live
// GCS and grs_batchsim run the exact same per-tick logic -- only what
// surrounds a tick differs (wall-clock pacing, MAVLink, thrust->rpm wire
// conversion on the GCS side; a truth plant and a simulated clock on the
// batch-sim side).
//
// The NMHE is never solved here: it runs through an EstimatorRunner (see
// estimatorRunner.h), on its own thread in the GCS
// (ThreadedEstimatorRunner) or as a deterministic emulation of that thread
// in grs_batchsim (DeferredEstimatorRunner). A tick never waits for an NMHE
// solve.
//
// Does not own the controller/estimator -- ControlInterface keeps owning
// them (its accessors use them directly), grs_batchsim owns them through a
// ControlStack. It owns the runner, which must be destroyed before the
// estimator (a ThreadedEstimatorRunner joins its thread on destruction).
class ControlStep {
public:
    // runner may be null (no EstimatorConfiguration in the YAML).
    // estimatorNu is the estimator's joint control dimension (ignored
    // without a runner).
    ControlStep(Controller& controller, std::unique_ptr<EstimatorRunner> runner, int estimatorNu);

    // navStates: telemetry already in the navigation (NED) frame, same map
    // m_controlLoop() hands the controller (UAVs as sysId 1..numUavs, the
    // payload, if any, above that).
    // Returns the controller's commands in physical units (thrust in N,
    // attitude in degrees) -- no thrust->rpm conversion here.
    std::map<uint8_t, uavCommandsFlags> tick(const std::map<uint8_t, uavStates>& navStates);

    // True if this tick handed the controller a new estimate.
    [[nodiscard]] bool estimateAppliedThisTick() const { return m_estimateAppliedThisTick; }
    [[nodiscard]] const EstimatorRunner* estimatorRunner() const { return m_runner.get(); }
    // Estimate the controller is currently using (empty until the first one).
    [[nodiscard]] const EstimatorRunner::Estimate& appliedEstimate() const { return m_appliedEstimate; }

private:
    Controller& m_controller;
    std::unique_ptr<EstimatorRunner> m_runner;

    // Control applied over the interval that ENDS at the current sample,
    // i.e. the previous tick's command -- see tick().
    std::vector<double> m_appliedControl;
    bool m_estimateAppliedThisTick = false;
    EstimatorRunner::Estimate m_appliedEstimate;

    void m_buildEstimatorStateVector(const std::map<uint8_t, uavStates>& states, std::vector<double>& out) const;
};

// Controller + optional estimator built from one YAML profile, the same way
// ControlInterface::initialize() builds them (backend picked from NUM_UAVS).
struct ControlStack {
    solverConfig solver;
    std::optional<estimatorConfig> estimator;
    std::unique_ptr<Controller> controller;
    std::unique_ptr<Estimator> estimatorInstance;  // null if estimator is nullopt
};

// withEstimator=false skips building the estimator even if the YAML has an
// EstimatorConfiguration section (grs_batchsim's "naive" controller variant).
ControlStack buildControlStack(YAML::Node& node, bool withEstimator = true);

#endif //CONTROLSTEP_H
