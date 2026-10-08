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
#include "stateVector.h"

// One tick of the MPC control mode, shared by the GCS and grs_batchsim:
// hand the controller the newest NMHE estimate, solve the NMPC, push this
// tick's sample to the estimator (in flight only). The NMHE solves elsewhere, through the
// EstimatorRunner; a tick never waits for it.
//
// The controller and estimator are owned by the caller. The runner is owned
// here and must be destroyed before the estimator.
class ControlStep {
public:
    // runner may be null (no EstimatorConfiguration in the YAML).
    // estimatorNu is the estimator's joint control dimension (ignored
    // without a runner).
    ControlStep(Controller& controller, std::unique_ptr<EstimatorRunner> runner, int estimatorNu);

    // navStates: telemetry in the NED frame, by sysId (stateVector.h).
    // time: when navStates were sampled (see Controller::solve()).
    // Returns commands in physical units: thrust in N, attitude in degrees.
    std::map<uint8_t, uavCommandsFlags> tick(const std::map<uint8_t, uavStates>& navStates, double time);

    [[nodiscard]] const EstimatorRunner* estimatorRunner() const { return m_runner.get(); }
    // Estimate the controller is currently using (empty until the first one).
    [[nodiscard]] const EstimatorRunner::Estimate& appliedEstimate() const { return m_appliedEstimate; }

private:
    Controller& m_controller;
    std::unique_ptr<EstimatorRunner> m_runner;

    // Previous tick's command: the control applied up to this tick's sample.
    std::vector<double> m_appliedControl;
    EstimatorRunner::Estimate m_appliedEstimate;
    bool m_wasInFlight = false;

    // Estimator sample, same layout as the controller's state (stateVector.h).
    // Kept between ticks: a vehicle without telemetry keeps its last value.
    grs::control::StateLayout m_layout;
    std::vector<double> m_measuredState;
};

// Controller and optional estimator built from one YAML profile.
struct ControlStack {
    solverConfig solver;
    std::optional<estimatorConfig> estimator;
    std::unique_ptr<Controller> controller;
    std::unique_ptr<Estimator> estimatorInstance;  // null if estimator is nullopt
};

// withEstimator=false skips the estimator even if the YAML configures one.
ControlStack buildControlStack(YAML::Node& node, bool withEstimator = true);

#endif //CONTROLSTEP_H
