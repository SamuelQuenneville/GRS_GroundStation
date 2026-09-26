/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "Definitions/vehicleStructures.h"

// Top-level abstraction over "the thing driving the vehicle(s) this tick"
// deliberately agnostic to controller family (NMPC, LMPC, TVLQR, ...),
// so ControlInterface can stay ignorant of which one is actually active.
class Controller {
public:
    virtual ~Controller() = default;

    virtual void initLaunch() = 0;

    // Feeds a fresh wind/disturbance estimate in. Physical units,
    // length np/nd matching this controller's own solverConfig. Called by
    // ControlInterface after a successful Estimator::estimate().
    virtual void setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) = 0;

    virtual void loadTrajectory(const std::string& file) = 0;
    virtual void saveTrajectory(const std::string& file) const = 0;

    // In-process equivalent of loadTrajectory(file), for a trajectory built
    // by TrajectoryGenerator rather than read from a CSV.
    virtual void setReferenceTrajectory(std::vector<double> referenceTrajectory) = 0;

    // Main entry point: convert states -> run controller -> return commands
    virtual std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates) = 0;
    [[nodiscard]] virtual double lastSolveMs() const = 0;

    struct DebugInfo {
        bool launched = false;
        bool inFlight = false;
        bool endedTraj = false;
        bool violation = false;
        double lastSolveMs = 0.0;
        size_t trackingNumber = 0;
        size_t trajectoryIndex = 0;
        size_t trajectoryTotal = 0;

        // Best available stand-ins for "Fatrop iteration count"
        // this is what's actually surfaced: the raw flag and the worst
        // constraint violation from the last solve.
        int lastFlag = 0;
        double lastMaxConstraintViolation = 0.0;
        const char* backendName = "";
    };
    [[nodiscard]] virtual DebugInfo getDebugInfo() const = 0;

    // Reference-trajectory readback for setup/orientation tooling (e.g. the
    // dashboard's 3D view). Angles in degrees.
    struct TrajectoryPointView {
        double north = 0.0, east = 0.0, down = 0.0;
        double vx = 0.0, vy = 0.0, vz = 0.0;
        double roll = 0.0, pitch = 0.0;   // stays 0 for the payload, which has no attitude state
    };

    // vehicleIndex: 0..numUavs()-1 are UAVs, numUavs() itself is the
    // payload if hasPayload() is true. Empty vector for an out-of-range index.
    [[nodiscard]] virtual std::vector<TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const = 0;
    [[nodiscard]] virtual int numUavs() const = 0;
    [[nodiscard]] virtual bool hasPayload() const = 0;
};

#endif //CONTROLLER_H
