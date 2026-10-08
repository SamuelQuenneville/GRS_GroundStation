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
#include "fatropStatus.h"

// Computes the commands of all vehicles each control tick, whatever the controller family (NMPC and LMPC today; TVLQR...).
class Controller {
public:
    virtual ~Controller() = default;

    virtual void initLaunch() = 0;

    // Whether a launch may start now; reason says why not.
    [[nodiscard]] virtual bool launchReady(std::string& reason) const = 0;
    // Launched and flying (speed threshold crossed since the launch).
    [[nodiscard]] virtual bool inFlight() const = 0;

    // Latest wind/disturbance estimate, physical units, lengths np/nd.
    virtual void setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) = 0;

    virtual void loadTrajectory(const std::string& file) = 0;
    virtual void saveTrajectory(const std::string& file) const = 0;

    // Same as loadTrajectory() for a reference built in memory.
    virtual void setReferenceTrajectory(std::vector<double> referenceTrajectory) = 0;

    // time: when latestStates were sampled, in seconds on the caller's clock
    // (steady clock in the GCS, simulated time in grs_batchsim). The
    // reference advances with it, whatever the tick rate.
    virtual std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates, double time) = 0;

    struct DebugInfo {
        bool launched = false;
        bool inFlight = false;
        bool endedTraj = false;
        bool violation = false;
        double lastSolveMs = 0.0;
        size_t trackingNumber = 0;
        size_t trajectoryIndex = 0;
        size_t trajectoryTotal = 0;

        // Last solve: generated solver flag (non-zero on evaluation errors only), Fatrop outcome and worst
        // constraint violation.
        int lastFlag = 0;
        FatropStatus lastFatrop;
        double lastMaxConstraintViolation = 0.0;
        const char* backendName = "";
    };
    [[nodiscard]] virtual DebugInfo getDebugInfo() const = 0;

    // Reference trajectory for display (dashboard 3D view). Angles in degrees.
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
