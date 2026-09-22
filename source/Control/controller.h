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

#include "Definitions/communicationStructures.h"

// Top-level abstraction over "the thing driving the vehicle(s) this tick"
// -- deliberately agnostic to controller family (NMPC, LMPC, TVLQR, ...),
// so ControlInterface can stay ignorant of which one is actually active.
// See gcs-sitl-integration-plan.md §3a for the reasoning behind the split.
//
// MpcController is the first (and, as of this writing, only) implementation
// -- it drives any NLP-based family through SolverBackend (NMPC today,
// LMPC eventually, since both compile to a standard nlpsol C API -- see
// solverBackend.h). A future TvlqrController would be a second
// implementation with no SolverBackend at all: TVLQR is a Riccati/
// gain-schedule design, not an NLP solve, so it has no `nlpsol` C API to
// wrap and no parameter vector to pack (see gcs-sitl-integration-plan.md
// §3 for why it needed no symbol-collision fix either). Not built yet --
// there's no TVLQR GCS integration to drive it (see the plan's Scope), so
// writing it now would mean guessing at its shape instead of deriving it
// from an actual gain-schedule format.
class Controller {
public:
    virtual ~Controller() = default;

    virtual void initLaunch() = 0;

    // Feeds a fresh wind/disturbance estimate in, replacing whatever was
    // packed before (zero, until an estimator exists -- see the wind/
    // disturbance gap in gcs-sitl-integration-plan.md §2). Physical units,
    // length np/nd matching this controller's own solverConfig. Called by
    // ControlInterface after a successful Estimator::estimate() (see
    // estimator.h), on the estimator's own cadence -- NOT once per control
    // tick -- so the value packed into the NEXT few solve()s is simply
    // whatever was set here last (zero-order hold), same convention
    // Estimator::windEstimate()/dEstimate() themselves use.
    virtual void setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) = 0;

    // Inverse of loadTrajectory(file): writes the current in-memory
    // reference trajectory back out to `file`. Throws if no trajectory has
    // been loaded/generated yet, or if `file` can't be opened for writing.
    virtual void loadTrajectory(const std::string& file) = 0;
    virtual void saveTrajectory(const std::string& file) const = 0;

    // In-process equivalent of loadTrajectory(file), for a trajectory built
    // by TrajectoryGenerator (see source/Trajectory) rather than read from a
    // CSV -- ADR-001 Phase 1.
    virtual void setReferenceTrajectory(std::vector<double> referenceTrajectory) = 0;

    // Main entry point: convert states -> run controller -> return commands
    virtual std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates) = 0;
    [[nodiscard]] virtual double lastSolveMs() const = 0;

    // Debug/health snapshot for dashboards or logging -- deliberately a
    // plain struct here (not a dashboard type) so this header stays
    // independent of Dashboard/. lastFlag/lastMaxConstraintViolation/
    // backendName are NLP-solve concepts (see SolverBackend.h) that a
    // non-NLP implementation (TVLQR) would simply leave at their defaults
    // -- dashboards should treat an empty backendName as "not applicable",
    // not "unknown error".
    struct DebugInfo {
        bool launched = false;
        bool inFlight = false;
        bool endedTraj = false;
        bool violation = false;
        double lastSolveMs = 0.0;
        size_t trackingNumber = 0;
        size_t trajectoryIndex = 0;
        size_t trajectoryTotal = 0;

        // Best available stand-ins for "Fatrop iteration count" -- the
        // bare codegen'd C API (see solverBackend.h) only ever returns the
        // pass/fail flag, not iteration counts; nlpsol's own .stats() with
        // that detail is a C++/Python-only interface, not something
        // solver.generate() emits into the C solve() call. Until/unless
        // that's worth building (e.g. a custom Fatrop stats callback),
        // this is what's actually surfaced: the raw flag and the worst
        // constraint violation from the last solve.
        int lastFlag = 0;
        double lastMaxConstraintViolation = 0.0;
        const char* backendName = "";
    };
    [[nodiscard]] virtual DebugInfo getDebugInfo() const = 0;

    // Reference-trajectory readback for setup/orientation tooling (e.g. the
    // dashboard's 3D view) -- deliberately a plain struct, same reasoning
    // as DebugInfo above. Angles in degrees.
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
