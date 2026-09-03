/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NMPCCONTROLLER_H
#define NMPCCONTROLLER_H

#pragma once

#include <map>
#include <cstring>
#include <cassert>

#include "Definitions/communicationStructures.h"
#include "Definitions/controllerStructures.h"
#include "Util/profilingTimer.h"
#include "Mathematics/math.h"
#include "Log/logger.h"

// CasADi-generated solver
#include "solver.h"

class NMPCController {
public:
    struct unwrapState {
        bool initialized = false;
        double prev = 0.0;
        double unwrapped = 0.0;
    };

    explicit NMPCController(const solverConfig& config);
    ~NMPCController();

    void initLaunch();

    void loadTrajectory(const std::string& file);

    // In-process equivalent of loadTrajectory(file), for a trajectory built
    // by TrajectoryGenerator (see source/Trajectory) rather than read from a
    // CSV -- ADR-001 Phase 1. `referenceTrajectory` must already be in the
    // solver's [x0 u0 x1 u1 ... xN uN] stride (TrajectoryGenerator::
    // toSolverReference() produces exactly this layout) and sampled at the
    // solver's dt; this does no resampling or validation of either.
    void setReferenceTrajectory(std::vector<double> referenceTrajectory);

    // Main entry point: convert states → run solver → return commands
    std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates);
    double lastSolveMs() const;

    // Debug/health snapshot for dashboards or logging -- deliberately a
    // plain struct here (not a dashboard type) so this header stays
    // independent of Dashboard/.
    struct DebugInfo {
        bool launched = false;
        bool inFlight = false;
        bool endedTraj = false;
        bool violation = false;
        double lastSolveMs = 0.0;
        size_t trackingNumber = 0;
        size_t trajectoryIndex = 0;
        size_t trajectoryTotal = 0;
    };
    DebugInfo getDebugInfo() const;

    // Reference-trajectory readback for setup/orientation tooling (e.g. the
    // dashboard's 3D view) -- deliberately a plain struct, same reasoning
    // as DebugInfo above. Angles in degrees.
    struct TrajectoryPointView {
        double north = 0.0, east = 0.0, down = 0.0;
        double vx = 0.0, vy = 0.0, vz = 0.0;
        double roll = 0.0, pitch = 0.0;   // stays 0 for the payload, which has no attitude state (see below)
    };

    // vehicleIndex: 0..numUavs()-1 are UAVs, numUavs() itself is the
    // payload if hasPayload() is true. Empty vector for an out-of-range index.
    std::vector<TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const;
    int numUavs() const { return m_config.numUavs; }
    // See m_unpackLatestStates(): state layout is numUavs() blocks of 8
    // (UAV: north,east,down,vN,vE,vD,roll,pitch), then -- only if this
    // trajectory's nx accounts for it -- one block of 6 for the payload
    // (no roll/pitch; it's towed, not independently attituded here).
    bool hasPayload() const { return m_config.nx > kUavBlockSize * m_config.numUavs; }

private:
    solverConfig m_config;

    static constexpr int kUavBlockSize = 8;
    static constexpr int kPayloadBlockSize = 6;

    std::vector<double> m_referenceTrajectory;
    size_t m_refStride; // nx+nu

    std::vector<double> m_initialStates;
    std::unordered_map<uint8_t, unwrapState> m_yawStates;

    bool m_launched = false;
    bool m_inFlight = false;
    bool m_endedTraj = false;
    std::chrono::steady_clock::time_point m_timeAtLaunched;

    size_t m_lastIdxTraj = 0;
    size_t m_endIdxTraj = 0;
    size_t m_numTrajectoryPoints = 0;

    size_t m_pendingSteps = 0;

    size_t m_trackingNumber = 0;

    // Solver memory handle
    int m_mem = -1;

    bool m_violation = false;

    // Solver C API pointers
    std::vector<const casadi_real*> m_arg;  // Input pointers
    std::vector<casadi_real*>       m_res;  // Output pointers

    // Solver Inputs
    std::vector<casadi_real> m_x0;
    std::vector<casadi_real> m_p;
    std::vector<casadi_real> m_lbx;
    std::vector<casadi_real> m_ubx;
    std::vector<casadi_real> m_lbg;
    std::vector<casadi_real> m_ubg;
    std::vector<casadi_real> m_lam_x0;
    std::vector<casadi_real> m_lam_g0;

    // Solver Outputs
    std::vector<casadi_real> m_x;
    std::vector<casadi_real> m_f;
    std::vector<casadi_real> m_g;
    std::vector<casadi_real> m_lam_x;
    std::vector<casadi_real> m_lam_g;
    std::vector<casadi_real> m_lam_p;

    // CasADi workspace arrays
    std::vector<casadi_int>  m_iw;
    std::vector<casadi_real> m_w;

    // Synchronization
    mutable std::mutex m_solveMutex;

    // Timing
    double m_lastSolveMs = -1.0;

    void m_initializeSolverIO();
    void m_bindSolverIO();

    double m_computeReferenceCost(size_t idx) const;

    void m_shiftSolution();
    void m_packBounds();
    void m_packInitialGuess();
    void m_packParameters();
    std::map<uint8_t, uavCommandsFlags> m_extractControls() const;

    bool m_solutionIsValid(int flag);

    double m_unwrapYaw(uint8_t sysId, double yawRadWrapped);

    void m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates, std::vector<double>& unpackStates);

    // Shared tail of loadTrajectory()/setReferenceTrajectory(): recomputes
    // m_numTrajectoryPoints/m_endIdxTraj from m_referenceTrajectory's size.
    void m_onReferenceTrajectoryChanged();
};

#endif //NMPCCONTROLLER_H
