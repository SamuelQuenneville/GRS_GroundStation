/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#pragma once

#include <chrono>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <cstring>
#include <cassert>

#include "Definitions/vehicleStructures.h"
#include "Definitions/controllerStructures.h"
#include "Util/profilingTimer.h"
#include "Mathematics/math.h"
#include "Log/logger.h"

#include "controller.h"

#include "nlpsol.h"
#include "stateVector.h"

class MpcController final : public Controller {
public:
    struct unwrapState {
        bool initialized = false;
        double prev = 0.0;
        double unwrapped = 0.0;
    };

    explicit MpcController(const solverConfig& config);
    ~MpcController() override;

    void initLaunch() override;

    void setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) override;

    void loadTrajectory(const std::string& file) override;

    // Inverse of loadTrajectory(file): writes the current in-memory
    // m_referenceTrajectory back out to `file`, one line per trajectory
    // point, m_refStride comma-separated raw solver values per line (same
    // radians/units TrajectoryGenerator::toSolverReference() produces --
    // NOT the degree-converted values getTrajectoryForVehicle() returns
    // for display) so the file round-trips through loadTrajectory()
    // unchanged. Throws if no trajectory has been loaded/generated yet, or
    // if `file` can't be opened for writing.
    void saveTrajectory(const std::string& file) const override;

    // In-process equivalent of loadTrajectory(file), for a trajectory built
    // by TrajectoryGenerator (see source/Trajectory) rather than read from a
    // CSV -- ADR-001 Phase 1. `referenceTrajectory` must already be in the
    // solver's [x0 u0 x1 u1 ... xN uN] stride (TrajectoryGenerator::
    // toSolverReference() produces exactly this layout) and sampled at the
    // solver's dt; this does no resampling or validation of either.
    void setReferenceTrajectory(std::vector<double> referenceTrajectory) override;

    // Main entry point: convert states → run solver → return commands
    std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates) override;
    [[nodiscard]] double lastSolveMs() const override;

    [[nodiscard]] DebugInfo getDebugInfo() const override;

    // vehicleIndex: 0..numUavs()-1 are UAVs, numUavs() itself is the
    // payload if hasPayload() is true. Empty vector for an out-of-range index.
    [[nodiscard]] std::vector<TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const override;
    [[nodiscard]] int numUavs() const override { return m_config.numUavs; }
    // State layout: see stateVector.h. The payload block exists only if nx
    // accounts for it.
    [[nodiscard]] bool hasPayload() const override { return m_config.nx > grs::control::kUavBlockSize * m_config.numUavs; }

private:
    solverConfig m_config;
    Nlpsol m_solver;

    grs::control::StateLayout m_layout;

    std::vector<double> m_referenceTrajectory;
    size_t m_refStride; // nx+nu

    std::vector<double> m_initialStates;
    std::unordered_map<uint8_t, unwrapState> m_yawStates;

    bool m_launched = false;
    bool m_inFlight = false;
    bool m_endedTraj = false;

    // Previous tick's value of the flags above/m_violation below, so
    // solve() can emit a sparse NMPC_EVENT log line only on a transition
    // (see m_logTransitions() in the .cpp) instead of every tick.
    bool m_prevInFlight = false;
    bool m_prevEndedTraj = false;
    bool m_prevViolation = false;
    // False while some vehicle of the layout sent no telemetry this tick
    // (its block then keeps its previous value, see m_unpackLatestStates()).
    bool m_telemetryComplete = true;
    bool m_prevTelemetryComplete = true;
    std::chrono::steady_clock::time_point m_timeAtLaunched;

    size_t m_lastIdxTraj = 0;
    size_t m_endIdxTraj = 0;
    size_t m_numTrajectoryPoints = 0;

    size_t m_pendingSteps = 0;
    size_t m_solvesSinceLaunch = 0; // ReferenceIndexing::Time only

    size_t m_trackingNumber = 0;

    bool m_violation = false;
    int m_lastFlag = 0;
    double m_lastMaxConstraintViolation = 0.0;

    // Previously-applied control [T, roll, pitch] per UAV, physical units
    // -- packed into the solver's U_prev parameter every solve so the
    // dU0 (first-stage control-rate) cost term has something to compare
    // against. Rdu0 is 0 by default in the shipped config (see
    // configuration.yaml), so this is inert until Rdu0 is tuned, but it's
    // packed correctly from day one rather than left as a TODO.
    std::vector<double> m_uPrev;

    // Latest wind/disturbance estimate, physical units, length
    // config.np/config.nd -- zero until an Estimator is configured and
    // calls setDisturbanceEstimate() (Phase 4). Zero-order held between
    // calls, same convention Estimator::windEstimate()/dEstimate() use
    // (see estimator.h) -- set on the estimator's own cadence, not once
    // per solve() here.
    std::vector<double> m_windEst;
    std::vector<double> m_dEst;
    mutable std::mutex m_disturbanceMutex;

    // Synchronization
    mutable std::mutex m_solveMutex;

    // Timing
    double m_lastSolveMs = -1.0;

    double m_computeReferenceCost(size_t idx) const;

    void m_shiftSolution();
    // Decision-variable bounds, scaled: [x0 u0 ... x(N-1) u(N-1) xN].
    void m_packBounds();
    // g rows (build_nlp_*_nmpc.m): nx initial-condition equalities, then per
    // stage nx dynamics equalities followed by numUavs angle-of-attack rows.
    // Only the alpha rows are inequalities, [-alphaMax, alphaMax].
    void m_packInequalityBounds();
    void m_packInitialGuess();
    // P_optim (build_nlp_*_nmpc.m): [x_initial; reference window x0 u0 ... xN;
    // Wind_est; D_est; Weight; U_prev; L0].
    void m_packParameters();
    std::map<uint8_t, uavCommandsFlags> m_extractControls() const;

    // Sets m_violation / m_lastMaxConstraintViolation from m_solver.check().
    bool m_solutionIsValid(int flag);

    double m_unwrapYaw(uint8_t sysId, double yawRadWrapped);

    // Fills m_initialStates from telemetry (stateVector.h), then applies the
    // pre-flight substitution: until launched and in flight, UAV position and
    // velocity come from the reference's first sample.
    void m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates);

    // Emits a LogType::NMPC_EVENT line for any of m_telemetryComplete/
    // m_inFlight/m_endedTraj/m_violation that changed since the last call.
    // Called once per solve(), after all of them have their final value.
    void m_logTransitions();

    // Shared tail of loadTrajectory()/setReferenceTrajectory(): recomputes
    // m_numTrajectoryPoints/m_endIdxTraj from m_referenceTrajectory's size.
    void m_onReferenceTrajectoryChanged();
};

#endif //MPCCONTROLLER_H
