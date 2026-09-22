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

#include <map>
#include <memory>
#include <cstring>
#include <cassert>

#include "Definitions/communicationStructures.h"
#include "Definitions/controllerStructures.h"
#include "Util/profilingTimer.h"
#include "Mathematics/math.h"
#include "Log/logger.h"

#include "controller.h"

// Solver-agnostic -- MpcController talks only to this interface, never to
// a specific codegen'd solver's global symbols, and never to that solver's
// parameter-vector layout either (see solverBackend.h -- packParameters()/
// packBounds() live there now, not in this class).
#include "SolverBackend/solverBackend.h"

class MpcController final : public Controller {
public:
    struct unwrapState {
        bool initialized = false;
        double prev = 0.0;
        double unwrapped = 0.0;
    };

    // backend is owned by this controller for its whole lifetime -- build
    // it with createSolverBackend(config.numUavs) (SolverBackendFactory.h)
    // and hand it in here.
    MpcController(const solverConfig& config, std::unique_ptr<SolverBackend> backend);
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
    // See m_unpackLatestStates(): state layout is numUavs() blocks of 8
    // (UAV: north,east,down,vN,vE,vD,roll,pitch), then -- only if this
    // trajectory's nx accounts for it -- one block of 6 for the payload
    // (no roll/pitch; it's towed, not independently attituded here).
    [[nodiscard]] bool hasPayload() const override { return m_config.nx > kUavBlockSize * m_config.numUavs; }

private:
    solverConfig m_config;
    std::unique_ptr<SolverBackend> m_backend;

    static constexpr int kUavBlockSize = 8;
    static constexpr int kPayloadBlockSize = 6;

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
    std::chrono::steady_clock::time_point m_timeAtLaunched;

    size_t m_lastIdxTraj = 0;
    size_t m_endIdxTraj = 0;
    size_t m_numTrajectoryPoints = 0;

    size_t m_pendingSteps = 0;

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

    // Solver C API pointers
    std::vector<const double*> m_arg;  // Input pointers
    std::vector<double*>       m_res;  // Output pointers

    // Solver Inputs
    std::vector<double> m_x0;
    std::vector<double> m_p;
    std::vector<double> m_lbx;
    std::vector<double> m_ubx;
    std::vector<double> m_lbg;
    std::vector<double> m_ubg;
    std::vector<double> m_lam_x0;
    std::vector<double> m_lam_g0;

    // Solver Outputs
    std::vector<double> m_x;
    std::vector<double> m_f;
    std::vector<double> m_g;
    std::vector<double> m_lam_x;
    std::vector<double> m_lam_g;
    std::vector<double> m_lam_p;

    // Solver workspace arrays, sized from m_backend->workIntSize()/
    // workRealSize() at construction (backend-specific, so no compile-time
    // SZ_IW/SZ_W constant is available here anymore).
    std::vector<long long> m_iw;
    std::vector<double>    m_w;

    // Synchronization
    mutable std::mutex m_solveMutex;

    // Timing
    double m_lastSolveMs = -1.0;

    void m_initializeSolverIO();
    void m_bindSolverIO();

    double m_computeReferenceCost(size_t idx) const;

    void m_shiftSolution();
    void m_packBounds();
    // Fills m_lbg/m_ubg with the per-stage alpha (angle-of-attack) path-
    // constraint bounds. m_initializeSolverIO() only zero-sizes m_lbg/m_ubg
    // -- left at all-zero, every g row (including the alpha inequality
    // rows) is enforced as an equality (alpha == 0), which is wrong: only
    // g's dynamics rows are equalities, the alpha rows are the NLP's one
    // genuine inequality, [-alpha_max, alpha_max] (see
    // build_nlp_oneGround_nmpc.m / build_nlp_twoUavPayload_nmpc.m). g's
    // row layout (fixed by those builders, not solver/backend-specific):
    // [nx initial-condition equality rows], then for each of the N stages,
    // [nx dynamics equality rows, numUavs alpha inequality rows] --
    // interleaved per stage on purpose, for Fatrop's structure detection.
    // Called once at construction, right after m_packBounds() -- these
    // bounds never change solve-to-solve, unlike m_lbx/m_ubx.
    void m_packInequalityBounds();
    void m_packInitialGuess();
    void m_packParameters();
    std::map<uint8_t, uavCommandsFlags> m_extractControls() const;

    bool m_solutionIsValid(int flag);

    double m_unwrapYaw(uint8_t sysId, double yawRadWrapped);

    void m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates, std::vector<double>& unpackStates);

    // Emits a LogType::NMPC_EVENT line for any of m_inFlight/m_endedTraj/
    // m_violation that changed since the last call. Called once per
    // solve(), after all three have their final value for this tick.
    void m_logTransitions();

    // Shared tail of loadTrajectory()/setReferenceTrajectory(): recomputes
    // m_numTrajectoryPoints/m_endIdxTraj from m_referenceTrajectory's size.
    void m_onReferenceTrajectoryChanged();
};

#endif //MPCCONTROLLER_H
