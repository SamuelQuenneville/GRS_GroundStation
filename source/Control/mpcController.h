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

#include <limits>
#include <mutex>
#include <optional>
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
    [[nodiscard]] bool launchReady(std::string& reason) const override;
    [[nodiscard]] bool inFlight() const override { return m_inFlight; }

    void setDisturbanceEstimate(const std::vector<double>& wind, const std::vector<double>& d) override;

    void loadTrajectory(const std::string& file) override;

    // Inverse of loadTrajectory(): one line per point, m_refStride raw solver
    // values (radians, not the degrees getTrajectoryForVehicle() returns).
    // Throws if no trajectory is loaded or the file can't be written.
    void saveTrajectory(const std::string& file) const override;

    // Same as loadTrajectory() for an in-memory reference, already in the
    // solver's [x0 u0 x1 u1 ...] stride and sampled at its dt (as
    // TrajectoryGenerator::toSolverReference() produces). Not resampled or
    // validated.
    void setReferenceTrajectory(std::vector<double> referenceTrajectory) override;

    std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates, double time) override;
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
    // LMPC only: P_lin, the per-stage affine model about the reference window.
    std::optional<GeneratedFunction> m_linearization;

    grs::control::StateLayout m_layout;

    std::vector<double> m_referenceTrajectory;
    size_t m_refStride; // nx+nu

    // Decision vector [z0 u0 ... z(N-1) u(N-1) zN], stage state z = [x; up],
    // up the previous control (build_nlp_*_nmpc.m, build_nlp_*_lmpc.m).
    size_t m_nz = 0;        // nx+nu
    size_t m_solStride = 0; // nz+nu, one stage [z u]

    std::vector<double> m_initialStates;
    std::unordered_map<uint8_t, unwrapState> m_yawStates;

    bool m_launched = false;
    // Largest UAV distance to its reference start, measured before launch.
    double m_launchPositionError = std::numeric_limits<double>::infinity();
    bool m_inFlight = false;
    bool m_endedTraj = false;
    // Past the last full window: no solve, the last applied control is repeated
    // (the trajectory ends with a buffer in which the pilot takes over).
    bool m_holding = false;
    // In flight with no plan left (N rejected solves): reference feedforward, open loop.
    bool m_openLoop = false;

    // Previous values, to log transitions only (m_logTransitions()).
    bool m_prevInFlight = false;
    bool m_prevEndedTraj = false;
    bool m_prevViolation = false;
    bool m_prevHolding = false;
    bool m_prevOpenLoop = false;
    // False while some vehicle of the layout sent no telemetry this tick
    // (its block then keeps its previous value, see m_unpackLatestStates()).
    bool m_telemetryComplete = true;
    bool m_prevTelemetryComplete = true;

    // Reference time: time since the first solve after launch, on the
    // caller's clock. Sample floor(t/dt) and the window interpolated at t.
    std::optional<double> m_launchTime;
    double m_referenceTime = 0.0;
    size_t m_lastIdxTraj = 0;
    std::vector<double> m_referenceWindow; // [x u] x N, then x
    size_t m_endIdxTraj = 0;               // number of full windows
    size_t m_numTrajectoryPoints = 0;

    size_t m_trackingNumber = 0;

    bool m_violation = false;
    Nlpsol::Status m_lastStatus;
    double m_lastMaxConstraintViolation = 0.0;
    // Last accepted solution (scaled, m_solver.x layout) and the reference
    // sample it starts at. While less than N samples old, it gives the warm
    // start, the LMPC linearization point and, after a rejected solve, the
    // control. m_planAge: its age at the current solve, N if none.
    std::vector<double> m_plan;
    std::vector<double> m_planAlpha; // its angle of attack [rad], numUavs per stage, N stages
    std::optional<size_t> m_planIdx;
    size_t m_planAge = 0;
    std::vector<double> m_linPoint; // LMPC: [x u] x N, physical

    // Last applied control [T, roll, pitch] per UAV, physical units: the
    // U_prev parameter of the first-stage rate cost (weight Rdu0).
    std::vector<double> m_uPrev;

    // Latest estimate from setDisturbanceEstimate(), physical units, held
    // between updates; zero without an estimator.
    std::vector<double> m_windEst;
    std::vector<double> m_dEst;
    mutable std::mutex m_disturbanceMutex;

    mutable std::mutex m_solveMutex;

    double m_lastSolveMs = 0.0;

    // Sets the reference time, m_lastIdxTraj, m_referenceWindow and m_endedTraj.
    void m_updateReference(double time);

    // m_plan advanced by `shift` stages (shift < N), tail repeated, z0 from
    // the measured state and U_prev.
    void m_shiftSolution(size_t shift);
    // z0 = [x measured; U_prev], scaled.
    void m_packFirstStage();
    // Decision-variable bounds, scaled: z0 free (fixed by the initial-
    // condition equality), x of the other stages by LBX/UBX_STATES, up free
    // (each equals a bounded control), u by LBX/UBX_CONTROLS.
    void m_packBounds();
    // g rows (NMPC and LMPC): nz initial-condition equalities, then per
    // stage nz dynamics equalities followed by numUavs angle-of-attack rows.
    // Only the alpha rows are inequalities, [-alphaMax, alphaMax].
    void m_packInequalityBounds();
    // Reference window: x_k, u_k, up_k = u_(k-1); z0 from the measurement.
    void m_packInitialGuess();
    // P_optim (build_nlp_*_nmpc.m, build_nlp_*_lmpc.m): [x_initial; reference
    // window x0 u0 ... xN; Wind_est; D_est; Weight; U_prev; L0; LMPC: P_lin].
    // P_lin is linearized about the warm start if aboutPlan, else the reference.
    void m_packParameters(bool aboutPlan);
    // Commands from m_uPrev, the control applied this solve, with the plan's
    // angle of attack AOA_FF_STAGE stages later (NaN without a plan).
    std::map<uint8_t, uavCommandsFlags> m_extractControls() const;

    double m_unwrapYaw(uint8_t sysId, double yawRadWrapped);

    // Fills m_initialStates from telemetry (stateVector.h) and updates the
    // launch phase: standby until initLaunch(), launching until a UAV
    // exceeds inFlightSpeed (back to standby after launchTimeout), then in
    // flight. Until in flight, the UAV velocity is the reference's first one
    // (the launch velocity), so the solve keeps a flight plan from the
    // launcher; position and attitude stay measured.
    void m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates, double time);

    // Emits a LogType::NMPC_EVENT line for any of m_telemetryComplete/
    // m_inFlight/m_endedTraj/m_violation that changed since the last call.
    // Called once per solve(), after all of them have their final value.
    void m_logTransitions();

    // Recomputes m_numTrajectoryPoints/m_endIdxTraj after a new reference.
    void m_onReferenceTrajectoryChanged();
};

#endif //MPCCONTROLLER_H
