/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#pragma once

#include <cstddef>
#include <vector>

// Top-level abstraction over "the thing estimating wind/disturbance this
// tick" -- the estimator-side counterpart to Controller (see controller.h
// and gcs-sitl-integration-plan.md §3a). NmheEstimator is the first (and,
// as of this writing, only) implementation, driving either NMHE solver
// through EstimatorBackend the same way MpcController drives either NMPC
// solver through SolverBackend.
//
// Deliberately runs on its own cadence, decoupled from the 20 Hz control
// loop (see ControlInterface -- the NMHE update rate is its own config
// value, starting point 5 Hz per gcs-sitl-integration-plan.md's
// Decisions): Estimator itself has no notion of a timer or a target rate,
// it only knows what's been pushed into its sliding window so far via
// addSample() and what estimate() does with that window when called. The
// caller decides when both happen.
class Estimator {
public:
    virtual ~Estimator() = default;

    // Pushes one new (measured state, applied control) sample into the
    // sliding window, in PHYSICAL units, joint-across-vehicles layout
    // (same convention as solverConfig::nx/nu -- see MpcController's
    // m_unpackLatestStates() for how that layout is built from telemetry).
    // appliedControl is the control that was applied going from the
    // PREVIOUS sample to this one -- ignored on the very first call (there
    // is no previous sample yet to pair it with).
    virtual void addSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) = 0;

    // Runs one NMHE solve against the current window. Returns false (no
    // solve attempted) until the window has filled (M+1 samples) -- until
    // then windEstimate()/dEstimate() hold their last value (zero before
    // the first successful solve). On a solver failure/violation, the
    // previous successful estimate is kept rather than overwritten with a
    // bad one -- same fallback philosophy as MpcController's
    // m_solutionIsValid()/m_violation.
    virtual bool estimate() = 0;

    // Current best estimate, physical units -- zero-order held between
    // estimate() calls, per gcs-sitl-integration-plan.md's Decisions.
    [[nodiscard]] virtual const std::vector<double>& windEstimate() const = 0;
    [[nodiscard]] virtual const std::vector<double>& dEstimate() const = 0;

    // Debug/health snapshot, same reasoning as Controller::DebugInfo
    // (plain struct, independent of Dashboard/).
    struct DebugInfo {
        bool windowFull = false;
        size_t sampleCount = 0;
        double lastSolveMs = 0.0;
        int lastFlag = 0;
        double lastMaxConstraintViolation = 0.0;
        const char* backendName = "";
    };
    [[nodiscard]] virtual DebugInfo getDebugInfo() const = 0;
};

#endif //ESTIMATOR_H
