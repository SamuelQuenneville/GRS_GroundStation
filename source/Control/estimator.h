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

#include "fatropStatus.h"

// Estimates wind and disturbances over a sliding window of past samples.
// Has no timing of its own: the caller decides when to add samples and
// when to estimate (see EstimatorRunner).
class Estimator {
public:
    virtual ~Estimator() = default;

    // One sample, physical units, joint layout (stateVector.h).
    // appliedControl is the control applied since the previous sample;
    // ignored on the first call.
    virtual void addSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) = 0;

    // Solves on the current window. Returns false without solving until the
    // window is full (M+1 samples), and when the solution is rejected; the
    // previous estimate is kept in both cases.
    virtual bool estimate() = 0;

    // Empties the window and zeroes the estimate, as at construction.
    virtual void reset() = 0;

    // Last valid estimate, physical units; zero before the first one.
    [[nodiscard]] virtual const std::vector<double>& windEstimate() const = 0;
    [[nodiscard]] virtual const std::vector<double>& dEstimate() const = 0;


    struct DebugInfo {
        bool windowFull = false;
        size_t sampleCount = 0;
        double lastSolveMs = 0.0;
        int lastFlag = 0;
        FatropStatus lastFatrop;
        double lastMaxConstraintViolation = 0.0;
        const char* backendName = "";
    };
    [[nodiscard]] virtual DebugInfo getDebugInfo() const = 0;
};

#endif //ESTIMATOR_H
