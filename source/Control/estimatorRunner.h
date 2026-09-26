/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ESTIMATORRUNNER_H
#define ESTIMATORRUNNER_H

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "estimator.h"

#include "Log/programLogger.h"


// How the NMHE runs relative to the control loop. ControlStep talks only to
// this interface; the Estimator itself is owned by whoever built it
// (ControlInterface, or grs_batchsim's ControlStack) and must outlive the
// runner.
//
// Contract, identical for every implementation, from the control thread:
//   1. takeEstimate()  at the start of a tick: the newest estimate finished
//                      since the last call, if any (zero-order hold otherwise)
//   2. pushSample()    the tick's measured state and the control applied over
//                      the interval that ended at it (previous tick's command)
//   3. endTick()       after the controller solve (the deterministic runner
//                      does its scheduled solve here; the threaded one ignores it)
// None of these may wait on an NMHE solve in progress.
class EstimatorRunner {
public:
    struct Estimate {
        std::vector<double> wind;
        std::vector<double> d;
    };

    struct Stats {
        uint64_t solves = 0;          // NMHE solves attempted
        uint64_t published = 0;       // successful solves published to the controller
        uint64_t overruns = 0;        // solves longer than one NMHE period (threaded)
        double lastSolveMs = 0.0;
        bool solvedThisTick = false;  // deterministic runner only: a solve happened in the last endTick()
    };

    virtual ~EstimatorRunner() = default;

    virtual std::optional<Estimate> takeEstimate() = 0;
    virtual void pushSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) = 0;
    virtual void endTick() {}
    [[nodiscard]] virtual Stats stats() const = 0;
};

// Live GCS / SITL: the NMHE runs on its own thread at nmheFrequency
// (wall clock), fully decoupled from the control loop. The worker owns the
// Estimator: it drains the samples the control thread pushed since its last
// wake-up into the estimator's window, solves, and publishes the result.
// The control thread only touches two small mutex-protected buffers (sample
// queue, latest estimate), never the Estimator, so an NMHE solve can take
// longer than a control tick without delaying a single command.
class ThreadedEstimatorRunner final : public EstimatorRunner {
public:
    // maxPending: samples kept while the worker is busy; older ones would
    // fall out of the M+1 window anyway (pass M+1 or a little more).
    ThreadedEstimatorRunner(Estimator& estimator, double nmheFrequency, size_t maxPending);
    ~ThreadedEstimatorRunner() override;

    std::optional<Estimate> takeEstimate() override;
    void pushSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) override;
    [[nodiscard]] Stats stats() const override;

private:
    Estimator& m_estimator;
    const double m_periodMs;
    const size_t m_maxPending;

    mutable std::mutex m_inMutex;   // m_pending
    std::deque<std::pair<std::vector<double>, std::vector<double>>> m_pending;

    mutable std::mutex m_outMutex;  // m_latest, m_hasNew, m_stats
    Estimate m_latest;
    bool m_hasNew = false;
    Stats m_stats;

    std::mutex m_wakeMutex;
    std::condition_variable m_wake;
    std::atomic<bool> m_running{true};
    std::thread m_worker;

    void m_loop();
};

// grs_batchsim: deterministic stand-in for ThreadedEstimatorRunner. Same
// cadence (counted in control ticks instead of wall time) and the same
// "result arrives later" behavior: a solve started at the end of tick k is
// handed to the controller at the start of tick k + latency, where latency is
// at least one tick (the controller never waits for the estimator), or more
// if the solve takes longer than a tick:
//   latencyMs < 0  -> measured: ceil(solve time / tick), at least 1
//   latencyMs >= 0 -> fixed:    ceil(latencyMs / tick), at least 1
// Runs the solve inline (so the result is reproducible), only its delivery is
// delayed.
class DeferredEstimatorRunner final : public EstimatorRunner {
public:
    DeferredEstimatorRunner(Estimator& estimator, double hlcFrequency, double nmheFrequency, double latencyMs);

    std::optional<Estimate> takeEstimate() override;
    void pushSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) override;
    void endTick() override;
    [[nodiscard]] Stats stats() const override { return m_stats; }

private:
    Estimator& m_estimator;
    const double m_tickMs;
    const double m_periodMs;
    const double m_latencyMs;
    double m_accumulatorMs = 0.0;
    uint64_t m_tick = 0;

    std::deque<std::pair<uint64_t, Estimate>> m_inFlight; // (tick it becomes available, estimate)
    Stats m_stats;
};

#endif //ESTIMATORRUNNER_H
