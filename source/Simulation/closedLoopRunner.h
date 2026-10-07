/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CLOSEDLOOPRUNNER_H
#define CLOSEDLOOPRUNNER_H

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>

#include "Control/controlStep.h"
#include "Control/estimatorRunner.h"
#include "Mathematics/math.h"

#include "yaml-cpp/yaml.h"

#include "plantModel.h"
#include "truthSample.h"

namespace grs::sim {

// C++ counterpart of mc_run_one_twoUav.m's opts.
struct RunOptions {
    std::optional<double> tEnd;        // [s]; nullopt = full reference (length - N samples)
    double abortErrM = 50.0;           // tracked-point position error counted as divergence [m]
    double measNoiseStd = 0.0;         // additive Gaussian noise on every measured state (SI, rad)
    unsigned seed = 1;                 // noise stream seed (combined with sample id)
    int nSub = 8;                      // truth-plant sub-steps per control interval
    IntegrationMethod method = IntegrationMethod::RK4;
    double nmheFrequency = 5.0;        // [Hz], NMHE cadence
    // NMHE runs as a deterministic emulation of the GCS's NMHE thread
    // (DeferredEstimatorRunner): its result reaches the controller this long
    // after the solve starts, rounded up to whole control ticks, at least
    // one tick. < 0 = the measured solve time.
    double nmheLatencyMs = 0.0;
    // Delay between the measurement and the command taking effect in the
    // plant (the GCS's solve time + link). 0 = command applied at the start
    // of the interval (MATLAB behavior). cmdDelayMeasured = use each tick's
    // measured NMPC solve time instead of cmdDelayMs.
    double cmdDelayMs = 0.0;
    bool cmdDelayMeasured = false;
    // Real-time controller: no tick while the previous solve still runs, so
    // a solve longer than dt skips the next ticks (the command is held) and
    // the reference, on the simulated clock, moves on meanwhile. Implies
    // cmdDelayMeasured.
    bool deadline = false;
    int storeTrajDecim = 0;            // 0 = no trajectory file, k = keep every k-th step
    int tsDecim = 1;                   // 0 = no time series, k = keep every k-th step
};

// Controller variants, same names as the MATLAB campaign:
// <nmpc|lmpc>_<naive|of>. naive: the wind/d estimate stays zero. of: with the
// NMHE (offset-free), needs EstimatorConfiguration.
struct ControllerVariant {
    std::string family;  // SolverConfiguration.CONTROLLER
    bool useEstimator = false;
};
ControllerVariant parseController(const std::string& controller);

struct RunResult {
    // Metric name -> value, in the same order as mc_metrics_twoUav.m plus
    // n_steps_planned/n_steps_done/completed/wall_s (abort_reason separate).
    std::vector<std::pair<std::string, double>> metrics;
    std::string abortReason;
    bool completed = false;

    // Decimated trajectory (only if RunOptions::storeTrajDecim > 0).
    std::vector<std::string> trajHeader;
    std::vector<std::vector<double>> trajRows;

    // Per-step tracking/constraint/solver signals for the phase-resolved
    // analysis (only if RunOptions::tsDecim > 0). Column-major:
    // tsColumns[c][k] is signal tsHeader[c] at time t = (k*tsDecim + 1)*dt,
    // the state after control k*tsDecim.
    std::vector<std::string> tsHeader;
    std::vector<std::vector<float>> tsColumns;
    double tsDt = 0.0;
};

// One closed-loop run: builds a fresh controller (+ estimator for *_of)
// from `config` exactly as the GCS does, a truth plant from `truth`, and
// steps them together for the reference's length. `reference` is the
// solver-stride reference ([x0 u0 x1 u1 ...], nx+nu per sample, sampled at
// the solver's dt), used both as the controller's reference and as the
// ground truth for the tracking metrics (time-indexed, like MATLAB).
RunResult runClosedLoop(YAML::Node config, const std::string& controller, const std::vector<double>& reference, const TruthSpec& truth, int sampleId, const RunOptions& opts);

} // namespace grs::sim

#endif //CLOSEDLOOPRUNNER_H
