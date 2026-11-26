/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logSchema.h"

#include <algorithm>
#include <string>
#include <vector>

#include "Log/logger.h"

namespace grs::logschema {

namespace {

using Columns = std::vector<LogColumn>;

void append(Columns& to, const Columns& from) {
    to.insert(to.end(), from.begin(), from.end());
}

// Joint state vector (stateVector.h): UAV i is "_<i>", the payload "_pay".
Columns stateColumns(const int numUavs, const bool hasPayload, const std::string& suffix = "") {
    Columns c;
    const auto block = [&](const std::string& who, const bool attitude) {
        for (const char* p : {"pN", "pE", "pD"}) c.push_back({p + who + suffix, "m"});
        for (const char* v : {"vN", "vE", "vD"}) c.push_back({v + who + suffix, "m/s"});
        if (attitude) {
            c.push_back({"roll" + who + suffix, "rad"});
            c.push_back({"pitch" + who + suffix, "rad"});
        }
    };
    for (int i = 1; i <= numUavs; ++i) block("_" + std::to_string(i), true);
    if (hasPayload) block("_pay", false);
    return c;
}

// Joint control vector of the solver: [thrust, roll, pitch] per UAV.
Columns controlColumns(const int numUavs, const int nu, const std::string& suffix) {
    Columns c;
    const int perUav = nu / std::max(1, numUavs);
    for (int i = 1; i <= numUavs; ++i) {
        const std::string who = "_" + std::to_string(i) + suffix;
        if (perUav == 3) {
            c.push_back({"thrustCmd" + who, "N"});
            c.push_back({"rollCmd" + who, "rad"});
            c.push_back({"pitchCmd" + who, "rad"});
        } else {
            for (int k = 1; k <= perUav; ++k) c.push_back({"u" + std::to_string(k) + who, "-"});
        }
    }
    return c;
}

Columns tickClock() { return {{"tick", "-"}, {"steadyMs", "ms"}, {"wallTimeMs", "ms"}}; }
Columns sampleClock() { return {{"steadyMs", "ms"}, {"wallTimeMs", "ms"}, {"steadySec", "s"}}; }

} // namespace

void registerControlColumns(const solverConfig& solver, const bool hasPayload) {
    auto& logger = Logger::instance();
    const int n = solver.numUavs;

    // STATES (MpcController::m_unpackLatestStates)
    Columns states = tickClock();
    append(states, stateColumns(n, hasPayload));
    logger.setColumns(LogType::STATES, states);

    // MEASURED_STATES (ControlStep::tick)
    Columns measured = sampleClock();
    append(measured, stateColumns(n, hasPayload));
    logger.setColumns(LogType::MEASURED_STATES, measured);

    // CONTROLS (MpcController::m_extractControls), one row per UAV
    Columns controls = tickClock();
    append(controls, {{"sysId", "-"}, {"solveMs", "ms"},
                      {"thrust", "N"}, {"rollDeg", "deg"}, {"pitchDeg", "deg"}, {"yawDeg", "deg"},
                      {"idxTraj", "-"}, {"iterations", "-"}, {"returnCode", "-"}, {"referenceTime", "s"},
                      {"aoaDeg", "deg"}, {"valid", "-"}, {"planAge", "-"}});
    logger.setColumns(LogType::CONTROLS, controls);

    // ESTIMATOR (controlStep.cpp logEstimator)
    Columns estimator = sampleClock();
    append(estimator, {{"event", "-"}, {"solves", "-"}, {"published", "-"}, {"overruns", "-"}, {"lastSolveMs", "ms"}});
    if (solver.np == 3) {
        append(estimator, {{"wN", "m/s"}, {"wE", "m/s"}, {"wD", "m/s"}});
    } else {
        for (int k = 1; k <= solver.np; ++k) estimator.push_back({"wind_" + std::to_string(k), "-"});
    }
    // Disturbances mix units (forces, coefficients, biases): see the model's d layout.
    for (int k = 1; k <= solver.nd; ++k) estimator.push_back({"d_" + std::to_string(k), "-"});
    logger.setColumns(LogType::ESTIMATOR, estimator);

    // PLAN (MpcController::m_logPlan): stages 0..5, then the terminal state
    Columns plan = sampleClock();
    append(plan, {{"accepted", "-"}, {"planAge", "-"}});
    for (int k = 0; k < std::min(6, solver.N); ++k) {
        const std::string s = "_k" + std::to_string(k);
        append(plan, stateColumns(n, hasPayload, s));
        append(plan, controlColumns(n, solver.nu, s));
        for (int i = 1; i <= n; ++i) plan.push_back({"alpha_" + std::to_string(i) + s, "rad"});
    }
    append(plan, stateColumns(n, hasPayload, "_kN"));
    logger.setColumns(LogType::PLAN, plan);

    // TIMING (ControlInterface::m_controlLoop): one slot per UAV, then the payload
    Columns timing = sampleClock();
    append(timing, {{"periodMs", "ms"}, {"latencyMs", "ms"}});
    const auto slot = [&](const std::string& who) {
        append(timing, {{"sysId_" + who, "-"}, {"ageMs_" + who, "ms"}, {"vehicleTimeUs_" + who, "us"}});
    };
    for (int i = 1; i <= n; ++i) slot(std::to_string(i));
    if (hasPayload) slot("pay");
    logger.setColumns(LogType::TIMING, timing);
}

} // namespace grs::logschema
