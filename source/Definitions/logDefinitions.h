/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef LOGDEFINITIONS_H
#define LOGDEFINITIONS_H

#pragma once

#include <string>

// Bumped on every change to a log file's columns or meaning; written to
// manifest.yaml so the MATLAB parser picks the matching reader.
//   1: no headers
//   2: names + units lines (Log/logSchema.cpp), manifest.yaml, config.yaml copy,
//      nmpc_events.csv (quoted event text), fixed-width ESTIMATOR and TIMING
//      rows, sysId/valid/planAge in CONTROLS, unused MPC_ARG_* / MPC_RES_X files removed
inline constexpr int kLogSchemaVersion = 2;

// Column names and units of every type are defined in Log/logSchema.cpp.
enum class LogType {
    // tick, steady ms, wall ms, then the state vector given to the solve
    // (stateVector.h), after the launch-phase substitution.
    STATES,
    // One row per UAV and tick: tick, steady ms, wall ms, sysId, solve ms,
    // command (thrust N, roll/pitch/yaw deg), solver status, valid, plan age.
    CONTROLS,
    // ms, wall ms, sample time, then the measured state vector (stateVector.h)
    // as received, before the launch-phase substitution logged in STATES.
    MEASURED_STATES,
    // ms, wall ms, sample time, event (applied | ignored | reset), NMHE solves,
    // published, overruns, last solve ms, then wind (np) and d (nd), NaN
    // when the row carries no estimate.
    ESTIMATOR,
    // ms, wall ms, sample time, accepted, warm start age (N: cold start), then for stages k = 0..5:
    // x_k (nx), u_k (nu), alpha_k (numUavs), then x_N. Physical units, the
    // solve's result whether accepted or not.
    PLAN,
    // ms, wall ms, sample time, tick period ms, latency ms (tick start to
    // command sent), then one slot per UAV (and the payload): sysId, state
    // age ms at tick start, vehicle time_usec of that state
    // (CONTROL_SYSTEM_STATE). Missing vehicle: sysId -1, age -1, time 0.
    TIMING,
    // Emulated link delay (SITL): wall ms, sysId, D (downlink) or U (uplink),
    // vehicle time_usec (downlink, 0 uplink), delay ms from reception or send to release.
    LINK,
    // Sparse NMPC events (launch, in flight, trajectory ended, violation,
    // telemetry complete). Always written, unlike the types above.
    NMPC_EVENT
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //LOGDEFINITIONS_H
