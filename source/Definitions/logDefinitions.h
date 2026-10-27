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

enum class LogType {
    MPC_ARG_X0,
    MPC_ARG_P,
    MPC_ARG_LBX,
    MPC_ARG_UBX,
    MPC_RES_X,
    STATES,
    CONTROLS,
    // ms, wall ms, sample time, then the measured state vector (stateVector.h)
    // as received, before the launch-phase substitution logged in STATES.
    MEASURED_STATES,
    // ms, wall ms, sample time, event (applied | ignored | reset), NMHE solves,
    // published, overruns, last solve ms, then wind (np) and d (nd) if any.
    ESTIMATOR,
    // ms, wall ms, sample time, accepted, warm start age (N: cold start), then for stages k = 0..5:
    // x_k (nx), u_k (nu), alpha_k (numUavs), then x_N. Physical units, the
    // solve's result whether accepted or not.
    PLAN,
    // ms, wall ms, sample time, tick period ms, latency ms (tick start to
    // command sent), then per vehicle: sysId, state age ms at tick start,
    // vehicle time_usec of that state (CONTROL_SYSTEM_STATE).
    TIMING,
    // Sparse NMPC events (launch, in flight, trajectory ended, violation,
    // telemetry complete). Always written, unlike the types above.
    NMPC_EVENT
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //LOGDEFINITIONS_H
