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
    // Sparse, human-readable NMPC controller events (launch, in-flight,
    // trajectory loaded/ended, solver violation entered/cleared) -- see
    // Logger::start()/log(): unlike every other type above, this one is
    // written regardless of the verboseLogging/heavy-CSV-dump toggle.
    NMPC_EVENT
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //LOGDEFINITIONS_H
