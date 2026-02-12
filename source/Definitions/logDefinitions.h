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
    CONTROLS
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //LOGDEFINITIONS_H
