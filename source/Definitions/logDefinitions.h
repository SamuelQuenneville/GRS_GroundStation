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
    EVENTS,
    MPC_STATS,
    MPC_X0,
    MPC_X
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //LOGDEFINITIONS_H
