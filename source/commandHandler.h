/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <string>

#include "groundStationApp.h"

class CommandHandler {

public:
    explicit CommandHandler(GroundStationApp& gcs);
    ~CommandHandler() = default;

    void handleCommand(const std::string& command) const;

private:
    GroundStationApp& m_gcs;
};



#endif //COMMANDHANDLER_H
