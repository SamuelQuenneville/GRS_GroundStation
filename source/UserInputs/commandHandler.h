/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "groundStationApp.h"

class CommandHandler {

public:
    explicit CommandHandler(GroundStationApp& gcs);
    ~CommandHandler() = default;

    static void printCommands();
    void handleCommand(const std::string& command) const;

private:
    GroundStationApp& m_gcs;
};



#endif //COMMANDHANDLER_H
