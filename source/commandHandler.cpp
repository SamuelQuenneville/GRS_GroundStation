/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "commandHandler.h"

CommandHandler::CommandHandler(GroundStationApp& gcs)
    : m_gcs(gcs)
{

}

void CommandHandler::handleCommand(const std::string& command) const {
    if (command == "start") {
        LOG_INFO("Starting main process...");
        m_gcs.start();
    } else if (command == "stop") {
        LOG_INFO("Stopping main process...");
        m_gcs.stop();
    } else if (command == "exit") {
        LOG_INFO("Exiting program...");
        m_gcs.stop();
        exit(0);
    } else {
        LOG_INFO("Unknown command");
    }
}
