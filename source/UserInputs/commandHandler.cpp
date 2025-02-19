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

void CommandHandler::printCommands() {
    std::cout << "Commands: [commands]\n"
                      << "  start             --> Start the Ground Station\n"
                      << "  startController   --> Start the controller\n"
                      << "  stop              --> Stop the Ground Station\n"
                      << "  update            --> Temporary passthrough to controller\n"
                      << "  exit              --> Terminate the execution\n";
}


void CommandHandler::handleCommand(const std::string& command) const {
    if (command == "start") {
        LOG_INFO("Starting main process...");
        m_gcs.start();
    } else if (command == "startController") {
        LOG_INFO("Starting controller ...");
        m_gcs.startController();
    } else if (command == "stop") {
        LOG_INFO("Stopping main process...");
        m_gcs.stop();
    } else if (command.substr(0, 7) == "update ") {
        const int value = std::stoi(command.substr(7));
        m_gcs.updateControlInput(value);
    }else if (command == "exit") {
        LOG_INFO("Exiting program...");
        m_gcs.stop();
        exit(0);
    } else {
        LOG_INFO("Unknown command");
    }
}
