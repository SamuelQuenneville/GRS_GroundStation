/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */
#include "consoleInterface.h"

ConsoleInterface::ConsoleInterface(GroundControlStation& gcs, bool& exitFlag, std::condition_variable& cv)
    : m_running(false)
    , m_gcs(gcs)
    , m_exitFlag(exitFlag)
    , m_cv(cv)
{

}

ConsoleInterface::~ConsoleInterface() {
    stop();
}

void ConsoleInterface::start() {
    m_running = true;
    m_inputThread = std::thread(&ConsoleInterface::m_listen, this);
}

void ConsoleInterface::stop() {
    m_running = false;
    if (m_inputThread.joinable()) {
        m_inputThread.join();
    }
}

void ConsoleInterface::printCommands() {
    std::cout << "Commands: [commands]\n"
                      << "  start             --> Start the Ground Station\n"
                      << "  connect           --> Connect to all UAVs\n"
                      << "  arm               --> Arm all connected system\n"
                      << "  mode [MODE]       --> Set mode for all connected system (MANUAL / GUIDED / XNAV)\n"
                      << "  startController   --> Start the controller\n"
                      << "  launch            --> Init launch sequence\n"
                      << "  fetchParams [ID]  --> Retrieve all parameter and create a .param file\n"
                      << "  stop              --> Stop the Ground Station\n"
                      << "  exit              --> Terminate the execution\n";
}

void ConsoleInterface::handleCommand(const std::string& command) const {
    if (command == "start") {
        LOG_INFO("Starting main process...");
        m_gcs.start();
    } else if (command == "connect") {
        LOG_INFO("Connecting to all UAVs...");
        m_gcs.connectAll();
    } else if (command == "arm") {
        LOG_INFO("Arming controller ...");
        m_gcs.armAll();
    } else if (command.starts_with("mode ")) {
        LOG_INFO("Setting mode ...");
        m_gcs.setModeAll(command.substr(5));
    }else if (command == "startController") {
        LOG_INFO("Starting controller ...");
        m_gcs.startController();
    } else if (command == "launch") {
        LOG_INFO("Launching ...");
        m_gcs.initLaunch();
    } else if (command.starts_with("fetchParams ")) {
        m_gcs.fetchParam(std::stoi(command.substr(12)));
    } else if (command == "stop") {
        LOG_INFO("Stopping main process...");
        m_gcs.stop();
    } else if (command == "exit") {
        LOG_INFO("Exiting program...");
        m_gcs.stop();
    } else {
        LOG_INFO("Unknown command");
    }
}

void ConsoleInterface::m_listen() {
    std::string command;
    while (m_running) {
        std::cout << "\nGCS->";
        std::getline(std::cin, command);

        if (!m_running) {
            break;
        }

        if (command == "exit") {
            {
                std::lock_guard<std::mutex> lock(*(new std::mutex())); // short-lived local lock
                m_exitFlag = true;
            }
            handleCommand(command);
            m_cv.notify_one();
            m_running = false;
            return;
        }

        if (!command.empty()) {
            handleCommand(command);
        }
    }
}
