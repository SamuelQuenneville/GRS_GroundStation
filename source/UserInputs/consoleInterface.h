/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONSOLEINTERFACE_H
#define CONSOLEINTERFACE_H

#include <atomic>

#include "groundStationApp.h"

class ConsoleInterface {

public:
    explicit ConsoleInterface(GroundStationApp& gcs, bool& exitFlag, std::condition_variable& cv);
    ~ConsoleInterface();

    void start();
    void stop();

    static void printCommands();
    void handleCommand(const std::string& command) const;

private:
    void m_listen(); // Function that runs in the thread
    std::thread m_inputThread;
    std::atomic<bool> m_running;

    GroundStationApp& m_gcs;
    bool& m_exitFlag;
    std::condition_variable& m_cv;
};

#endif //CONSOLEINTERFACE_H
