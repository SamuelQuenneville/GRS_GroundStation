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

#include "gcs.h"
#include "Util/parseUtils.h"

class ConsoleInterface {

public:
    explicit ConsoleInterface(GroundControlStation& gcs, bool& exitFlag, std::mutex& exitMutex, std::condition_variable& cv);
    ~ConsoleInterface();

    void start();
    void stop();

    static void printCommands();
    void handleCommand(const std::string& command) const;

    static bool parseOrigin(const std::string& input, double& lat, double& lon, double& alt);

private:
    void m_listen(); // Function that runs in the thread
    void m_dispatch(const std::string& command) const;

    std::thread m_inputThread;
    std::atomic<bool> m_running;

    GroundControlStation& m_gcs;
    bool& m_exitFlag;
    std::mutex& m_exitMutex;
    std::condition_variable& m_cv;
};

#endif //CONSOLEINTERFACE_H
