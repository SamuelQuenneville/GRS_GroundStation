/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H

#include <mavsdk/mavsdk.h>
#include <thread>
#include <memory>

#include "Log/programLogger.h"

#define GROUND_STATION mavsdk::Mavsdk::Configuration(mavsdk::Mavsdk::ComponentType::GroundStation)

class GroundStationApp {

public:
    GroundStationApp();
    ~GroundStationApp();

    void start();
    void stop();

private:
    mavsdk::Mavsdk m_mavsdk;
    std::shared_ptr<mavsdk::System> m_system;

    void m_run();
    std::thread m_runThread;
    std::atomic<bool> m_running;
};



#endif //GROUNDSTATION_H
