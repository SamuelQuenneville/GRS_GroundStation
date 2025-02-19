/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLINTERFACE_H
#define CONTROLINTERFACE_H

#include <thread>
#include <atomic>
#include <chrono>

class GroundStationApp;

class ControlInterface {

public:
    ControlInterface(GroundStationApp& gcs, double frequency);
    ~ControlInterface();

    void start();
    void stop();

private:
    void controlLoop();

    GroundStationApp& m_gcs;
    std::atomic<bool> m_running;
    std::thread m_controllerThread;
    double m_frequency;
};



#endif //CONTROLINTERFACE_H
