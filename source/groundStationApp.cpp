/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "groundStationApp.h"

GroundStationApp::GroundStationApp()
    : m_mavsdk(GROUND_STATION)
    , m_running(false)
{
    LOG_INFO("----- GCS Online -----");
}

GroundStationApp::~GroundStationApp() {
    stop();
}

void GroundStationApp::start() {
    m_running = true;
    m_runThread = std::thread(&GroundStationApp::m_run, this);
}

void GroundStationApp::stop() {
    m_running = false;
    if (m_runThread.joinable()) {
        m_runThread.join();
    }
    LOG_INFO("----- GCS Disconnected -----");
}

void GroundStationApp::m_run() {
    while (m_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}