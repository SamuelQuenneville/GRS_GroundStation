/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "groundStationApp.h"

GroundStationApp::GroundStationApp()
    : m_running(false)
{
    m_controlInterface = std::make_unique<ControlInterface>(*this, 10.0);
    m_controlInput = 0;
    m_controlOutput = 0;
}

GroundStationApp::~GroundStationApp() {
    stop();
}

void GroundStationApp::start() {
    m_running = true;
    m_communicationThread = std::thread(&GroundStationApp::m_run, this);
}

void GroundStationApp::startController() const {
    m_controlInterface->start();
}


void GroundStationApp::stop() {
    m_running = false;
    if (m_communicationThread.joinable()) {
        m_communicationThread.join();
    }

    m_controlInterface->stop();
}

void GroundStationApp::updateControlInput(const int newInput) {
    std::unique_lock<std::shared_mutex> lock(m_dataMutex);
    m_controlInput = newInput;
    LOG_INFO("Updated shared data:");
}

int GroundStationApp::getControlInput() {
    std::shared_lock<std::shared_mutex> lock(m_dataMutex);
    return m_controlInput;
}

void GroundStationApp::updateControlOutput(const int newOutput) {
    std::unique_lock<std::shared_mutex> lock(m_dataMutex);
    m_controlOutput = newOutput;
}

int GroundStationApp::getControlOutput() {
    std::shared_lock<std::shared_mutex> lock(m_dataMutex);
    return m_controlOutput;
}

void GroundStationApp::setNumberOfUavs(const int numUavs) {
    m_numberOfUavs = numUavs;
}

int GroundStationApp::getNumberOfUavs() const {
    return m_numberOfUavs;
}

void GroundStationApp::m_run() {

    if (!m_numberOfUavs == 0) {
        for (int i = 0; i < m_numberOfUavs; i++) {
        m_communicationManager.addLink("tcpout://" + DEFAULT_REMOTE_IP + ":" + std::to_string(DEFAULT_TCP_PORT + (10*i)));
        }
    }

    while (m_running) {
        m_updateState();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void GroundStationApp::m_updateState() {
    m_communicationManager.listLinks();
}
