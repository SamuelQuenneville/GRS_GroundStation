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
    m_controlInterface = std::make_unique<ControlInterface>(*this, ControlMode::LOCAL);
}

GroundStationApp::~GroundStationApp() {
    stop();
}

void GroundStationApp::start() {
    m_running = true;
    m_communicationThread = std::thread(&GroundStationApp::m_run, this);
}

void GroundStationApp::initMatlabController(const char* ip, const uint16_t port) const {
    m_controlInterface->setMatlabMode(ip, port);
}

void GroundStationApp::startController() const {
    m_controlInterface->start();
}

void GroundStationApp::setControllerFrequency(const double frequency) const {
    m_controlInterface->setControllerFrequency(frequency);
}


void GroundStationApp::stop() {
    m_running = false;
    if (m_communicationThread.joinable()) {
        m_communicationThread.join();
    }

    m_controlInterface->stop();
}

void GroundStationApp::armAll() {
    m_communicationManager.armAll();
}

void GroundStationApp::setGuidedAll() {
    m_communicationManager.setGuidedAll();
}

std::map<uint8_t, uavStates> GroundStationApp::getControllerInput() {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    return m_communicationManager.getUavsStates();
}

void GroundStationApp::updateControlOutput(const std::map<uint8_t, uavCommands>& uavCommands) {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    m_communicationManager.setUavCommands(uavCommands);
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
        // Ardupilot add 10 to the tcp port for each new connection
        m_communicationManager.addLink("tcpout://" + DEFAULT_REMOTE_IP + ":" + std::to_string(DEFAULT_TCP_PORT + (10*i)));
        }
    }

    while (m_running) {
        //m_updateState();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void GroundStationApp::m_updateState() {
    m_communicationManager.listLinks();
}
