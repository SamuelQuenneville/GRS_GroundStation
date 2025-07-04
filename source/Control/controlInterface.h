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
#include <map>
#include <arpa/inet.h>
#include <ranges>

#include "Definitions/communicationStructures.h"
#include "Log/programLogger.h"

class GroundStationApp;

enum class ControlMode {
    LOCAL,
    MATLAB,
    FILE
};

static constexpr double CONTROLLER_DEFAULT_FREQUENCY_HZ = 10;

class ControlInterface {

public:
    ControlInterface(GroundStationApp& gcs, ControlMode mode);
    ~ControlInterface();

    void start();
    void stop();

    void setControllerFrequency(double frequency);
    void setFileFrequency(double frequency);
    void setMatlabMode(const char* ip, uint16_t port);
    void setCommandsList(const std::map<uint8_t, std::vector<uavCommands>>& commandsList);
    void setShouldMoveList(const std::map<uint8_t, std::vector<bool>>& shouldMoveList);
    void setEndSimulationList(const std::map<uint8_t, std::vector<bool>>& endSimulationList);

private:
    void m_controlLoop();

    void m_initMatlabConnection(const char* ip, uint16_t port);
    void m_sendDataToMatlab(const std::map<uint8_t, uavStates>& states);
    std::map<uint8_t, uavCommands> m_receiveDataFromMatlab();

    GroundStationApp& m_gcs;
    std::atomic<bool> m_running;
    std::thread m_controllerThread;
    double m_frequency;
    double m_fileFrequency;

    std::map<uint8_t, std::vector<uavCommands>> m_commandsList{};
    std::map<uint8_t, std::vector<bool>> m_shouldMoveList{};
    std::map<uint8_t, std::vector<bool>> m_endSimulationList{};

    std::atomic<ControlMode> m_controlMode;
    int m_udpSocketMatlab = 0;
    sockaddr_in m_matlabAddress{};
};



#endif //CONTROLINTERFACE_H
