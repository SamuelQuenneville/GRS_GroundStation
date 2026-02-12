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

#include "gcsConfig.h"
#include "Definitions/communicationStructures.h"
#include "Configuration/configurationParser.h"
#include "Log/programLogger.h"
#include "Powertrain/powertrain.h"
#include "navigationFrameManager.h"
#include "NMPCController.h"

class ControlInterface {

public:
    ControlInterface();
    ~ControlInterface();

    void initialize(const gcsConfig& config);
    void start();
    void stop();

    void setCommandCallback(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> cb);
    void updateStates(const std::map<uint8_t, uavStates>& states);

    void initMatlabConnection(const char* ip, uint16_t port);
    void setCommandsList(const std::map<uint8_t, std::vector<uavCommandsFlags>>& commandsList);

    void initLaunch() const;

    void loadTrajectory(const std::string& file) const;
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

private:
    NavigationFrameManager m_navFrameManager;

    void m_controlLoop();

    void m_initMatlabConnection(const char* ip, uint16_t port);
    void m_sendDataToMatlab(const std::map<uint8_t, uavStates>& states);
    std::map<uint8_t, uavCommands> m_receiveDataFromMatlab();

    std::atomic<bool> m_running;
    std::thread m_controllerThread;
    double m_fileFrequency = 10.0;

    gcsConfig m_config;

    std::unique_ptr<NMPCController> m_nmpc;

    std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> m_sendCommand;
    std::map<uint8_t, uavStates> m_latestStates;
    std::mutex m_stateMutex;

    std::map<uint8_t, std::vector<uavCommandsFlags>> m_commandsList{};

    int m_udpSocketMatlab = 0;
    sockaddr_in m_matlabAddress{};
};


#endif //CONTROLINTERFACE_H
