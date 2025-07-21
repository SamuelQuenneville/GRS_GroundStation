/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlInterface.h"
#include "groundStationApp.h"

ControlInterface::ControlInterface(GroundStationApp& gcs, const ControlMode mode)
    : m_gcs(gcs)
    , m_running(false)
    , m_frequency(CONTROLLER_DEFAULT_FREQUENCY_HZ)
    , m_fileFrequency(CONTROLLER_DEFAULT_FREQUENCY_HZ)
    , m_controlMode(mode) {
}

ControlInterface::~ControlInterface() {
    stop();
    close(m_udpSocketMatlab);
}

void ControlInterface::start() {
    m_running = true;
    m_controllerThread = std::thread(&ControlInterface::m_controlLoop, this);
}

void ControlInterface::stop() {
    m_running = false;
    if (m_controllerThread.joinable()) {
        m_controllerThread.join();
    }
}

void ControlInterface::setControllerFrequency(const double frequency) {
    m_frequency = frequency;
}

void ControlInterface::setFileFrequency(const double frequency) {
    m_fileFrequency = frequency;
}

void ControlInterface::setMatlabMode(const char* ip, const uint16_t port) {
    m_controlMode = ControlMode::MATLAB;

    m_initMatlabConnection(ip, port);
}

void ControlInterface::setCommandsList(const std::map<uint8_t, std::vector<uavCommands>>& commandsList) {
    m_controlMode = ControlMode::FILE;
    m_commandsList = commandsList;
}

void ControlInterface::setRcList(const std::map<uint8_t, std::vector<uavRc> > &rcList) {
    m_controlMode = ControlMode::RC;
    m_rcList = rcList;
}

void ControlInterface::setShouldMoveList(const std::map<uint8_t, std::vector<bool>>& shouldMoveList) {
    m_shouldMoveList = shouldMoveList;
}

void ControlInterface::setEndSimulationList(const std::map<uint8_t, std::vector<bool>>& endSimulationList) {
    m_endSimulationList = endSimulationList;
}

void ControlInterface::m_controlLoop() {
    std::chrono::steady_clock::time_point nextTick = std::chrono::steady_clock::now();
    int fileIdx = 0;

    while (m_running) {
        nextTick += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / m_frequency));

        std::map<uint8_t, uavStates>  latestData = m_gcs.getControllerInput();

        if (m_controlMode == ControlMode::MATLAB) {
            LOG_DEBUG("ControlInterface::m_controlLoop() -->MATLAB");
            m_sendDataToMatlab(latestData);
            m_gcs.updateControlOutput(m_receiveDataFromMatlab());
        } else if (m_controlMode == ControlMode::LOCAL) {
            // TODO mpc controller interface
            std::map<uint8_t, uavCommands>  cmd;
            cmd[1] = {1,0,15,0, 0.8};
            cmd[2] = {2,0,15,0, 0.8};
            m_gcs.updateControlOutput(cmd);
        } else if (m_controlMode == ControlMode::FILE) {
            std::map<uint8_t, uavCommands>  cmd;
            std::map<uint8_t, bool>  shouldMove;
            std::map<uint8_t, bool>  endSimulation;

            if (fileIdx >= m_commandsList[1].size()) {
                LOG_INFO("Reach end of trajectory!");
                return;
            }

            for (size_t i = 0; i < m_commandsList.size(); i++) {
                cmd[i+1] = m_commandsList[i+1].at(fileIdx);
                shouldMove[i+1] = m_shouldMoveList[i+1].at(fileIdx);
                endSimulation[i+1] = m_endSimulationList[i+1].at(fileIdx);
            }

            m_gcs.updateControlOutput(cmd, shouldMove, endSimulation);
            fileIdx += static_cast<int>(m_fileFrequency / m_frequency);

        } else if (m_controlMode == ControlMode::RC) {
            std::map<uint8_t, uavRc>  rcData;
            std::map<uint8_t, bool>  shouldMove;
            std::map<uint8_t, bool>  endSimulation;

            if (fileIdx >= m_rcList[1].size()) {
                LOG_INFO("Reach end of trajectory!");
                return;
            }

            for (size_t i = 0; i < m_rcList.size(); i++) {
                rcData[i+1] = m_rcList[i+1].at(fileIdx);
                shouldMove[i+1] = m_shouldMoveList[i+1].at(fileIdx);
                endSimulation[i+1] = m_endSimulationList[i+1].at(fileIdx);
            }

            m_gcs.updateControlOutput(rcData, shouldMove, endSimulation);
            fileIdx += static_cast<int>(m_fileFrequency / m_frequency);

        } else {
            LOG_ERROR("Error setting controller Mode");
        }

        std::this_thread::sleep_until(nextTick);
    }
}

void ControlInterface::m_initMatlabConnection(const char* ip, const uint16_t port) {

    m_udpSocketMatlab = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSocketMatlab < 0) {
        LOG_ERROR("Error creating UDP socket");
        return;
    }

    m_matlabAddress.sin_family = AF_INET;
    m_matlabAddress.sin_port = htons(port);
    inet_pton(AF_INET, ip, &m_matlabAddress.sin_addr);
}


void ControlInterface::m_sendDataToMatlab(const std::map<uint8_t, uavStates>& states) {

    std::vector<uavStates> packet;
    for (const auto& state: states | std::views::values) {
        packet.push_back(state);
    }

    const size_t dataSize = packet.size() * sizeof(uavStates);
    std::vector<uint8_t> buffer(dataSize);
    memcpy(buffer.data(), packet.data(), dataSize);

    sendto(m_udpSocketMatlab, buffer.data(), buffer.size(), 0,
           reinterpret_cast<struct sockaddr *>(&m_matlabAddress), sizeof(m_matlabAddress));

}

std::map<uint8_t, uavCommands> ControlInterface::m_receiveDataFromMatlab() {

    std::map<uint8_t, uavCommands> receivedData;
    char buffer[m_gcs.getNumberOfUavs() * sizeof(uavCommands)];
    std::vector<uavCommands> commands(m_gcs.getNumberOfUavs());

    socklen_t addrLen = sizeof(m_matlabAddress);
    const auto bytesReceived = recvfrom(m_udpSocketMatlab, buffer, sizeof(buffer), 0,
                                 reinterpret_cast<struct sockaddr *>(&m_matlabAddress), &addrLen);

    if (bytesReceived > 0) {
        memcpy(commands.data(), buffer, bytesReceived);
        for (const auto& command: commands) {
            receivedData[static_cast<uint8_t>(command.sysId)] = command;
        }
    }

    return receivedData;
}