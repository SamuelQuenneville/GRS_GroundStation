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
    , m_controlMode(mode)
{

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


void ControlInterface::setMatlabMode(const char* ip, const uint16_t port) {
    m_controlMode = ControlMode::MATLAB;

    m_initMatlabConnection(ip, port);
}

void ControlInterface::m_controlLoop() {
    std::chrono::steady_clock::time_point nextTick = std::chrono::steady_clock::now();

    while (m_running) {
        nextTick += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / m_frequency));

        std::map<uint8_t, uavStates>  latestData = m_gcs.getControllerInput();

        if (m_controlMode == ControlMode::MATLAB) {
            m_sendDataToMatlab(latestData);
            m_gcs.updateControlOutput(m_receiveDataFromMatlab());
        } else if (m_controlMode == ControlMode::LOCAL) {
            // TODO mpc controller interface

            //m_gcs.updateControlOutput(controlOutput);
        } else {
            std::cerr << "Error setting controller Mode\n";
        }

        std::this_thread::sleep_until(nextTick);
    }
}

void ControlInterface::m_initMatlabConnection(const char* ip, const uint16_t port) {

    m_udpSocketMatlab = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSocketMatlab < 0) {
        std::cerr << "Error creating UDP socket\n";
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