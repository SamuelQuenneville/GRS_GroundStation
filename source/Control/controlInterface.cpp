/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlInterface.h"
#include "gcs.h"

ControlInterface::ControlInterface()
    : m_running(false)
{

}

ControlInterface::~ControlInterface() {
    stop();
    close(m_udpSocketMatlab);
}

void ControlInterface::initialize(const gcsConfig& config) {
    m_config = config;

    if (m_config.controlMode == ControlMode::MPC) {
        constexpr double degToRad = M_PI / 180.0;
        double INF = std::numeric_limits<double>::infinity();

        std::vector<double> lbxStates = {-INF, -INF, -INF, 10.0, -INF, -30.0*degToRad, -40.0*degToRad, -INF, -INF, -INF, 10.0, -INF, -30.0*degToRad, -40.0*degToRad, -INF, -INF, -INF, -INF, -INF, -INF};
        std::vector<double> ubxStates = { INF,  INF,  0.0, 40.0,  INF,  30.0*degToRad,  40.0*degToRad,  INF,  INF,  INF, 40.0,  INF,  30.0*degToRad,  40.0*degToRad,  INF,  INF,  INF,  INF,  INF,  INF};
        std::vector<double> lbxControls = { 0.0, -30.0*degToRad, -40.0*degToRad,  0.0, -30.0*degToRad, -40.0*degToRad};
        std::vector<double> ubxControls = {60.0,  30.0*degToRad,  40.0*degToRad, 60.0,  30.0*degToRad,  40.0*degToRad};

        NMPCController::solverConfig solverConfig{20,6,3,25,2, lbxStates,ubxStates, lbxControls, ubxControls};
        m_nmpc = std::make_unique<NMPCController>(solverConfig);
    }
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

void ControlInterface::setCommandCallback(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> cb) {
    m_sendCommand = std::move(cb);
}

void ControlInterface::updateStates(const std::map<uint8_t, uavStates>& states) {
    std::lock_guard lock(m_stateMutex);
    m_latestStates = states;
}

void ControlInterface::initMatlabConnection(const char* ip, const uint16_t port) {
    m_initMatlabConnection(ip, port);
}

void ControlInterface::setCommandsList(const std::map<uint8_t, std::vector<uavCommandsFlags>>& commandsList) {
    m_commandsList = commandsList;
    m_fileFrequency = 1.0 / (m_commandsList[1][1].timestamp.value() - m_commandsList[1][0].timestamp.value());
}

void ControlInterface::initLaunch() const {
    m_nmpc->initLaunch();
}

void ControlInterface::m_controlLoop() {
    int fileIdx = 0;

    while (m_running) {

        std::map<uint8_t, uavStates> latestStates;
        {
            std::lock_guard lock(m_stateMutex);
            latestStates = m_latestStates;
        }

        std::map<uint8_t, uavCommandsFlags>  cmds;

        if (m_config.controlMode == ControlMode::MATLAB) {
            LOG_DEBUG("ControlInterface::m_controlLoop() --> MATLAB");
            m_sendDataToMatlab(latestStates);
            auto output = m_receiveDataFromMatlab();

            for (size_t i = 0; i < output.size(); i++) {
                cmds[i+1].commands = output[i+1];
            }

        } else if (m_config.controlMode == ControlMode::MPC) {
            cmds = m_nmpc->solve(latestStates);

        } else if (m_config.controlMode == ControlMode::ATTITUDE_FILE) {

            if (fileIdx >= m_commandsList[1].size()) {
                LOG_INFO("Reach end of trajectory!");
                return;
            }

            for (size_t i = 0; i < m_commandsList.size(); i++) {
                cmds[i+1] = m_commandsList[i+1].at(fileIdx);
            }

            fileIdx += static_cast<int>(m_fileFrequency / m_config.hlcFrequency);

        } else {
            LOG_ERROR("Error setting controller Mode");
        }

        if (m_sendCommand) {
            m_sendCommand(cmds); // push to dispatcher queue
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / m_config.hlcFrequency)));
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
    char buffer[m_config.numUavs * sizeof(uavCommands)];
    std::vector<uavCommands> commands(m_config.numUavs);

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