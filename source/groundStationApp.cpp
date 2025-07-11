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

void GroundStationApp::parseCommandFile(const std::string& file) const {
    std::ifstream fileStream(file);

    if (!fileStream.is_open()) {
        LOG_ERROR("Failed to open file");
    }

    std::map<uint8_t, std::vector<double>> timeList;
    std::map<uint8_t, std::vector<uavCommands>> commandsList;
    std::map<uint8_t, std::vector<bool>> shouldMoveList;
    std::map<uint8_t, std::vector<bool>> endSimulationList;
    std::string line;

    while (std::getline(fileStream, line)) {
        double time;
        uavCommands command{};
        bool shouldMove;
        bool endSimulation;
        if (parseUavCommandsLine(line, time, command, shouldMove, endSimulation)) {
            timeList[static_cast<uint8_t>(command.sysId)].push_back(time);
            commandsList[static_cast<uint8_t>(command.sysId)].push_back(command);
            shouldMoveList[static_cast<uint8_t>(command.sysId)].push_back(shouldMove);
            endSimulationList[static_cast<uint8_t>(command.sysId)].push_back(endSimulation);
        } else {
            LOG_ERROR("Failed to parse command line from file");
        }
    }

    fileStream.close();

    double f = 1 / (timeList[1][1] - timeList[1][0]);
    m_controlInterface->setFileFrequency(f);

    m_controlInterface->setShouldMoveList(shouldMoveList);
    m_controlInterface->setEndSimulationList(endSimulationList);
    m_controlInterface->setCommandsList(commandsList);
}

bool GroundStationApp::parseUavCommandsLine(const std::string& line, double& time, uavCommands& command, bool& shouldMove, bool& endSimulation) {
    std::istringstream lineStream(line);
    std::string token;

    // Line definition:
    // time (sec), sysId, roll (deg), pitch (deg), yaw (deg), thrust (N), F1 (0/1), F2 (0/1)

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    time = std::stod(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    command.sysId = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    command.rollCommand = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    command.pitchCommand = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    command.yawCommand = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    command.thrustCommand = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    shouldMove = static_cast<bool>(std::stoi(token));

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    endSimulation = static_cast<bool>(std::stoi(token));

    return true;
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

void GroundStationApp::updateControlOutput(const std::map<uint8_t, uavCommands>& uavCommands, const std::map<uint8_t, bool>& shouldMove, const std::map<uint8_t, bool>& endSimulation) {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    m_communicationManager.setUavShouldMove(shouldMove);
    m_communicationManager.setEndSimulation(endSimulation);
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
        m_communicationManager.addLink("tcpout://" + std::string(DEFAULT_REMOTE_IP) + ":" + std::to_string(DEFAULT_TCP_PORT + (10*i)));
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
