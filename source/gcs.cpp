/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gcs.h"

GroundControlStation::GroundControlStation()
    : m_running(false)
{
    m_controlDispatcher    = std::make_unique<ControlDispatcher>();
    m_communicationManager = std::make_unique<CommunicationManager>();
    m_controlInterface     = std::make_unique<ControlInterface>();

    // Link communication → dispatcher → controller
    m_communicationManager->setTelemetryCallback([this](const std::map<uint8_t, uavStates>& states) {
        m_controlDispatcher->updateTelemetry(states);
    });

    m_controlDispatcher->attachCommunicationManager([this](const std::map<uint8_t, uavCommandsFlags>& cmds) {
        m_communicationManager->setUavCommands(cmds);
    });

    m_controlDispatcher->attachControllerInput([this](const std::map<uint8_t, uavStates>& states) {
        m_controlInterface->updateStates(states);
    });

    m_controlInterface->setCommandCallback([this](const std::map<uint8_t, uavCommandsFlags>& cmds) {
        m_controlDispatcher->pushCommand(cmds);
    });
}

GroundControlStation::~GroundControlStation() {
    stop();
}

void GroundControlStation::initialize(const gcsConfig& config)
{
    m_gcsConfig = config;

    if (config.verbose) {
        PROGRAM_LOGGER.enableVerbose(true);
    }

    m_communicationManager->initialize(config);
    m_controlInterface->initialize(config);

    if (config.matlab.has_value()) {
        const auto& [ip, port] = config.matlab.value();
        m_controlInterface->initMatlabConnection(ip.c_str(), port);
    }

    if (config.attitudeFile.has_value()) {
        m_parseCommandFile(config.attitudeFile.value());
    }

}

void GroundControlStation::start() {
    if (m_running) {
        LOG_WARNING("GroundControlStation already running.");
        return;
    }

    m_running = true;
    m_supervisorThread = std::thread(&GroundControlStation::m_supervisorLoop, this);
    LOG_INFO("GroundControlStation supervisor thread started.");
}

void GroundControlStation::stop() {
    if (!m_running)
        return;

    m_running = false;

    if (m_supervisorThread.joinable())
        m_supervisorThread.join();

    LOG_INFO("GroundControlStation stopped cleanly.");
}

void GroundControlStation::connectAll() {
    if (m_gcsConfig.numUavs == 0) {
        LOG_ERROR("Cannot connect — number of UAVs not set.");
        return;
    }

    LOG_INFO("Starting communication...");
    m_communicationManager->connectAll(m_gcsConfig.pixhawk.remoteIP, m_gcsConfig.pixhawk.tcpPort, m_gcsConfig.numUavs, m_gcsConfig.pixhawk.tcpPortIncrement);

    if (!m_running) {
        m_running = true;
        m_supervisorThread = std::thread(&GroundControlStation::m_supervisorLoop, this);
    }
}

void GroundControlStation::armAll() const {
    m_communicationManager->armAll();
}

void GroundControlStation::setModeAll(const std::string& mode) const {
    m_communicationManager->setModeAll(mode);
}

void GroundControlStation::startController() const {
    m_controlInterface->start();
}

void GroundControlStation::initLaunch() const {
    m_controlInterface->initLaunch();
}

void GroundControlStation::fetchParam(const int sysId) const {
    m_communicationManager->fetchParam(sysId);
}

void GroundControlStation::loadTrajectory(const std::string& file) const {
    if (m_gcsConfig.controlMode == ControlMode::MPC) {
        LOG_INFO("Loading trajectory ...");
        m_controlInterface->loadTrajectory(file);
    } else {
        LOG_INFO("Control mode MPC is required!");
    }
}

void GroundControlStation::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    m_controlInterface->setOrigin(latitudeDegrees, longitudeDegrees, altitude);
}

void GroundControlStation::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    m_controlInterface->debugConvert(latitudeDegrees, longitudeDegrees, altitude);
}

void GroundControlStation::m_parseCommandFile(const std::string& file) const {
    std::ifstream fileStream(file);

    if (!fileStream.is_open()) {
        LOG_ERROR("Failed to open file");
    }

    std::map<uint8_t, std::vector<uavCommandsFlags>> data;
    std::string line;

    while (std::getline(fileStream, line)) {
        uavCommandsFlags commands{};

        if (m_parseUavCommandsLine(line, commands)) {
            data[static_cast<uint8_t>(commands.commands.sysId)].push_back(commands);

        } else {
            LOG_ERROR("Failed to parse command line from file");
        }
    }

    fileStream.close();

    m_controlInterface->setCommandsList(data);
}

bool GroundControlStation::m_parseUavCommandsLine(const std::string& line, uavCommandsFlags& commands) {
    std::istringstream lineStream(line);
    std::string token;

    // Line definition:
    // time (sec), sysId, roll (deg), pitch (deg), yaw (deg), thrust (N), F1 (0/1), F2 (0/1)

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.timestamp = std::stod(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.commands.sysId = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.commands.rollDegree = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.commands.pitchDegree = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.commands.yawDegree = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.commands.thrust = std::stof(token);

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.F1Command = static_cast<bool>(std::stoi(token));

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.F2Command = static_cast<bool>(std::stoi(token));

    if (!std::getline(lineStream, token, ',')) {
        return false;
    }
    commands.F3Command = static_cast<bool>(std::stoi(token));

    return true;
}

void GroundControlStation::m_supervisorLoop() {

    LOG_INFO("GroundControlStation main loop started.");

    // Enable and start logging
    Logger::instance().start(true, "logGcs" + Logger::getDateString());

    m_communicationManager->start();
    m_controlDispatcher->start();

    while (m_running) {
        // Monitor system state / heartbeat / stats
        //m_communicationManager.pollStatus();

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LOG_INFO("Supervisor loop stopping...");
    m_communicationManager->stop();
    m_controlDispatcher->stop();
    m_controlInterface->stop();
}