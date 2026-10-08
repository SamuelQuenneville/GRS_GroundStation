/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlInterface.h"

#include "mpcController.h"
#include "nmheEstimator.h"

#include <algorithm>
#include <ranges>
#include <stdexcept>

ControlInterface::ControlInterface()
    : m_running(false)
{

}

ControlInterface::~ControlInterface() {
    stop();
    if (m_udpSocketMatlab >= 0) close(m_udpSocketMatlab);
}

void ControlInterface::initialize(const gcsConfig& config) {
    m_config = config;

    if (m_config.controlMode == ControlMode::MPC) {
        YAML::Node node = YAML::LoadFile(config.configPath);

        auto stack = buildControlStack(node);
        // One control step per shooting interval.
        if (std::abs(m_config.hlcFrequency * stack.solver.dt - 1.0) > 1e-6) {
            throw std::runtime_error("hlcFrequency " + std::to_string(m_config.hlcFrequency) + " Hz does not match solver dt " + std::to_string(stack.solver.dt) + " s");
        }
        m_controller = std::move(stack.controller);
        m_estimator = std::move(stack.estimatorInstance);

        // The NMHE gets its own thread at nmheFrequency, fully decoupled from the control loop.
        std::unique_ptr<EstimatorRunner> runner;
        if (m_estimator) {
            runner = std::make_unique<ThreadedEstimatorRunner>(*m_estimator, m_config.nmheFrequency, static_cast<size_t>(stack.estimator->M) + 8);
        }
        m_controlStep = std::make_unique<ControlStep>(*m_controller, std::move(runner), stack.estimator ? stack.estimator->nu : 0);
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

void ControlInterface::setTelemetryAgeProvider(std::function<std::optional<double>(uint8_t)> provider) {
    m_telemetryAge = std::move(provider);
}

bool ControlInterface::m_checkTelemetry(const std::map<uint8_t, uavStates>& navStates) {
    // Every vehicle of the controller's state: the UAVs 1..numUavs and, with a
    // payload, the highest sysId above them (stateVector.h).
    std::vector<uint8_t> ids;
    for (int id = 1; id <= m_controller->numUavs(); ++id) ids.push_back(static_cast<uint8_t>(id));
    if (m_controller->hasPayload()) {
        const auto payload = navStates.upper_bound(static_cast<uint8_t>(m_controller->numUavs()));
        ids.push_back(payload == navStates.end() ? static_cast<uint8_t>(0) : std::prev(navStates.end())->first);
    }

    std::string reason;
    for (const uint8_t id : ids) {
        const auto age = m_telemetryAge ? m_telemetryAge(id) : std::optional<double>(0.0);
        if (id == 0) {
            reason = "no payload telemetry";
        } else if (!navStates.contains(id)) {
            reason = "sysId " + std::to_string(id) + " has no frame offset yet";
        } else if (!age) {
            reason = "no telemetry from sysId " + std::to_string(id);
        } else if (!(*age <= m_config.telemetryTimeout)) {
            reason = "telemetry of sysId " + std::to_string(id) + " is " + std::to_string(*age) + " s old";
        } else {
            continue;
        }
        break;
    }
    const bool stale = !reason.empty();
    if (stale != m_telemetryStale.exchange(stale)) {
        if (stale) LOG_ERROR("Telemetry stale (" + reason + "): no command sent until it recovers");
        else LOG_INFO("Telemetry complete and fresh: commands sent");
    }
    std::lock_guard lock(m_staleMutex);
    m_staleReason = reason;
    return !stale;
}

void ControlInterface::setNmpcDebugCallback(std::function<void(const Controller::DebugInfo&)> cb) {
    m_nmpcDebugCallback = std::move(cb);
}

void ControlInterface::setOriginCallback(std::function<void(double, double, double)> cb) {
    m_originCallback = std::move(cb);
}

void ControlInterface::setTrajectoryLoadedCallback(std::function<void()> cb) {
    m_trajectoryLoadedCallback = std::move(cb);
}

std::vector<Controller::TrajectoryPointView> ControlInterface::getTrajectoryForVehicle(const int vehicleIndex) const {
    return m_controller ? m_controller->getTrajectoryForVehicle(vehicleIndex) : std::vector<Controller::TrajectoryPointView>{};
}

int ControlInterface::numUavs() const {
    return m_controller ? m_controller->numUavs() : 0;
}

bool ControlInterface::trajectoryHasPayload() const {
    return m_controller && m_controller->hasPayload();
}

bool ControlInterface::getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const {
    return m_navFrameManager.getOrigin(latitudeDegrees, longitudeDegrees, altitude);
}

std::optional<ControlInterface::GpsFix> ControlInterface::getPayloadGpsFix() const {
    std::lock_guard lock(m_stateMutex);

    std::optional<GpsFix> fix;
    for (const auto& [sysId, state] : m_latestStates) {
        if (sysId <= m_config.numUavs) {
            continue; // a UAV, not the payload
        }

        // Highest sysId wins if more than one somehow lands above numUavs
        fix = GpsFix{.latitudeDegrees = state.latitudeDegree, .longitudeDegrees = state.longitudeDegree, .altitudeMeters = state.altitudeAmslMeter};
    }
    return fix;
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
    if (m_controller) m_controller->initLaunch();
}

bool ControlInterface::launchReady(std::string& reason) const {
    if (!m_controller) return true;
    if (m_telemetryStale) {
        std::lock_guard lock(m_staleMutex);
        reason = m_staleReason.empty() ? "telemetry not checked yet (controller not running)" : m_staleReason;
        return false;
    }
    return m_navFrameManager.frameReady(m_controller->numUavs(), reason) && m_controller->launchReady(reason);
}

void ControlInterface::setEkfOrigin(const uint8_t sysId, const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    m_navFrameManager.setEkfOrigin(sysId, latitudeDegrees, longitudeDegrees, altitude);
}

void ControlInterface::loadTrajectory(const std::string& file) const {
    m_controller->loadTrajectory(file);
    if (m_trajectoryLoadedCallback) m_trajectoryLoadedCallback();
}

void ControlInterface::saveTrajectory(const std::string& file) const {
    if (!m_controller) {
        throw std::runtime_error("saveTrajectory: control mode [MPC] is required (no NMPC controller instantiated)");
    }
    m_controller->saveTrajectory(file);
}

grs::trajgen::GeneratedMission ControlInterface::previewTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed) {
    const grs::trajgen::TrajectoryGenerator generator(config);
    auto mission = generator.generate();
    grs::trajgen::TrajectoryGenerator::applyFieldCalibration(mission, config.fieldHeadingDeg, config.originOffsetNed);

    // Must run before extractSubset(): liveLaunchPositionsNed is indexed by the full mission's original mission.aircraft[]
    // order, which extractSubset may reorder/drop.
    grs::trajgen::TrajectoryGenerator::snapToLiveLaunchPositions(mission, liveLaunchPositionsNed);
    return grs::trajgen::TrajectoryGenerator::extractSubset(mission, selection);
}

void ControlInterface::generateTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed) const {

    if (!m_controller) {
        LOG_ERROR("generateTrajectory: control mode [MPC] is required (no NMPC controller instantiated)");
        return;
    }

    const auto mission = previewTrajectory(config, selection, liveLaunchPositionsNed);

    if (static_cast<int>(mission.aircraft.size()) != m_controller->numUavs()) {
        throw std::runtime_error("generateTrajectory: mission has " + std::to_string(mission.aircraft.size()) +
            " aircraft but the loaded controller (" + std::to_string(m_controller->numUavs()) +
            " UAV(s)) expects a different count -- wrong --config profile loaded, or a SubsetSelection.uavIndices mismatch?");
    }

    const bool hasPayload = selection.includePayload.value_or(m_controller->hasPayload());
    auto reference = grs::trajgen::TrajectoryGenerator::toSolverReference(mission, hasPayload);
    m_controller->setReferenceTrajectory(std::move(reference));

    if (m_trajectoryLoadedCallback) {
        m_trajectoryLoadedCallback();
    }
}

std::map<uint8_t, uavStates> ControlInterface::getLiveNavigationStates() const {
    if (!m_navFrameManager.isInitialized()) {
        return {};
    }

    std::map<uint8_t, uavStates> states;
    {
        std::lock_guard lock(m_stateMutex);
        states = m_latestStates;
    }
    return m_navFrameManager.toNavigationFrame(states);
}

void ControlInterface::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    m_navFrameManager.setOrigin(latitudeDegrees, longitudeDegrees, altitude);

    if (m_originCallback) {
        m_originCallback(latitudeDegrees, longitudeDegrees, altitude);
    }
}

void ControlInterface::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    m_navFrameManager.debugConvert(latitudeDegrees, longitudeDegrees, altitude);
}

void ControlInterface::m_controlLoop() {
    int fileIdx = 0;
    int failedTicks = 0;

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / m_config.hlcFrequency));
    auto next = std::chrono::steady_clock::now();

    while (m_running) {
        next += period;

        auto now = std::chrono::steady_clock::now();

        if (now < next) {
            std::this_thread::sleep_until(next);
        } else {
            LOG_WARNING("Control loop running slow");
            next = now;
        }

        // A failed tick sends no command (the autopilot's command timeout then
        // applies); the loop keeps running.
        try {
            std::map<uint8_t, uavStates> latestStates;
            {
                std::lock_guard lock(m_stateMutex);
                latestStates = m_latestStates;
            }
            const double time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

            // Every tick: adds the offset of systems that connected since (see initializeOffset()).
            m_navFrameManager.initializeOffset(latestStates);

            if (m_navFrameManager.isInitialized()) {
                std::map<uint8_t, uavCommandsFlags>  cmds;

                auto navStates = m_navFrameManager.toNavigationFrame(latestStates);

                if (m_config.controlMode == ControlMode::MATLAB) {
                    m_sendDataToMatlab(navStates);
                    auto output = m_receiveDataFromMatlab();

                    for (size_t i = 0; i < output.size(); i++) {
                        cmds[i+1].commands = output[i+1];
                    }

                } else if (m_config.controlMode == ControlMode::MPC) {
                    // Stale telemetry: no solve and no command, the autopilot's
                    // command timeout applies until it recovers.
                    if (!m_checkTelemetry(navStates)) continue;

                    // Commands in physical units.
                    cmds = m_controlStep->tick(navStates, time);

                    for (auto& [sysId, cmd] : cmds) {
                        cmd.commands.thrust = static_cast<float>(thrust2rpm(navStates.at(sysId).airspeedMeterSecond, cmd.commands.thrust));
                    }

                    if (m_nmpcDebugCallback) {
                        m_nmpcDebugCallback(m_controller->getDebugInfo());
                    }

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
                    LOG_ERROR("Not a valid control mode. Options are Matlab/MPC/AttitudeFile");
                }

                if (m_sendCommand) {
                    m_sendCommand(cmds);
                }

            }
        } catch (const std::exception& e) {
            if (failedTicks++ % std::max(1, static_cast<int>(m_config.hlcFrequency)) == 0) {
                LOG_ERROR(std::string("Control tick failed, no command sent: ") + e.what() + " (" + std::to_string(failedTicks) + " ticks)");
            }
            continue;
        }
        failedTicks = 0;
    }
}

void ControlInterface::m_initMatlabConnection(const char* ip, const uint16_t port) {

    m_udpSocketMatlab = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSocketMatlab < 0) {
        LOG_ERROR("Failed to init matlab udp socket");
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
           reinterpret_cast<sockaddr *>(&m_matlabAddress), sizeof(m_matlabAddress));

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
