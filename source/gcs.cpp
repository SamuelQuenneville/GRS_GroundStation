/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gcs.h"

#include <algorithm>
#include <cmath>
#include <ranges>

GroundControlStation::GroundControlStation()
    : m_running(false)
{
    m_controlDispatcher    = std::make_unique<ControlDispatcher>();
    m_communicationManager = std::make_unique<CommunicationManager>();
    m_controlInterface     = std::make_unique<ControlInterface>();

    // Link communication → dispatcher → controller
    m_communicationManager->setTelemetryCallback([this](const std::map<uint8_t, uavStates>& states) {
        m_controlDispatcher->updateTelemetry(states);

        if (!m_dashboardServer) return;

        {
            std::lock_guard lock(m_dashboardMutex);
            for (const auto& [sysId, state] : states) {
                m_latestUavStates[sysId] = state;
            }
        }

        for (const auto& sysId : states | std::views::keys) {
            m_pushDashboardSnapshot(sysId);
        }
    });

    // Non-numeric status (health, battery, GPS, RC, armed, mode, connection)
    // arrives separately and at a much lower rate -- merge it with whatever
    // numeric state we already have and re-push the full snapshot.
    m_communicationManager->setStatusCallback([this](const std::map<uint8_t, uavHealth>& healthMap) {
        if (!m_dashboardServer) return;

        {
            std::lock_guard lock(m_dashboardMutex);
            for (const auto& [sysId, health] : healthMap) {
                m_latestUavHealth[sysId] = health;
            }
        }

        for (const auto& sysId : healthMap | std::views::keys) {
            m_pushDashboardSnapshot(sysId);
        }
    });

    // Dedicated NMPC controller debug/health panel, decoupled from any UAV.
    m_controlInterface->setNmpcDebugCallback([this](const NMPCController::DebugInfo& info) {
        if (!m_dashboardServer) return;

        NmpcTelemetrySnapshot snap;
        snap.launched        = info.launched;
        snap.inFlight        = info.inFlight;
        snap.endedTraj       = info.endedTraj;
        snap.violation       = info.violation;
        snap.lastSolveMs     = info.lastSolveMs;
        snap.trackingNumber  = info.trackingNumber;
        snap.trajectoryIndex = info.trajectoryIndex;
        snap.trajectoryTotal = info.trajectoryTotal;

        m_dashboardServer->updateNmpcTelemetry(snap);
    });

    // Setup/orientation 3D view (dashboard/setup3d.html) reads these once
    // via GET /api/origin and GET /api/trajectory, not the WebSocket --
    // neither changes at telemetry rates, so just cache the latest and
    // let the HTTP handler serve it on request.
    m_controlInterface->setOriginCallback([this](const double lat, const double lon, const double alt) {
        if (!m_dashboardServer) return;

        OriginSnapshot snap;
        snap.hasOrigin = true;
        snap.latitude  = lat;
        snap.longitude = lon;
        snap.altitude  = alt;
        m_dashboardServer->setOrigin(snap);
    });

    m_controlInterface->setTrajectoryLoadedCallback([this]() {
        if (!m_dashboardServer) return;
        m_dashboardServer->setTrajectory(m_buildTrajectorySnapshotFromController());
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

    m_catapultLauncher = std::make_unique<CatapultLauncher>();
    m_catapultLauncher->setStatusCallback([this](const uint8_t id, CatapultState state, const uint32_t bits) {
        LOG_DEBUG("Catapult " + std::to_string(id) + " -> state=" + std::to_string(static_cast<int>(state)) + " bits=0x" + std::to_string(bits));

        if (!m_dashboardServer) return;

        LauncherTelemetrySnapshot snap;
        snap.id          = std::to_string(id);
        snap.state       = m_catapultStateToString(state);
        snap.connected   = state != CatapultState::Disconnected;
        snap.cocked      = bits & STATUS_COCKED;
        snap.armed       = bits & STATUS_ARMED;
        snap.countdown   = bits & STATUS_COUNTDOWN;
        snap.lowBattery  = bits & STATUS_LOW_BATTERY;
        snap.safetyPinIn = bits & STATUS_SAFETY_PIN_IN;
        snap.gcsTimeout  = bits & STATUS_GCS_TIMEOUT;
        snap.battery     = catapultUnpackBatteryPct(bits);

        m_dashboardServer->updateLauncherTelemetry(snap);
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

    if (!config.catapults.empty()) {
        std::vector<CatapultEndpoint> endpoints;
        for (const auto&[id, port, ip] : config.catapults) {
            endpoints.push_back({id, port, ip});
        }
        m_catapultLauncher->configure(endpoints);
    }
}

void GroundControlStation::setDashboard(DashboardServer* dashboard) {
    m_dashboardServer = dashboard;
    if (!m_dashboardServer) return;

    m_dashboardServer->setTrajectoryGeneratorDefaults(TrajectoryGenerationParams{});

    // Pure preview: computes a mission and converts it, without touching
    // m_controlInterface/m_nmpc at all. Safe to call before the controller
    // is even running.
    m_dashboardServer->setTrajectoryGenerateHandler([this](const TrajectoryGenerationParams& params) {
        const auto config = m_paramsToTrajectoryConfig(params);
        const auto mission = m_controlInterface->previewTrajectory(config);
        return m_missionToTrajectorySnapshot(mission);
    });

    // Commits: loads the generated trajectory into the NMPC controller, then
    // re-reads it back out (rather than reusing the just-generated mission)
    // so the response -- and the WebSocket-independent GET /api/trajectory
    // path -- always reflect what's actually loaded in the controller.
    m_dashboardServer->setTrajectoryApplyHandler([this](const TrajectoryGenerationParams& params) {
        const auto config = m_paramsToTrajectoryConfig(params);
        generateTrajectory(config);
        return m_buildTrajectorySnapshotFromController();
    });
}

void GroundControlStation::start() {
    if (m_running) {
        LOG_WARNING("GroundControlStation already running.");
        return;
    }

    m_running = true;
    m_supervisorThread = std::thread(&GroundControlStation::m_supervisorLoop, this);
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

    LOG_INFO("Starting connection...");

    if (m_gcsConfig.pixhawk.sitl) {
        m_communicationManager->connectAll(m_gcsConfig.pixhawk.remoteIP, m_gcsConfig.pixhawk.tcpPort, m_gcsConfig.numUavs, m_gcsConfig.pixhawk.tcpPortIncrement);
    } else if (!m_gcsConfig.pixhawkEndpoints.empty()) {
        m_communicationManager->connectAll(m_gcsConfig.pixhawkEndpoints);
    } else {
        LOG_ERROR("Real-hardware mode needs Pixhawk.endpoints in config (or set sitl: true).");
        return;
    }

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
        LOG_WARNING("Control mode [MPC] is required to load a trajectory!");
    }
}

void GroundControlStation::generateTrajectory(const grs::trajgen::TrajectoryConfig& config) const {
    if (m_gcsConfig.controlMode == ControlMode::MPC) {
        LOG_INFO("Generating trajectory in-process ...");
        m_controlInterface->generateTrajectory(config);
    } else {
        LOG_WARNING("Control mode [MPC] is required to generate a trajectory!");
    }
}

TrajectorySnapshot GroundControlStation::m_buildTrajectorySnapshotFromController() const {
    static const char* uavColors[] = {"#ef4444", "#f59e0b", "#a78bfa", "#2dd4bf"};

    TrajectorySnapshot snap;
    const int numUavs = m_controlInterface->numUavs();
    for (int i = 0; i < numUavs; ++i) {
        TrajectoryVehicleSnapshot vehicle;
        vehicle.id    = "uav" + std::to_string(i + 1);
        vehicle.label = "UAV " + std::to_string(i + 1);
        vehicle.color = uavColors[i % 4];
        for (const auto& p : m_controlInterface->getTrajectoryForVehicle(i)) {
            TrajectoryPointJson pt;
            pt.north = p.north; pt.east = p.east; pt.down = p.down;
            pt.vx = p.vx; pt.vy = p.vy; pt.vz = p.vz;
            pt.roll = p.roll; pt.pitch = p.pitch;
            vehicle.points.push_back(pt);
        }
        snap.vehicles.push_back(vehicle);
    }
    if (m_controlInterface->trajectoryHasPayload()) {
        TrajectoryVehicleSnapshot vehicle;
        vehicle.id = "payload";
        vehicle.label = "Payload";
        vehicle.color = "#3ecf6e";
        for (const auto& p : m_controlInterface->getTrajectoryForVehicle(numUavs)) {
            TrajectoryPointJson pt;
            pt.north = p.north; pt.east = p.east; pt.down = p.down;
            pt.vx = p.vx; pt.vy = p.vy; pt.vz = p.vz;
            pt.roll = p.roll; pt.pitch = p.pitch;   // stay 0, payload has no attitude state
            vehicle.points.push_back(pt);
        }
        snap.vehicles.push_back(vehicle);
    }

    return snap;
}

TrajectorySnapshot GroundControlStation::m_missionToTrajectorySnapshot(const grs::trajgen::GeneratedMission& mission) {
    static const char* uavColors[] = {"#ef4444", "#f59e0b", "#a78bfa", "#2dd4bf"};

    TrajectorySnapshot snap;
    const size_t numUavs = mission.aircraft.size();
    for (size_t i = 0; i < numUavs; ++i) {
        TrajectoryVehicleSnapshot vehicle;
        vehicle.id    = "uav" + std::to_string(i + 1);
        vehicle.label = "UAV " + std::to_string(i + 1);
        vehicle.color = uavColors[i % 4];

        const auto& timeline = mission.aircraft[i].inertial;
        const auto& controls = mission.controls[i];
        const size_t n = std::min(timeline.size(), controls.size());
        vehicle.points.reserve(n);
        for (size_t s = 0; s < n; ++s) {
            const auto& k = timeline[s];
            const auto& c = controls[s];
            TrajectoryPointJson pt;
            pt.north = k.pos[0]; pt.east = k.pos[1]; pt.down = k.pos[2];
            pt.vx = k.vel[0]; pt.vy = k.vel[1]; pt.vz = k.vel[2];
            // Matches toSolverReference()/getTrajectoryForVehicle(): roll/pitch
            // stay in radians here too, so a preview and its post-Apply
            // reload (GET /api/trajectory) render identically.
            pt.roll = c.rollRad; pt.pitch = c.pitchRad;
            vehicle.points.push_back(pt);
        }
        snap.vehicles.push_back(vehicle);
    }

    if (!mission.payload.empty()) {
        TrajectoryVehicleSnapshot vehicle;
        vehicle.id = "payload";
        vehicle.label = "Payload";
        vehicle.color = "#3ecf6e";
        vehicle.points.reserve(mission.payload.size());
        for (const auto& k : mission.payload) {
            TrajectoryPointJson pt;
            pt.north = k.pos[0]; pt.east = k.pos[1]; pt.down = k.pos[2];
            pt.vx = k.vel[0]; pt.vy = k.vel[1]; pt.vz = k.vel[2];
            pt.roll = 0.0; pt.pitch = 0.0; // payload has no attitude state
            vehicle.points.push_back(pt);
        }
        snap.vehicles.push_back(vehicle);
    }

    return snap;
}

grs::trajgen::TrajectoryConfig GroundControlStation::m_paramsToTrajectoryConfig(const TrajectoryGenerationParams& params) {
    grs::trajgen::TrajectoryConfig config; // start from config.m-mirrored defaults

    config.aircraftPath.radius  = params.radiusMeters;
    config.aircraftPath.velMean = params.velMeanMetersPerSecond;

    config.payloadPath.distanceClimb = params.climbDistanceMeters;
    config.payloadPath.velClimb      = params.climbVelMetersPerSecond;
    config.payloadPath.accClimb      = params.climbAccelMetersPerSecondSq;

    config.payloadPath.distanceMove  = params.moveDistanceMeters;
    config.payloadPath.velMove       = params.moveVelMetersPerSecond;
    config.payloadPath.accMove       = params.moveAccelMetersPerSecondSq;
    config.payloadPath.angleMoveDeg  = params.moveAngleDegrees;

    config.payloadPath.timehold = params.holdTimeSeconds;
    config.tether.length        = params.tetherLengthMeters;
    config.payload.mass         = params.payloadMassKg;

    config.fieldHeadingDeg  = params.fieldHeadingDeg;
    config.originOffsetNed  = grs::Vec3d(params.originNorthOffsetMeters, params.originEastOffsetMeters, params.originDownOffsetMeters);

    config.finalize();
    return config;
}

void GroundControlStation::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    m_controlInterface->setOrigin(latitudeDegrees, longitudeDegrees, altitude);
}

void GroundControlStation::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    m_controlInterface->debugConvert(latitudeDegrees, longitudeDegrees, altitude);
}

void GroundControlStation::startRtkBase(const std::string& device, const unsigned baudrate) {
    std::string resolvedDevice = device;

    if (device == "auto") {
        resolvedDevice = RtkBaseStation::findFirstMatchingPort();
        if (resolvedDevice.empty()) {
            LOG_ERROR("RTK base station: no u-blox USB serial device found");
            return;
        }
        LOG_INFO("RTK base station: auto-detected " + resolvedDevice);
    }

    if (!m_rtkBaseStation) {
        m_rtkBaseStation = std::make_unique<RtkBaseStation>();
    }

    const bool started = m_rtkBaseStation->start(resolvedDevice, baudrate, [this](const std::vector<uint8_t>& rtcmData) {
            m_communicationManager->sendRtcmData(rtcmData);
        });

    if (!started) {
        LOG_ERROR("Failed to start RTK base station on " + resolvedDevice);
    }
}

void GroundControlStation::stopRtkBase() const {
    if (m_rtkBaseStation) {
        m_rtkBaseStation->stop();
    }
}

void GroundControlStation::catapultConnect() const {
    if (!m_catapultLauncher->connectAll()) {
        LOG_WARNING("Not all catapults connected — check IPs/Wi-Fi before arming.");
    }
}

void GroundControlStation::catapultArm() const {
    [[maybe_unused]] auto res = m_catapultLauncher->armAll();
}

void GroundControlStation::catapultFire(const uint32_t countdownMs) const {
    [[maybe_unused]] auto res = m_catapultLauncher->fireAll(countdownMs);
}

void GroundControlStation::catapultAbort() const {
    m_catapultLauncher->abortAll();
}

void GroundControlStation::catapultDisarm() const {
    m_catapultLauncher->disarmAll();
}

void GroundControlStation::catapultStatus() const {
    for (const uint8_t id : m_catapultLauncher->configureIds()) {
        LOG_INFO(m_catapultLauncher->describeStatus(id));
    }
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

void GroundControlStation::m_pushDashboardSnapshot(const uint8_t sysId) {
    uavStates state{};
    uavHealth health{};
    bool haveState = false;

    {
        std::lock_guard lock(m_dashboardMutex);

        if (const auto it = m_latestUavStates.find(sysId); it != m_latestUavStates.end()) {
            state = it->second;
            haveState = true;
        }

        if (const auto it = m_latestUavHealth.find(sysId); it != m_latestUavHealth.end()) {
            health = it->second;
        }
    }

    // Nothing to show yet for this UAV -- wait for the first numeric
    // telemetry sample before creating its panel.
    if (!haveState) return;

    UavTelemetrySnapshot snap;
    snap.id          = "UAV-" + std::to_string(sysId);
    snap.connected   = health.isConnected;
    snap.armed       = health.isArmed;
    snap.mode        = m_flightModeToString(health.flightMode);

    snap.airspeed    = state.airspeedMeterSecond;
    snap.groundspeed = std::hypot(state.northMeterSecond, state.eastMeterSecond);
    snap.altitude    = state.altitudeAmslMeter;
    snap.roll        = state.rollDegree;
    snap.pitch       = state.pitchDegree;
    // rpm / cl: not currently published over MAVSDK telemetry (no
    // subscription wired for them yet) -- left at 0 until that's added.

    snap.battery  = health.batteryRemainingPercent * 100.0;
    // gpsHdop: mavsdk::Telemetry::GpsInfo doesn't expose HDOP, only fix type
    // and satellite count -- left at 0 until/unless you pull it from a raw
    // GPS_RAW_INT mavlink subscription instead.
    snap.gpsFix   = m_gpsFixToString(health.gpsFixType);
    snap.satellites = health.gpsNumSatellites;
    snap.rcSignal = health.rcAvailable ? health.rcSignalPercent : 0.0;
    snap.linkQuality = !health.isConnected ? "Offline"
                      : health.rcAvailable && health.rcSignalPercent >= 80.0f ? "Excellent"
                      : health.rcAvailable && health.rcSignalPercent >= 50.0f ? "Good"
                      : "Poor";

    // MAVSDK's Telemetry::Health doesn't break out barometer/battery/RC
    // individually, so those three are best-effort derived here rather than
    // read straight off the struct -- adjust the thresholds/mapping to
    // taste.
    snap.health.imu     = (health.health.is_gyrometer_calibration_ok && health.health.is_accelerometer_calibration_ok)
                           ? HealthStatus::Ok : HealthStatus::Fail;
    snap.health.compass = health.health.is_magnetometer_calibration_ok ? HealthStatus::Ok : HealthStatus::Fail;
    snap.health.gps     = health.health.is_global_position_ok ? HealthStatus::Ok : HealthStatus::Warn;
    snap.health.baro    = health.health.is_local_position_ok ? HealthStatus::Ok : HealthStatus::Warn;
    snap.health.battery = health.batteryRemainingPercent <= 0.10f ? HealthStatus::Fail
                         : health.batteryRemainingPercent <= 0.25f ? HealthStatus::Warn
                         : HealthStatus::Ok;
    snap.health.rc      = health.rcAvailable ? HealthStatus::Ok : HealthStatus::Warn;

    m_dashboardServer->updateTelemetry(snap);
}

std::string GroundControlStation::m_flightModeToString(const mavsdk::Telemetry::FlightMode mode) {
    switch (mode) {
        case mavsdk::Telemetry::FlightMode::Ready:        return "READY";
        case mavsdk::Telemetry::FlightMode::Takeoff:      return "TAKEOFF";
        case mavsdk::Telemetry::FlightMode::Hold:         return "HOLD";
        case mavsdk::Telemetry::FlightMode::Mission:      return "MISSION";
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return "RTL";
        case mavsdk::Telemetry::FlightMode::Land:         return "LAND";
        case mavsdk::Telemetry::FlightMode::Offboard:     return "OFFBOARD";
        case mavsdk::Telemetry::FlightMode::FollowMe:     return "FOLLOW_ME";
        case mavsdk::Telemetry::FlightMode::Manual:       return "MANUAL";
        case mavsdk::Telemetry::FlightMode::Altctl:       return "ALTCTL";
        case mavsdk::Telemetry::FlightMode::Posctl:       return "POSCTL";
        case mavsdk::Telemetry::FlightMode::Acro:         return "ACRO";
        case mavsdk::Telemetry::FlightMode::Stabilized:   return "STABILIZED";
        case mavsdk::Telemetry::FlightMode::Rattitude:    return "RATTITUDE";
        default:                                          return "UNKNOWN";
    }
}

std::string GroundControlStation::m_gpsFixToString(const mavsdk::Telemetry::FixType fix) {
    switch (fix) {
        case mavsdk::Telemetry::FixType::NoGps:    return "No GPS";
        case mavsdk::Telemetry::FixType::NoFix:    return "No Fix";
        case mavsdk::Telemetry::FixType::Fix2D:    return "2D Fix";
        case mavsdk::Telemetry::FixType::Fix3D:    return "3D Fix";
        case mavsdk::Telemetry::FixType::FixDgps:  return "DGPS Fix";
        case mavsdk::Telemetry::FixType::RtkFloat: return "RTK Float";
        case mavsdk::Telemetry::FixType::RtkFixed: return "RTK Fixed";
        default:                                   return "Unknown";
    }
}

std::string GroundControlStation::m_catapultStateToString(const CatapultState state) {
    switch (state) {
        case CatapultState::Disconnected: return "Disconnected";
        case CatapultState::Connecting:   return "Connecting";
        case CatapultState::Connected:    return "Connected";
        case CatapultState::Arming:       return "Arming";
        case CatapultState::Armed:        return "Armed";
        case CatapultState::Countdown:    return "Countdown";
        case CatapultState::Launched:     return "Launched";
        case CatapultState::Fault:        return "Fault";
        default:                          return "Unknown";
    }
}

void GroundControlStation::m_supervisorLoop() const {

    LOG_INFO("GroundControlStation main loop started");

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
