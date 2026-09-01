/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "communicationManager.h"

CommunicationManager::CommunicationManager()
    : m_mavsdk(GROUND_STATION)
{
    // mavsdk::log::subscribe([](const mavsdk::log::Level level, const std::string& message, const std::string& file, int line) {
    //     // Returning true from the callback disables printing the message to stdout
    //     return level < mavsdk::log::Level::Warn;
    // });
}

CommunicationManager::~CommunicationManager() {
    stop();
}

void CommunicationManager::initialize(const gcsConfig& config) {
    m_config = config;
}

void CommunicationManager::start() {
    bool expected = false;
    if (!m_running.compare_exchange_strong(expected, true)) return;

    if (m_config.telemetry_publish_hz > 0.0) {
        // start a small publisher thread that will send consolidated snapshots at fixed rate
        const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / m_config.telemetry_publish_hz));
        m_publishThread = std::thread([this, period]() {
            while (m_running.load()) {
                std::this_thread::sleep_for(period);

                // only publish if some aggregator changed since last publish
                if (!m_snapshotDirty.load()) continue;

                std::map<uint8_t, uavStates> snapshot;
                {
                    std::lock_guard lock(m_statesMutex);
                    for (const auto& [id, agg] : m_aggregators) {
                        if (agg) snapshot[id] = agg->getSnapshot();
                    }
                    m_snapshotDirty.store(false);
                }

                if (m_telemetryCallback) {
                    m_telemetryCallback(snapshot);
                }
            }
        });
    }

    LOG_INFO("CommunicationManager started");
}

void CommunicationManager::stop() {
    m_running = false;
    if (m_publishThread.joinable()) m_publishThread.join();

    // unsubscribe & remove connections
    {
        std::lock_guard lock(m_linkMutex);
        for (const auto& [sysId, handle]: m_connectionHandles) {
            try {
                m_unsubscribeMavlink(sysId);
                m_mavsdk.remove_connection(handle);
            } catch ([[maybe_unused]] const std::exception& e) {
                LOG_WARNING("Exception removing connection handle");
            }
        }

        m_messageHandles.clear();
        m_connectionHandles.clear();
        m_links.clear();
    }

    LOG_INFO("CommunicationManager stopped");
}

void CommunicationManager::setTelemetryCallback(std::function<void(const std::map<uint8_t, uavStates>&)> cb) {
    m_telemetryCallback = std::move(cb);
}

void CommunicationManager::setStatusCallback(std::function<void(const std::map<uint8_t, uavHealth>&)> cb) {
    m_statusCallback = std::move(cb);
}

void CommunicationManager::connectAll(const std::string& baseIp, const uint16_t basePort, const int numUavs, const int increment, const int discoveryTimeoutMs) {

    LOG_INFO("Connecting to UAV(s)...");

    for (int i = 0; i < numUavs; ++i) {
        const uint16_t port = basePort + i * increment;
        const std::string uri = "tcpout://" + baseIp + ":" + std::to_string(port);
        LOG_INFO("Adding link");
        addLink(uri, discoveryTimeoutMs);
    }

    LOG_INFO("All UAV links initialized");
}

void CommunicationManager::connectAll(const std::vector<pixhawkEndpointConfig>& endpoints, const int discoveryTimeoutMs) {
    LOG_INFO("Connecting to UAV(s) via explicit endpoints...");
    for (const auto&[id, ip, port] : endpoints) {
        const std::string uri = "udpin://0.0.0.0:" + std::to_string(port);
        LOG_INFO("Adding link for UAV " + std::to_string(id) + " -> " + uri);
        addLink(uri, discoveryTimeoutMs);
    }
    LOG_INFO("All UAV links initialized");
}

void CommunicationManager::armAll() {

    if (m_passthrough.empty()) {
        LOG_WARNING("Arm command ignored: no UAV connected");
        return;
    }

    for (const auto& [sysId, passthrough]: m_passthrough) {

        LOG_INFO("Arming UAV sysId = " + std::to_string(sysId) + " ...");

        mavsdk::MavlinkPassthrough::CommandLong command{};
        command.command = MAV_CMD_COMPONENT_ARM_DISARM;
        command.param1 = 1;
        command.param2 = 2989;
        command.target_sysid = passthrough->get_target_sysid();
        command.target_compid = MAV_COMP_ID_AUTOPILOT1;

        const auto result = passthrough->send_command_long(command);

        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            LOG_WARNING("Arming failed for sysId = " + std::to_string(sysId) + ", result = " + std::to_string(static_cast<int>(result)));
            continue;
        }

        LOG_INFO("Arm command sent successfully to sysId = " + std::to_string(sysId));
    }
}

void CommunicationManager::setMode(const uint8_t sysId, const std::string& mode) {

    const auto it = m_passthrough.find(sysId);

    if (it == m_passthrough.end() || !it->second) {
        LOG_WARNING("Cannot set mode: UAV sysId = " + std::to_string(sysId) + " is not connected");
        return;
    }

    const auto modes = flightModeMap();
    const auto modeIt = modes.find(mode);

    if (modeIt == modes.end()) {
        LOG_WARNING("Cannot set mode for sysId = " + std::to_string(sysId) + ": unknown mode '" + mode + "'");
        return;
    }

    LOG_INFO("Setting UAV sysId = " + std::to_string(sysId) + " to mode " + mode + " ...");

    mavsdk::MavlinkPassthrough::CommandLong command{};
    command.command = MAV_CMD_DO_SET_MODE;
    command.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    command.param2 = modeIt->second;
    command.target_sysid = it->second->get_target_sysid();
    command.target_compid = MAV_COMP_ID_AUTOPILOT1;

    const auto result = it->second->send_command_long(command);

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        LOG_WARNING("Failed to set mode for sysId = " + std::to_string(sysId) + ", mode = " + mode + ", result = " + std::to_string(static_cast<int>(result)));
        return;
    }

    LOG_INFO("Mode command sent successfully to sysId = " + std::to_string(sysId) + ": " + mode);
}

void CommunicationManager::setModeAll(const std::string& mode) {

    if (m_passthrough.empty()) {
        LOG_WARNING("Set mode ignored: no UAV connected");
        return;
    }

    setHomeToCurrentPosition();

    LOG_INFO("Setting mode '" + mode + "' for " + std::to_string(m_passthrough.size()) + " UAV(s) ...");

    for (const auto& sysId : m_passthrough | std::views::keys) {
        setMode(sysId, mode);
    }
}

void CommunicationManager::fetchParam(const int sysId) {
    if (!m_param.contains(sysId)) {
        LOG_INFO("UAV not connected");
        return;
    }

    auto [int_params, float_params, custom_params] = m_param[sysId]->get_all_params();

    std::string fileName = "uav" + std::to_string(sysId) + ".param";
    std::ofstream file(fileName);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file: " + fileName);
        return;
    }

    for (const auto&[name, value] : int_params) {
        file << name << "," << value << "\n";
    }

    for (const auto&[name, value] : float_params) {
        file << name << "," << value << "\n";
    }

    for (const auto&[name, value] : custom_params) {
        file << name << "," << value << "\n";
    }

    file.close();
    LOG_INFO("Params file created");
}

bool CommunicationManager::addLink(const std::string& connection, const int discoveryTimeoutMs) {
    LOG_INFO("Connection: " + connection);
    const auto [connectionResult, connectionHandle] = m_mavsdk.add_any_connection_with_handle(connection);

    if (connectionResult != mavsdk::ConnectionResult::Success) {
        LOG_ERROR("Failed to add link: " + connection);
        return false;
    }

    m_numberOfUavs += 1;

    // Bounded wait: give MAVSDK a chance to discover the new vehicle, but
    // never block the caller (e.g. the console thread handling "connect")
    // forever if it never shows up.
    // The connection itself is left open either way; MAVSDK keeps
    // listening/retrying on it in the background.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(discoveryTimeoutMs);
    while (static_cast<int>(m_mavsdk.systems().size()) < m_numberOfUavs && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (static_cast<int>(m_mavsdk.systems().size()) < m_numberOfUavs) {
        LOG_WARNING("addLink(" + connection + "): no new vehicle discovered within "
            + std::to_string(discoveryTimeoutMs) + "ms -- check the IP/port and that the vehicle "
            "is powered on. The link stays open; re-run 'connect' once it's reachable.");
    }

    for (auto& system : m_mavsdk.systems()) {
        if (!system->is_connected()) {
            continue;
        }

        uint8_t sysId = system->get_system_id();
        m_vehicleType[sysId] = system->vehicle_type();

        m_aggregators[sysId] = std::make_shared<StatesAggregator>();

        std::lock_guard lock(m_linkMutex);
        if (!m_links.contains(sysId)) {
            m_links[sysId] = system;
            m_connectionHandles[sysId] = connectionHandle;

            m_telemetry[sysId]   = std::make_shared<mavsdk::Telemetry>(system);
            m_action[sysId]      = std::make_shared<mavsdk::Action>(system);
            m_param[sysId]       = std::make_shared<mavsdk::Param>(system);
            m_passthrough[sysId] = std::make_shared<mavsdk::MavlinkPassthrough>(system);
            m_rtk[sysId]         = std::make_shared<mavsdk::Rtk>(system);

            {
                std::lock_guard statesLock(m_statesMutex);
                m_uavHealths[sysId].isConnected = true;
            }

            m_subscribeMavlink(sysId);
            m_onStatusUpdate();

            LOG_INFO("Connected: sysID = " + std::to_string(sysId));
        }
    }
    return true;
}

void CommunicationManager::listLinks() {
    std::lock_guard lock(m_linkMutex);
    if (m_links.empty()) {
        LOG_WARNING("No links connected");
    } else {
        for (const auto& sysId : m_links | std::views::keys) {
            LOG_INFO("Connected to sysId = " + std::to_string(sysId));
        }
    }
}

std::shared_ptr<mavsdk::Telemetry> CommunicationManager::getTelemetry(const uint8_t sysId) {
    std::lock_guard lock(m_linkMutex);
    if (m_telemetry.contains(sysId)) {
        return m_telemetry[sysId];
    }
    return nullptr;
}

std::shared_ptr<mavsdk::Action> CommunicationManager::getAction(const uint8_t sysId) {
    std::lock_guard lock(m_linkMutex);
    if (m_links.contains(sysId)) {
        return m_action[sysId];
    }
    return nullptr;
}

void CommunicationManager::setHomeToCurrentPosition() {

    for (const auto& [sysId, system]: m_links) {
        mavsdk::MavlinkPassthrough mavlink_passthrough{system};
        auto telemetry = m_telemetry[sysId];

        // Create MAV_CMD_DO_SET_HOME command
        mavsdk::MavlinkPassthrough::CommandLong commandLong{};
        commandLong.command = MAV_CMD_DO_SET_HOME;
        commandLong.param1 = 1;  // 1 = Set home at current position
        commandLong.target_sysid = sysId;
        commandLong.target_compid = MAV_COMP_ID_AUTOPILOT1;

        // Send the command
        auto result = mavlink_passthrough.send_command_long(commandLong);
        if (result == mavsdk::MavlinkPassthrough::Result::Success) {
            LOG_INFO("Home position set to current location");
        } else {
            LOG_WARNING("Failed to set home position set to current location, result = " + std::to_string(static_cast<int>(result)));
        }
    }
}

void CommunicationManager::setUavCommands(const std::map<uint8_t, uavCommandsFlags>& uavCommands) {
    {
        std::lock_guard lock(m_statesMutex);
        m_uavCommands = uavCommands;
    }

    m_sendAttitudeTarget();
}

void CommunicationManager::sendRtcmData(const std::vector<uint8_t>& data) {
    std::lock_guard lock(m_linkMutex);

    if (m_rtk.empty()) {
        LOG_DEBUG("sendRtcmData called but no UAV is connected yet");
        return;
    }

    // mavsdk::base64_encode() takes a non-const std::vector<uint8_t>&, so we
    // need a mutable copy even though sendRtcmData() itself takes const&.
    std::vector<uint8_t> encodableData = data;

    mavsdk::Rtk::RtcmData rtcmData;
    rtcmData.data_base64 = mavsdk::base64_encode(encodableData);

    for (const auto& [sysId, rtk] : m_rtk) {
        const auto result = rtk->send_rtcm_data(rtcmData);
        if (result != mavsdk::Rtk::Result::Success) {
            LOG_WARNING("Failed to send RTCM data to sysId = " + std::to_string(sysId));
        }
    }
}

void CommunicationManager::m_subscribeMavlink(const uint8_t sysId) {
    const auto telemetryIterator = m_telemetry.find(sysId);

    if (telemetryIterator == m_telemetry.end()) {
        LOG_ERROR("Telemetry not found for sysId = " + std::to_string(sysId));
        return;
    }

    const auto telemetry = telemetryIterator->second;
    subscriptionHandles handles;

    // HEARTBEAT
    m_subscribeHealth(telemetry, sysId, handles);
    m_subscribeHealthAllOk(telemetry, sysId, handles);
    m_subscribeArmed(telemetry, sysId, handles);
    m_subscribeFlightMode(telemetry, sysId, handles);
    m_subscribeBattery(telemetry, sysId, handles);
    m_subscribeGpsInfo(telemetry, sysId, handles);
    m_subscribeRcStatus(telemetry, sysId, handles);

    // LOCAL_POSITION_NED
    m_subscribePositionVelocity(telemetry, sysId, handles);

    // GLOBAL_POSITION_INT
    m_subscribePosition(telemetry, sysId, handles);

    // Command Ack
    m_subscribeHome(telemetry, sysId, handles);
    m_subscribeCommandAck(sysId);
    m_subscribeToHeartbeat(sysId);

    if (m_vehicleType[sysId] == mavsdk::Vehicle::FixedWing) {
        // ATTITUDE
        m_subscribeAttitude(telemetry, sysId, handles);

        // VFR_HUD
        m_subscribeFixedwingMetrics(telemetry, sysId, handles);

        m_requestAttitudeTarget(sysId);
        m_subscribeAttitudeTarget(sysId);
    }

    m_messageHandles[sysId] = handles;
}

void CommunicationManager::m_unsubscribeMavlink(const uint8_t sysId) {
    const auto handleIterator = m_messageHandles.find(sysId);
    const auto telemetryIterator = m_telemetry.find(sysId);

    if (telemetryIterator == m_telemetry.end()) {
        LOG_ERROR("Telemetry not found for sysId = " + std::to_string(sysId));
        return;
    }

    const auto telemetry = telemetryIterator->second;

    //const auto telemetry = telemetryIterator->second;
    const auto handles = handleIterator->second;

    // HEARTBEAT
    telemetry->unsubscribe_health(handles.healthHandle);
    telemetry->unsubscribe_health_all_ok(handles.healthAllOkHandle);
    telemetry->unsubscribe_armed(handles.armedHandle);
    telemetry->unsubscribe_flight_mode(handles.flightModeHandle);
    telemetry->unsubscribe_battery(handles.batteryHandle);
    telemetry->unsubscribe_gps_info(handles.gpsInfoHandle);
    telemetry->unsubscribe_rc_status(handles.rcStatusHandle);

    {
        std::lock_guard statesLock(m_statesMutex);
        m_uavHealths[sysId].isConnected = false;
    }
    m_onStatusUpdate();

    // LOCAL_POSITION_NED
    telemetry->unsubscribe_position_velocity_ned(handles.positionVelocityNedHandle);

    // GLOBAL_POSITION_INT
    telemetry->unsubscribe_position(handles.positionHandle);

    if (m_vehicleType[sysId] == mavsdk::Vehicle::FixedWing) {
       // ATTITUDE
        telemetry->unsubscribe_attitude_euler(handles.attitudeHandle);

        // VFR_HUD
        telemetry->unsubscribe_fixedwing_metrics(handles.fixedwingMetricsHandle);
    }

    // Remove from map
    m_messageHandles.erase(sysId);
}

void CommunicationManager::m_onTelemetryUpdate() {
    std::map<uint8_t, uavStates> snapshot;
    {
        std::lock_guard lock(m_statesMutex);
        for (const auto& [id, agg] : m_aggregators) {
            snapshot[id] = agg->getSnapshot();
        }
    }

    if (m_telemetryCallback) {
        m_telemetryCallback(snapshot);
    }
}

void CommunicationManager::m_onStatusUpdate() {
    std::map<uint8_t, uavHealth> snapshot;
    {
        std::lock_guard lock(m_statesMutex);
        snapshot = m_uavHealths;
    }

    if (m_statusCallback) {
        m_statusCallback(snapshot);
    }
}

void CommunicationManager::m_handleCommandAck(const mavlink_message_t& message) {
    std::thread([this, message]() {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&message, &ack);

        std::lock_guard lock(m_commandAckMutex);
        m_lastAck = ack;

        if (m_lastAck.command == MAVLINK_MSG_ID_ATTITUDE_TARGET) {
           LOG_DEBUG("ATTITUDE ACK");
        }

        m_cvCommandAck.notify_all();
    }).detach();
}

void CommunicationManager::m_subscribeCommandAck(const uint8_t sysId) {
    m_passthrough[sysId]->subscribe_message(MAVLINK_MSG_ID_COMMAND_ACK,
        [this](const mavlink_message_t& message) { m_handleCommandAck(message); });
}

void CommunicationManager::m_handleHeartbeat(const mavlink_message_t& message) {
    std::thread([this, message]() {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&message, &heartbeat);

        m_currentMode = heartbeat.custom_mode;
    }).detach();
}

void CommunicationManager::m_subscribeToHeartbeat(const uint8_t sysId) {
    m_passthrough[sysId]->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT,
        [this](const mavlink_message_t& message) { m_handleHeartbeat(message); });
}

void CommunicationManager::m_requestAttitudeTarget(const uint8_t sysId) {
    const auto result = m_passthrough[sysId]->queue_message(
        [&](const MavlinkAddress mavlink_address, const uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_command_long_pack_chan(
                mavlink_address.system_id,
                mavlink_address.component_id,
                channel,
                &message,
                m_passthrough[sysId]->get_target_sysid(),
                m_passthrough[sysId]->get_target_compid(),
                MAV_CMD_SET_MESSAGE_INTERVAL, // Command 511
                0,
                MAVLINK_MSG_ID_ATTITUDE_TARGET, // Message ID 83
                1e6 / 10.0, // Microseconds between messages
                0, 0, 0, 0, 0
            );
            return message;
        });

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        LOG_WARNING("Failed to request ATTITUDE_TARGET stream! Result = " + std::to_string(static_cast<int>(result)));
    } else {
        LOG_INFO("Requested ATTITUDE_TARGET stream at " + std::to_string(10.0) + " Hz");
    }
}

void CommunicationManager::m_handleAttitudeTarget(const mavlink_message_t& message) const {
    std::thread([this, message]() {
        mavlink_attitude_target_t attitudeTarget;
        mavlink_msg_attitude_target_decode(&message, &attitudeTarget);

        LOG_DEBUG("Thrust target from AP = " + std::to_string(attitudeTarget.thrust));
    }).detach();
}

void CommunicationManager::m_subscribeAttitudeTarget(const uint8_t sysId) {
     m_passthrough[sysId]->subscribe_message(MAVLINK_MSG_ID_ATTITUDE_TARGET, [this](const mavlink_message_t& message) {
         m_handleAttitudeTarget(message);
     });
}

void CommunicationManager::m_subscribeHealth(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.healthHandle = telemetry->subscribe_health([this, sysId](const mavsdk::Telemetry::Health& health) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].health = health;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeHealthAllOk(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.healthAllOkHandle = telemetry->subscribe_health_all_ok([this, sysId](const bool isHealthy) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].isHealthy = isHealthy;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeArmed(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.armedHandle = telemetry->subscribe_armed([this, sysId](const bool isArmed) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].isArmed = isArmed;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeBattery(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.batteryHandle = telemetry->subscribe_battery([this, sysId](const mavsdk::Telemetry::Battery& battery) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].batteryRemainingPercent = battery.remaining_percent;
            m_uavHealths[sysId].batteryVoltageVolt = battery.voltage_v;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeGpsInfo(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.gpsInfoHandle = telemetry->subscribe_gps_info([this, sysId](const mavsdk::Telemetry::GpsInfo& gpsInfo) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].gpsNumSatellites = gpsInfo.num_satellites;
            m_uavHealths[sysId].gpsFixType = gpsInfo.fix_type;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeRcStatus(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.rcStatusHandle = telemetry->subscribe_rc_status([this, sysId](const mavsdk::Telemetry::RcStatus& rcStatus) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].rcAvailable = rcStatus.is_available;
            m_uavHealths[sysId].rcSignalPercent = rcStatus.signal_strength_percent;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeHome(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.homeHandle = telemetry->subscribe_home([](const mavsdk::Telemetry::Position &home) {
        LOG_INFO("Home position: Lat = " + std::to_string(home.latitude_deg) + ", Lon = " + std::to_string(home.longitude_deg) + ", Alt = " + std::to_string(home.absolute_altitude_m) + "m");
    });
}


void CommunicationManager::m_subscribeFlightMode(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.flightModeHandle = telemetry->subscribe_flight_mode([this, sysId](const mavsdk::Telemetry::FlightMode& flightMode) {
        {
            std::lock_guard lock(m_statesMutex);
            m_uavHealths[sysId].flightMode = flightMode;
        }
        m_onStatusUpdate();
    });
}

void CommunicationManager::m_subscribeAttitude(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    const mavsdk::Telemetry::Result setRateResult = telemetry->set_rate_attitude_euler(25.0);
    if (setRateResult != mavsdk::Telemetry::Result::Success) {
        LOG_ERROR("Failed to set rate attitude_euler");
    }

    handles.attitudeHandle = telemetry->subscribe_attitude_euler([this, sysId](const mavsdk::Telemetry::EulerAngle& attitude) {
        m_aggregators[sysId]->updateAttitude(attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg);

        if (m_config.telemetry_publish_hz <= 0.0) {
            // immediate publish
            m_onTelemetryUpdate();
        } else {
            m_snapshotDirty.store(true);
        }
    });
}

void CommunicationManager::m_subscribePositionVelocity(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    const mavsdk::Telemetry::Result setRateResult = telemetry->set_rate_position_velocity_ned(25.0);
    if (setRateResult != mavsdk::Telemetry::Result::Success) {
        LOG_ERROR("Failed to set rate position_velocity_ned");
    }

    handles.positionVelocityNedHandle = telemetry->subscribe_position_velocity_ned([this, sysId](const mavsdk::Telemetry::PositionVelocityNed& positionVelocityNed) {
        m_aggregators[sysId]->updatePosition(positionVelocityNed.position.north_m, positionVelocityNed.position.east_m, positionVelocityNed.position.down_m);
        m_aggregators[sysId]->updateVelocity(positionVelocityNed.velocity.north_m_s, positionVelocityNed.velocity.east_m_s, positionVelocityNed.velocity.down_m_s);

        if (m_config.telemetry_publish_hz <= 0.0) {
            // immediate publish
            m_onTelemetryUpdate();
        } else {
            m_snapshotDirty.store(true);
        }
    });
}

void CommunicationManager::m_subscribePosition(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.positionHandle = telemetry->subscribe_position([this, sysId](const mavsdk::Telemetry::Position& position) {
        m_aggregators[sysId]->updateGlobalPosition(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m);

        if (m_config.telemetry_publish_hz <= 0.0) {
            // immediate publish
            m_onTelemetryUpdate();
        } else {
            m_snapshotDirty.store(true);
        }
    });
}

void CommunicationManager::m_subscribeFixedwingMetrics(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.fixedwingMetricsHandle = telemetry->subscribe_fixedwing_metrics([this, sysId](const mavsdk::Telemetry::FixedwingMetrics& fixedwingMetrics) {
        m_aggregators[sysId]->updateAirspeed(fixedwingMetrics.airspeed_m_s);

        if (m_config.telemetry_publish_hz <= 0.0) {
            // immediate publish
            m_onTelemetryUpdate();
        } else {
            m_snapshotDirty.store(true);
        }
    });
}

void CommunicationManager::m_sendAttitudeTarget() {
    // Snapshot commands under lock
    std::map<uint8_t, uavCommandsFlags> commandsCopy;
    {
        std::lock_guard lock(m_statesMutex);
        commandsCopy = m_uavCommands;
    }

    // Iterate over each UAV command
    for (const auto& [sysId, cmd] : commandsCopy) {
        if (!m_passthrough.contains(sysId)) {
            LOG_WARNING("Skipping sysId " + std::to_string(sysId) + ": no passthrough instance");
            continue;
        }

        // Send via MavlinkPassthrough (non-blocking)
        const auto targetSysId  = m_passthrough[sysId]->get_target_sysid();
        const auto targetCompId = m_passthrough[sysId]->get_target_compid();

        const auto result = m_passthrough[sysId]->queue_message([cmd, targetSysId, targetCompId](const MavlinkAddress address, const uint8_t channel) {
            return MavlinkMessageBuilder::buildSetAttitudeTarget(address, channel, targetSysId, targetCompId, cmd);
        });

        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            LOG_WARNING("Failed to queue SET_ATTITUDE_TARGET for sysId " + std::to_string(sysId));
        }
    }
}

void CommunicationManager::m_setParameter(const uint8_t sysId, const MAV_PARAM_TYPE type, std::string name, const float value) {
    const auto result = m_passthrough[sysId]->queue_message(
        [&](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_param_set_pack(
                m_passthrough[sysId]->get_our_sysid(),
                m_passthrough[sysId]->get_our_compid(),
                &message,
                m_passthrough[sysId]->get_target_sysid(),
                MAV_COMP_ID_AUTOPILOT1,
                name.c_str(),
                value,
                type
            );
            return message;
        });

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        LOG_ERROR("Failed to set " + name + ", result = " + std::to_string(static_cast<int>(result)));
    } else {
        LOG_INFO("Successfully sent command to set " + name + " to " + std::to_string(value));
    }
};