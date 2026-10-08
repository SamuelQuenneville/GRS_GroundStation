/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "communicationManager.h"

#include <algorithm>
#include <fstream>
#include <ranges>

CommunicationManager::CommunicationManager()
    : m_mavsdk(GROUND_STATION)
{
    // For the lifetime of this object: fires whenever MAVSDK sees a new system on any link, at any time (see m_watchSystem).
    m_newSystemHandle = m_mavsdk.subscribe_on_new_system([this]() {
        for (const auto& system : m_mavsdk.systems()) {
            m_watchSystem(system);
        }
    });
}

CommunicationManager::~CommunicationManager() {
    m_mavsdk.unsubscribe_on_new_system(m_newSystemHandle);
    stop();
}

void CommunicationManager::initialize(const gcsConfig& config) {
    m_config = config;
}

void CommunicationManager::start() {
    bool expected = false;
    if (!m_running.compare_exchange_strong(expected, true)) return;

    if (m_config.telemetry_publish_hz > 0.0) {
        // Consolidated snapshots at a fixed rate, only when something changed.
        const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / m_config.telemetry_publish_hz));
        m_publishThread = std::thread([this, period]() {
            while (m_running.load()) {
                std::this_thread::sleep_for(period);
                if (m_snapshotDirty.exchange(false)) m_onTelemetryUpdate();
            }
        });
    }

    LOG_INFO("CommunicationManager started");
}

void CommunicationManager::stop() {
    m_running = false;
    if (m_publishThread.joinable()) m_publishThread.join();

    // Everything is moved out under the lock and released outside it: the
    // MAVSDK callbacks take the same locks while running.
    std::map<uint8_t, Watcher> watchers;
    {
        std::lock_guard lock(m_linkMutex);
        watchers.swap(m_watchers);
    }
    for (auto& watcher : watchers | std::views::values) {
        if (watcher.handle) watcher.system->unsubscribe_is_connected(*watcher.handle);
    }

    // Registrations in progress finish before their vehicle is released.
    std::vector<std::thread> registrations;
    {
        std::lock_guard lock(m_linkMutex);
        registrations.swap(m_registrationThreads);
    }
    for (auto& t : registrations) {
        if (t.joinable()) t.join();
    }

    std::map<uint8_t, Vehicle> vehicles;
    std::vector<mavsdk::Mavsdk::ConnectionHandle> connections;
    {
        std::lock_guard lock(m_linkMutex);
        vehicles.swap(m_vehicles);
        connections.swap(m_connectionHandles);
        m_expectedSysIds.clear();
    }
    for (auto& vehicle : vehicles | std::views::values) {
        m_unsubscribeMavlink(vehicle);
    }
    for (const auto& handle : connections) {
        m_mavsdk.remove_connection(handle);
    }

    {
        std::lock_guard lock(m_statesMutex);
        m_aggregators.clear();
        for (auto& health : m_uavHealths | std::views::values) health.isConnected = false;
    }
    m_onStatusUpdate();

    LOG_INFO("CommunicationManager stopped");
}

void CommunicationManager::setTelemetryCallback(std::function<void(const std::map<uint8_t, uavStates>&)> cb) {
    m_telemetryCallback = std::move(cb);
}

void CommunicationManager::setStatusCallback(std::function<void(const std::map<uint8_t, uavHealth>&)> cb) {
    m_statusCallback = std::move(cb);
}

void CommunicationManager::setEkfOriginCallback(std::function<void(uint8_t, double, double, double)> cb) {
    m_ekfOriginCallback = std::move(cb);
}

std::optional<double> CommunicationManager::telemetryAge(const uint8_t sysId) {
    const auto aggregator = m_aggregatorOf(sysId);
    if (!aggregator) return std::nullopt;
    const auto last = aggregator->lastStateTime();
    if (!last) return std::nullopt;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - *last).count();
}

void CommunicationManager::connectAll(const std::string& baseIp, const uint16_t basePort, const int numUavs, const int increment, const int discoveryTimeoutMs) {
    LOG_INFO("Connecting to UAV(s)...");
    for (int i = 0; i < numUavs; ++i) {
        const uint16_t port = basePort + i * increment;
        addLink("tcpout://" + baseIp + ":" + std::to_string(port));
    }
    // Systems MAVSDK already knows (a connect after stop()) get no new-system event.
    for (const auto& system : m_mavsdk.systems()) m_watchSystem(system);

    LOG_INFO("All UAV links opened -- vehicles will register automatically as they connect.");
    m_waitAndSummarize(numUavs, discoveryTimeoutMs);
}

void CommunicationManager::connectAll(const std::vector<pixhawkEndpointConfig>& endpoints, const int discoveryTimeoutMs) {
    LOG_INFO("Connecting to UAV(s) via explicit endpoints...");
    {
        std::lock_guard lock(m_linkMutex);
        for (const auto& endpoint : endpoints) m_expectedSysIds.push_back(endpoint.id);
    }
    for (const auto& [id, ip, port] : endpoints) {
        const std::string uri = "udpin://0.0.0.0:" + std::to_string(port);
        LOG_INFO("Adding link for UAV " + std::to_string(id) + " -> " + uri);
        addLink(uri);
    }
    for (const auto& system : m_mavsdk.systems()) m_watchSystem(system);

    LOG_INFO("All UAV links opened -- vehicles will register automatically as they connect.");
    m_waitAndSummarize(static_cast<int>(endpoints.size()), discoveryTimeoutMs);
}

void CommunicationManager::armAll(const bool force) {
    const auto passthroughs = m_passthroughs();
    if (passthroughs.empty()) {
        LOG_WARNING("Arm command ignored: no UAV connected");
        return;
    }

    for (const auto& [sysId, passthrough] : passthroughs) {
        LOG_INFO(std::string(force ? "Force arming" : "Arming") + " UAV sysId = " + std::to_string(sysId) + " ...");

        mavsdk::MavlinkPassthrough::CommandLong command{};
        command.command = MAV_CMD_COMPONENT_ARM_DISARM;
        command.param1 = 1;
        command.param2 = force ? 2989.0f : 0.0f; // 2989: ArduPilot force arm, skips the pre-arm checks
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
    const auto passthrough = m_passthroughOf(sysId);
    if (!passthrough) {
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
    command.target_sysid = passthrough->get_target_sysid();
    command.target_compid = MAV_COMP_ID_AUTOPILOT1;

    const auto result = passthrough->send_command_long(command);
    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        LOG_WARNING("Failed to set mode for sysId = " + std::to_string(sysId) + ", mode = " + mode + ", result = " + std::to_string(static_cast<int>(result)));
        return;
    }
    LOG_INFO("Mode command sent successfully to sysId = " + std::to_string(sysId) + ": " + mode);
}

void CommunicationManager::setModeAll(const std::string& mode) {
    const auto passthroughs = m_passthroughs();
    if (passthroughs.empty()) {
        LOG_WARNING("Set mode ignored: no UAV connected");
        return;
    }

    LOG_INFO("Setting mode '" + mode + "' for " + std::to_string(passthroughs.size()) + " UAV(s) ...");
    for (const auto& sysId : passthroughs | std::views::keys) {
        setMode(sysId, mode);
    }
}

void CommunicationManager::fetchParam(const int sysId) {
    std::shared_ptr<mavsdk::Param> param;
    {
        std::lock_guard lock(m_linkMutex);
        const auto it = m_vehicles.find(static_cast<uint8_t>(sysId));
        if (it != m_vehicles.end()) param = it->second.param;
    }
    if (!param) {
        LOG_INFO("UAV not connected");
        return;
    }

    auto [int_params, float_params, custom_params] = param->get_all_params();

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

bool CommunicationManager::addLink(const std::string& connection) {
    LOG_INFO("Connection: " + connection);
    const auto [connectionResult, connectionHandle] = m_mavsdk.add_any_connection_with_handle(connection);

    if (connectionResult != mavsdk::ConnectionResult::Success) {
        LOG_ERROR("Failed to add link: " + connection);
        return false;
    }

    std::lock_guard lock(m_linkMutex);
    m_connectionHandles.push_back(connectionHandle);
    return true;
}

void CommunicationManager::listLinks() {
    std::lock_guard lock(m_linkMutex);
    if (m_vehicles.empty()) {
        LOG_WARNING("No links connected");
        return;
    }
    for (const auto& sysId : m_vehicles | std::views::keys) {
        LOG_INFO("Connected to sysId = " + std::to_string(sysId));
    }
}

void CommunicationManager::setHomeToCurrentPosition() {
    for (const auto& [sysId, passthrough] : m_passthroughs()) {
        mavsdk::MavlinkPassthrough::CommandLong command{};
        command.command = MAV_CMD_DO_SET_HOME;
        command.param1 = 1; // current position
        command.target_sysid = passthrough->get_target_sysid();
        command.target_compid = MAV_COMP_ID_AUTOPILOT1;

        const auto result = passthrough->send_command_long(command);
        if (result == mavsdk::MavlinkPassthrough::Result::Success) {
            LOG_INFO("sysId " + std::to_string(sysId) + ": home set to the current position");
        } else {
            LOG_WARNING("sysId " + std::to_string(sysId) + ": failed to set home, result = " + std::to_string(static_cast<int>(result)));
        }
    }
}

void CommunicationManager::setUavCommands(const std::map<uint8_t, uavCommandsFlags>& uavCommands) {
    m_sendAttitudeTarget(uavCommands);
}

void CommunicationManager::sendRtcmData(const std::vector<uint8_t>& data) {
    std::vector<std::pair<uint8_t, std::shared_ptr<mavsdk::Rtk>>> rtks;
    {
        std::lock_guard lock(m_linkMutex);
        for (const auto& [sysId, vehicle] : m_vehicles) rtks.emplace_back(sysId, vehicle.rtk);
    }
    if (rtks.empty()) {
        LOG_DEBUG("sendRtcmData called but no UAV is connected yet");
        return;
    }

    // mavsdk::base64_encode() takes a non-const reference.
    std::vector<uint8_t> encodableData = data;
    mavsdk::Rtk::RtcmData rtcmData;
    rtcmData.data_base64 = mavsdk::base64_encode(encodableData);

    for (const auto& [sysId, rtk] : rtks) {
        if (rtk->send_rtcm_data(rtcmData) != mavsdk::Rtk::Result::Success) {
            LOG_WARNING("Failed to send RTCM data to sysId = " + std::to_string(sysId));
        }
    }
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

std::shared_ptr<mavsdk::MavlinkPassthrough> CommunicationManager::m_passthroughOf(const uint8_t sysId) {
    std::lock_guard lock(m_linkMutex);
    const auto it = m_vehicles.find(sysId);
    return it == m_vehicles.end() ? nullptr : it->second.passthrough;
}

std::vector<std::pair<uint8_t, std::shared_ptr<mavsdk::MavlinkPassthrough>>> CommunicationManager::m_passthroughs() {
    std::lock_guard lock(m_linkMutex);
    std::vector<std::pair<uint8_t, std::shared_ptr<mavsdk::MavlinkPassthrough>>> out;
    for (const auto& [sysId, vehicle] : m_vehicles) out.emplace_back(sysId, vehicle.passthrough);
    return out;
}

std::shared_ptr<StatesAggregator> CommunicationManager::m_aggregatorOf(const uint8_t sysId) {
    std::lock_guard lock(m_statesMutex);
    const auto it = m_aggregators.find(sysId);
    return it == m_aggregators.end() ? nullptr : it->second;
}

void CommunicationManager::m_watchSystem(const std::shared_ptr<mavsdk::System>& system) {
    const uint8_t sysId = system->get_system_id();
    {
        std::lock_guard lock(m_linkMutex);
        if (!m_watchers.try_emplace(sysId, Watcher{system, std::nullopt}).second) return; // already watched
    }

    // Subscribed outside the lock: the callback takes it.
    const auto handle = system->subscribe_is_connected([this, system](const bool connected) {
        m_onConnectionChanged(system, connected);
    });
    {
        std::lock_guard lock(m_linkMutex);
        const auto it = m_watchers.find(sysId);
        if (it != m_watchers.end()) it->second.handle = handle;
    }

    // Already connected when first seen (common over TCP): no change event will come.
    if (system->is_connected()) m_onConnectionChanged(system, true);
}

void CommunicationManager::m_onConnectionChanged(const std::shared_ptr<mavsdk::System>& system, const bool connected) {
    const uint8_t sysId = system->get_system_id();
    bool registered;
    {
        std::lock_guard lock(m_linkMutex);
        registered = m_vehicles.contains(sysId);
        if (connected && !registered && system->has_autopilot()) {
            // Plugin construction off the MAVSDK callback thread.
            m_registrationThreads.emplace_back([this, system]() { m_registerSystem(system); });
            return;
        }
    }
    if (!registered) return;

    m_updateHealth(sysId, [connected](uavHealth& h) { h.isConnected = connected; });
    if (connected) {
        // A reconnection after a reboot: the vehicle lost every requested rate.
        LOG_INFO("UAV sysId = " + std::to_string(sysId) + " reconnected");
        m_requestStatusRates(sysId);
        m_requestControlState(sysId);
    } else {
        LOG_WARNING("UAV sysId = " + std::to_string(sysId) + " lost connection");
    }
}

void CommunicationManager::m_registerSystem(const std::shared_ptr<mavsdk::System>& system) {
    const uint8_t sysId = system->get_system_id();

    Vehicle vehicle;
    vehicle.system      = system;
    vehicle.telemetry   = std::make_shared<mavsdk::Telemetry>(system);
    vehicle.param       = std::make_shared<mavsdk::Param>(system);
    vehicle.passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);
    vehicle.rtk         = std::make_shared<mavsdk::Rtk>(system);

    {
        std::lock_guard lock(m_linkMutex);
        if (m_vehicles.contains(sysId)) return;
        if (!m_expectedSysIds.empty() && std::ranges::find(m_expectedSysIds, sysId) == m_expectedSysIds.end()) {
            LOG_WARNING("UAV connected with sysId = " + std::to_string(sysId)
                        + ", which isn't one of the ids configured under Pixhawk.endpoints -- check for "
                        "a MAVLink SYSID collision between vehicles (e.g. two boards both left on the "
                        "default SYSID_THISMAV) or unexpected traffic on the listening port.");
        }
        m_vehicles.emplace(sysId, std::move(vehicle));
    }
    {
        std::lock_guard lock(m_statesMutex);
        m_aggregators[sysId] = std::make_shared<StatesAggregator>();
        m_uavHealths[sysId].isConnected = true;
    }

    m_subscribeMavlink(sysId);
    m_onStatusUpdate();
    LOG_INFO("Connected: sysID = " + std::to_string(sysId));
}

void CommunicationManager::m_waitAndSummarize(const int expectedCount, const int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    int connected;
    do {
        {
            std::lock_guard lock(m_linkMutex);
            connected = static_cast<int>(m_vehicles.size());
        }
        if (connected >= expectedCount) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::steady_clock::now() < deadline);

    if (connected < expectedCount) {
        LOG_WARNING(std::to_string(connected) + "/" + std::to_string(expectedCount)
                    + " UAV(s) connected within " + std::to_string(timeoutMs) + "ms. Still listening in "
                    "the background -- any remaining vehicle registers automatically the moment it sends "
                    "its first heartbeat, no need to re-run 'connect'.");
    } else {
        LOG_INFO("All " + std::to_string(connected) + " UAV(s) connected.");
    }
}

// ---------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------

void CommunicationManager::m_subscribeMavlink(const uint8_t sysId) {
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
    {
        std::lock_guard lock(m_linkMutex);
        const auto it = m_vehicles.find(sysId);
        if (it == m_vehicles.end()) return;
        telemetry = it->second.telemetry;
        passthrough = it->second.passthrough;
    }

    subscriptionHandles h;
    h.healthHandle = telemetry->subscribe_health([this, sysId](const mavsdk::Telemetry::Health& health) {
        m_updateHealth(sysId, [&](uavHealth& u) { u.health = health; });
    });
    h.healthAllOkHandle = telemetry->subscribe_health_all_ok([this, sysId](const bool ok) {
        m_updateHealth(sysId, [&](uavHealth& u) { u.isHealthy = ok; });
    });
    h.armedHandle = telemetry->subscribe_armed([this, sysId](const bool armed) {
        m_updateHealth(sysId, [&](uavHealth& u) { u.isArmed = armed; });
    });
    h.batteryHandle = telemetry->subscribe_battery([this, sysId](const mavsdk::Telemetry::Battery& battery) {
        m_updateHealth(sysId, [&](uavHealth& u) {
            u.batteryRemainingPercent = battery.remaining_percent;
            u.batteryVoltageVolt = battery.voltage_v;
        });
    });
    h.gpsInfoHandle = telemetry->subscribe_gps_info([this, sysId](const mavsdk::Telemetry::GpsInfo& gps) {
        m_updateHealth(sysId, [&](uavHealth& u) {
            u.gpsNumSatellites = gps.num_satellites;
            u.gpsFixType = gps.fix_type;
        });
    });
    h.rcStatusHandle = telemetry->subscribe_rc_status([this, sysId](const mavsdk::Telemetry::RcStatus& rc) {
        m_updateHealth(sysId, [&](uavHealth& u) {
            u.rcAvailable = rc.is_available;
            u.rcSignalPercent = rc.signal_strength_percent;
        });
    });
    h.homeHandle = telemetry->subscribe_home([](const mavsdk::Telemetry::Position& home) {
        LOG_INFO("Home position: Lat = " + std::to_string(home.latitude_deg) + ", Lon = " + std::to_string(home.longitude_deg) + ", Alt = " + std::to_string(home.absolute_altitude_m) + "m");
    });

    // GLOBAL_POSITION_INT: GCS origin, frame-offset check, dashboard. Also
    // the cadence of the GPS_GLOBAL_ORIGIN and control-state requests.
    h.positionHandle = telemetry->subscribe_position([this, sysId](const mavsdk::Telemetry::Position& position) {
        if (const auto aggregator = m_aggregatorOf(sysId)) {
            aggregator->updateGlobalPosition(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m);
        }
        m_requestGpsGlobalOrigin(sysId);
        m_requestControlState(sysId);
        m_telemetryChanged();
    });

    const auto raw = [&](const uint16_t id, void (CommunicationManager::*handler)(uint8_t, const mavlink_message_t&)) {
        h.messageHandles.emplace_back(id, passthrough->subscribe_message(id, [this, sysId, handler](const mavlink_message_t& m) {
            (this->*handler)(sysId, m);
        }));
    };
    raw(MAVLINK_MSG_ID_HEARTBEAT, &CommunicationManager::m_onHeartbeat);
    raw(MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE, &CommunicationManager::m_onControlState);
    raw(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, &CommunicationManager::m_onGpsGlobalOrigin);

    {
        std::lock_guard lock(m_linkMutex);
        const auto it = m_vehicles.find(sysId);
        if (it != m_vehicles.end()) it->second.handles = std::move(h);
    }

    m_requestControlState(sysId);
    m_requestStatusRates(sysId);
}

void CommunicationManager::m_unsubscribeMavlink(Vehicle& vehicle) {
    auto& t = *vehicle.telemetry;
    const auto& h = vehicle.handles;
    t.unsubscribe_health(h.healthHandle);
    t.unsubscribe_health_all_ok(h.healthAllOkHandle);
    t.unsubscribe_armed(h.armedHandle);
    t.unsubscribe_battery(h.batteryHandle);
    t.unsubscribe_gps_info(h.gpsInfoHandle);
    t.unsubscribe_rc_status(h.rcStatusHandle);
    t.unsubscribe_home(h.homeHandle);
    t.unsubscribe_position(h.positionHandle);
    for (const auto& [id, handle] : h.messageHandles) {
        vehicle.passthrough->unsubscribe_message(id, handle);
    }
    vehicle.handles = {};
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

void CommunicationManager::m_telemetryChanged() {
    if (m_config.telemetry_publish_hz <= 0.0) {
        m_onTelemetryUpdate();
    } else {
        m_snapshotDirty.store(true);
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

void CommunicationManager::m_updateHealth(const uint8_t sysId, const std::function<void(uavHealth&)>& update) {
    {
        std::lock_guard lock(m_statesMutex);
        update(m_uavHealths[sysId]);
    }
    m_onStatusUpdate();
}

void CommunicationManager::m_onHeartbeat(const uint8_t sysId, const mavlink_message_t& message) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    m_updateHealth(sysId, [&](uavHealth& h) {
        h.customMode = heartbeat.custom_mode;
        h.customModeReceived = true;
    });
}

void CommunicationManager::m_onControlState(const uint8_t sysId, const mavlink_message_t& message) {
    mavlink_control_system_state_t state;
    mavlink_msg_control_system_state_decode(&message, &state);
    const float pos[3] = {state.x_pos, state.y_pos, state.z_pos};
    const float vel[3] = {state.x_vel, state.y_vel, state.z_vel};

    const auto aggregator = m_aggregatorOf(sysId);
    if (!aggregator || !aggregator->updateControlState(state.time_usec, pos, vel, state.airspeed, state.q)) return;

    {
        std::lock_guard lock(m_requestMutex);
        auto& w = m_stateRate[sysId];
        const auto now = std::chrono::steady_clock::now();
        if (w.count++ == 0 && !w.reported) w.start = now;
        const double elapsed = std::chrono::duration<double>(now - w.start).count();
        if (elapsed >= 5.0) {
            const double rate = (w.count - 1) / elapsed;
            if (!w.reported || rate < 0.7 * m_config.stateRateHz) {
                LOG_INFO("sysId " + std::to_string(sysId) + ": CONTROL_SYSTEM_STATE at " + std::to_string(rate)
                         + " Hz (requested " + std::to_string(m_config.stateRateHz) + " Hz)");
            }
            w = {.start = now, .count = 1, .reported = true};
        }
    }

    m_telemetryChanged();
}

void CommunicationManager::m_onGpsGlobalOrigin(const uint8_t sysId, const mavlink_message_t& message) {
    mavlink_gps_global_origin_t origin;
    mavlink_msg_gps_global_origin_decode(&message, &origin);
    {
        std::lock_guard lock(m_requestMutex);
        m_ekfOriginKnown[sysId] = true;
    }
    if (m_ekfOriginCallback) {
        m_ekfOriginCallback(sysId, origin.latitude * 1e-7, origin.longitude * 1e-7, origin.altitude * 1e-3);
    }
}

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

void CommunicationManager::m_requestControlState(const uint8_t sysId) {
    const auto aggregator = m_aggregatorOf(sysId);
    if (!aggregator) return;
    const auto now = std::chrono::steady_clock::now();
    const auto last = aggregator->lastStateTime();
    if (last && now - *last < std::chrono::seconds(1)) return;
    {
        std::lock_guard lock(m_requestMutex);
        const auto requested = m_stateRequestedAt.find(sysId);
        if (requested != m_stateRequestedAt.end() && now - requested->second < std::chrono::seconds(2)) return;
        if (requested != m_stateRequestedAt.end()) {
            LOG_WARNING("sysId " + std::to_string(sysId) + ": no CONTROL_SYSTEM_STATE (GRS firmware required), requested again");
        }
        m_stateRequestedAt[sysId] = now;
    }
    m_sendCommandLong(sysId, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE,
                      static_cast<float>(1e6 / m_config.stateRateHz));
}

void CommunicationManager::m_requestGpsGlobalOrigin(const uint8_t sysId) {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard lock(m_requestMutex);
        const auto period = std::chrono::seconds(m_ekfOriginKnown[sysId] ? 10 : 2);
        const auto last = m_ekfOriginRequestedAt.find(sysId);
        if (last != m_ekfOriginRequestedAt.end() && now - last->second < period) return;
        m_ekfOriginRequestedAt[sysId] = now;
    }
    m_sendCommandLong(sysId, MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0.0f);
}

void CommunicationManager::m_requestStatusRates(const uint8_t sysId) {
    // Dashboard and frame offset only. Set every SRx_* of this port to 0 on the
    // vehicle: then only what is requested here is sent.
    m_sendCommandLong(sysId, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1e6f / 5.0f);
    for (const uint32_t id : {MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_BATTERY_STATUS,
                              MAVLINK_MSG_ID_RC_CHANNELS}) {
        m_sendCommandLong(sysId, MAV_CMD_SET_MESSAGE_INTERVAL, static_cast<float>(id), 1e6f);
    }
}

void CommunicationManager::m_sendCommandLong(const uint8_t sysId, const uint16_t command, const float param1, const float param2) {
    const auto passthrough = m_passthroughOf(sysId);
    if (!passthrough) return;
    passthrough->queue_message([passthrough, command, param1, param2](const MavlinkAddress address, const uint8_t channel) {
        mavlink_message_t message;
        mavlink_msg_command_long_pack_chan(address.system_id, address.component_id, channel, &message,
                                           passthrough->get_target_sysid(), passthrough->get_target_compid(),
                                           command, 0, param1, param2, 0, 0, 0, 0, 0);
        return message;
    });
}

void CommunicationManager::m_sendAttitudeTarget(const std::map<uint8_t, uavCommandsFlags>& commands) {
    for (const auto& [sysId, cmd] : commands) {
        const auto passthrough = m_passthroughOf(sysId);
        if (!passthrough) {
            LOG_WARNING("Skipping sysId " + std::to_string(sysId) + ": not connected");
            continue;
        }

        const auto targetSysId  = passthrough->get_target_sysid();
        const auto targetCompId = passthrough->get_target_compid();
        const auto result = passthrough->queue_message([cmd, targetSysId, targetCompId](const MavlinkAddress address, const uint8_t channel) {
            return MavlinkMessageBuilder::buildSetAttitudeTarget(address, channel, targetSysId, targetCompId, cmd);
        });
        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            LOG_WARNING("Failed to queue SET_ATTITUDE_TARGET for sysId " + std::to_string(sysId));
        }
    }
}
