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
    mavsdk::log::subscribe([](const mavsdk::log::Level level, const std::string& message, const std::string& file, int line) {
        // Returning true from the callback disables printing the message to stdout
        return level < mavsdk::log::Level::Warn;
    });
}

CommunicationManager::~CommunicationManager() {

    for (const auto [sysId, handle]: m_connectionHandles) {
        m_unsubscribeMavlink(sysId);
        m_mavsdk.remove_connection(handle);
    }

    m_messageHandles.clear();
    m_connectionHandles.clear();
}


bool CommunicationManager::addLink(const std::string& connection) {
    const auto [connectionResult, connectionHandle] = m_mavsdk.add_any_connection_with_handle(connection);

    if (connectionResult != mavsdk::ConnectionResult::Success) {
        LOG_ERROR("Failed to add link: " + connection);
        return false;
    }

    m_numberOfUavs += 1;
    while (m_mavsdk.systems().size() < m_numberOfUavs) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (auto& system : m_mavsdk.systems()) {
        if (!system->is_connected()) {
            continue;
        }

        uint8_t sysId = system->get_system_id();

        std::lock_guard<std::mutex> lock(m_linkMutex);
        if (!m_links.contains(sysId)) {
            m_links[sysId] = system;
            m_connectionHandles[sysId] = connectionHandle;
            m_telemetry[sysId] = std::make_shared<mavsdk::Telemetry>(system);
            m_action[sysId] = std::make_shared<mavsdk::Action>(system);

            m_subscribeMavlink(sysId);

            LOG_INFO("Connected: sysID = " + std::to_string(sysId));
        }
    }
    return true;
}

void CommunicationManager::listLinks() {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    if (m_links.empty()) {
        LOG_WARNING("No links connected");
    } else {
        for (const auto& sysId : m_links | std::views::keys) {
            LOG_INFO("Connected to sysId = " + std::to_string(sysId));
        }
    }
}

void CommunicationManager::m_subscribeMavlink(const uint8_t sysId) {
    const auto telemetryIterator = m_telemetry.find(sysId);

    if (telemetryIterator == m_telemetry.end()) {
        LOG_WARNING("Telemetry not found for sysId = " + std::to_string(sysId));
        return;
    }

    const auto telemetry = telemetryIterator->second;
    subscriptionHandles handles;

    // HEARTBEAT
    m_subscribeHealth(telemetry, sysId, handles);
    m_subscribeHealthAllOk(telemetry, sysId, handles);
    m_subscribeArmed(telemetry, sysId, handles);
    m_subscribeFlightMode(telemetry, sysId, handles);

    // ATTITUDE
    m_subscribeAttitude(telemetry, sysId, handles);
    m_subscribeAngularVelocity(telemetry, sysId, handles);

    // LOCAL_POSITION_NED
    m_subscribePositionVelocity(telemetry, sysId, handles);

    // GLOBAL_POSITION_INT
    m_subscribePosition(telemetry, sysId, handles);
    m_subscribeVelocityNed(telemetry, sysId, handles);
    m_subscribeHeading(telemetry, sysId, handles);

    // VFR_HUD
    m_subscribeFixedwingMetrics(telemetry, sysId, handles);

    m_messageHandles[sysId] = handles;
}

void CommunicationManager::m_unsubscribeMavlink(const uint8_t sysId) {
    const auto handleIterator = m_messageHandles.find(sysId);
    const auto telemetryIterator = m_telemetry.find(sysId);

    if (telemetryIterator == m_telemetry.end()) {
        LOG_WARNING("Telemetry not found for sysId = " + std::to_string(sysId));
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

    // ATTITUDE
    telemetry->unsubscribe_attitude_euler(handles.attitudeHandle);
    telemetry->unsubscribe_attitude_angular_velocity_body(handles.angularVelocityHandle);

    // LOCAL_POSITION_NED
    telemetry->unsubscribe_position_velocity_ned(handles.positionVelocityNedHandle);

    // GLOBAL_POSITION_INT
    telemetry->unsubscribe_position(handles.positionHandle);
    telemetry->unsubscribe_velocity_ned(handles.velocityNedHandle);
    telemetry->unsubscribe_heading(handles.headingHandle);

    // VFR_HUD
    telemetry->unsubscribe_fixedwing_metrics(handles.fixedwingMetricsHandle);

    // Remove from map
    m_messageHandles.erase(sysId);
}

void CommunicationManager::m_subscribeHealth(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.healthHandle = telemetry->subscribe_health([this, sysId](const mavsdk::Telemetry::Health& health) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO health callback
    });
}

void CommunicationManager::m_subscribeHealthAllOk(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.healthAllOkHandle = telemetry->subscribe_health_all_ok([this, sysId](const bool isHealthy) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO is healthy callback
    });
}

void CommunicationManager::m_subscribeArmed(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.armedHandle = telemetry->subscribe_armed([this, sysId](const bool isArmed) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO is armed callback
    });
}

void CommunicationManager::m_subscribeFlightMode(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.flightModeHandle = telemetry->subscribe_flight_mode([this, sysId](const mavsdk::Telemetry::FlightMode& flightMode) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

       // TODO flight mode callback
    });
}


void CommunicationManager::m_subscribeAttitude(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles) {

    handles.attitudeHandle = telemetry->subscribe_attitude_euler([this, sysId](const mavsdk::Telemetry::EulerAngle& attitude) {
        std::lock_guard<std::mutex> lock(m_linkMutex);
        m_uavStates[sysId] = {attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg};

        LOG_INFO("Attitude sysId = " + std::to_string(sysId) + " | Roll: " + std::to_string(attitude.roll_deg) + " | Pitch: " + std::to_string(attitude.pitch_deg) + " | Yaw: " + std::to_string(attitude.yaw_deg));
    });
}

void CommunicationManager::m_subscribeAngularVelocity(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.angularVelocityHandle = telemetry->subscribe_attitude_angular_velocity_body([this, sysId](const mavsdk::Telemetry::AngularVelocityBody& angularVelocity) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO angular velocity callback
        LOG_INFO("Receive angular velocity sysId = " + std::to_string(sysId));
    });
}

void CommunicationManager::m_subscribePositionVelocity(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.positionVelocityNedHandle = telemetry->subscribe_position_velocity_ned([this, sysId](const mavsdk::Telemetry::PositionVelocityNed& positionVelocityNed) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

       // TODO position velocity callback
    });
}

void CommunicationManager::m_subscribePosition(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.positionHandle = telemetry->subscribe_position([this, sysId](const mavsdk::Telemetry::Position& position) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO position callback
    });
}

void CommunicationManager::m_subscribeVelocityNed(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.velocityNedHandle = telemetry->subscribe_velocity_ned([this, sysId](const mavsdk::Telemetry::VelocityNed& velocityNed) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO velocity ned callback
    });
}

void CommunicationManager::m_subscribeHeading(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.headingHandle = telemetry->subscribe_heading([this, sysId](const mavsdk::Telemetry::Heading& heading) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO heading callback
    });
}

void CommunicationManager::m_subscribeFixedwingMetrics(const std::shared_ptr<mavsdk::Telemetry> &telemetry, uint8_t sysId, subscriptionHandles &handles) {

    handles.fixedwingMetricsHandle = telemetry->subscribe_fixedwing_metrics([this, sysId](const mavsdk::Telemetry::FixedwingMetrics& fixedwingMetrics) {
        std::lock_guard<std::mutex> lock(m_linkMutex);

        // TODO fixedwing metrics callback
    });
}

std::shared_ptr<mavsdk::Telemetry> CommunicationManager::getTelemetry(const uint8_t sysId) {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    if (m_telemetry.contains(sysId)) {
        return m_telemetry[sysId];
    }
    return nullptr;
}

std::shared_ptr<mavsdk::Action> CommunicationManager::getAction(const uint8_t sysId) {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    if (m_links.contains(sysId)) {
        return m_action[sysId];
    }
    return nullptr;
}