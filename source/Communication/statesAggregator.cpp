/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "statesAggregator.h"

bool StatesAggregator::updateControlState(const uint64_t timeUsec, const float pos[3], const float vel[3], const float airspeed, const float q[4]) {
    // q = [w x y z], body to NED (ZYX Euler).
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    constexpr float radToDeg = 180.0f / static_cast<float>(M_PI);
    const float roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * radToDeg;
    const float pitch = std::asin(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f)) * radToDeg;
    const float yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * radToDeg;

    std::lock_guard lock(m_mutex);
    // Duplicate or reordered datagram. A jump back of more than 1 s is a reboot.
    if (m_lastStateTime && timeUsec <= m_lastTimeUsec && m_lastTimeUsec - timeUsec < 1'000'000) {
        return false;
    }

    m_lastTimeUsec = timeUsec;
    m_lastStateTime = std::chrono::steady_clock::now();

    m_state.northMeter = pos[0];
    m_state.eastMeter  = pos[1];
    m_state.downMeter  = pos[2];
    m_state.northMeterSecond = vel[0];
    m_state.eastMeterSecond  = vel[1];
    m_state.downMeterSecond  = vel[2];
    m_state.airspeedMeterSecond = airspeed;
    m_state.rollDegree  = roll;
    m_state.pitchDegree = pitch;
    m_state.yawDegree   = yaw;
    return true;
}

void StatesAggregator::updateGlobalPosition(const double lat, const double lon, const double alt) {
    std::lock_guard lock(m_mutex);
    m_state.latitudeDegree    = lat;
    m_state.longitudeDegree   = lon;
    m_state.altitudeAmslMeter = alt;
}

uavStates StatesAggregator::getSnapshot() const {
    std::lock_guard lock(m_mutex);
    return m_state;
}

std::optional<std::chrono::steady_clock::time_point> StatesAggregator::lastStateTime() const {
    std::lock_guard lock(m_mutex);
    return m_lastStateTime;
}