/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "statesAggregator.h"

StatesAggregator::StatesAggregator()
{
    m_lastUpdate = std::chrono::steady_clock::now();
}

void StatesAggregator::updateAttitude(const float roll, const float pitch, const float yaw) {
    std::lock_guard lock(m_mutex);
    m_state.rollDegree  = roll;
    m_state.pitchDegree = pitch;
    m_state.yawDegree   = yaw;
    m_lastUpdate = std::chrono::steady_clock::now();
}

void StatesAggregator::updatePosition(const float n, const float e, const float d) {
    std::lock_guard lock(m_mutex);
    m_state.northMeter = n;
    m_state.eastMeter  = e;
    m_state.downMeter  = d;
    m_lastUpdate = std::chrono::steady_clock::now();
}

void StatesAggregator::updateVelocity(const float vn, const float ve, const float vd) {
    std::lock_guard lock(m_mutex);
    m_state.northMeterSecond = vn;
    m_state.eastMeterSecond  = ve;
    m_state.downMeterSecond  = vd;
    m_lastUpdate = std::chrono::steady_clock::now();
}

void StatesAggregator::updateAirspeed(const float airspeed) {
    std::lock_guard lock(m_mutex);
    m_state.airspeedMeterSecond = airspeed;
    m_lastUpdate = std::chrono::steady_clock::now();
}

void StatesAggregator::updateGlobalPosition(const double lat, const double lon, const double alt) {
    std::lock_guard lock(m_mutex);
    m_state.latitudeDegree    = lat;
    m_state.longitudeDegree   = lon;
    m_state.altitudeAmslMeter = alt;
    m_lastUpdate = std::chrono::steady_clock::now();
}

uavStates StatesAggregator::getSnapshot() const {
    std::lock_guard lock(m_mutex);
    return m_state;
}

std::chrono::steady_clock::time_point StatesAggregator::lastUpdate() const {
    std::lock_guard lock(m_mutex);
    return m_lastUpdate;
}