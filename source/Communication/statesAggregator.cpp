/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "statesAggregator.h"

#include <iostream>

StatesAggregator::StatesAggregator()
{
    const auto now = std::chrono::steady_clock::now();

    m_rates.lastAttitude = now;
    m_rates.lastPosition = now;
    m_rates.lastVelocity = now;
    m_rates.lastAirspeed = now;
    m_rates.lastGlobalPosition = now;
}

void StatesAggregator::updateAttitude(const float roll, const float pitch, const float yaw) {
    std::lock_guard lock(m_mutex);
    m_state.rollDegree  = roll;
    m_state.pitchDegree = pitch;
    m_state.yawDegree   = yaw;

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - m_rates.lastAttitude;
    m_rates.rateAttitude = 1.0 / dt.count();
    m_rates.lastAttitude = now;
}

void StatesAggregator::updatePosition(const float n, const float e, const float d) {
    std::lock_guard lock(m_mutex);
    m_state.northMeter = n;
    m_state.eastMeter  = e;
    m_state.downMeter  = d;

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - m_rates.lastPosition;
    m_rates.ratePosition = 1.0 / dt.count();
    m_rates.lastPosition = now;

}

void StatesAggregator::updateVelocity(const float vn, const float ve, const float vd) {
    std::lock_guard lock(m_mutex);
    m_state.northMeterSecond = vn;
    m_state.eastMeterSecond  = ve;
    m_state.downMeterSecond  = vd;

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - m_rates.lastVelocity;
    m_rates.rateVelocity = 1.0 / dt.count();
    m_rates.lastVelocity = now;
}

void StatesAggregator::updateAirspeed(const float airspeed) {
    std::lock_guard lock(m_mutex);
    m_state.airspeedMeterSecond = airspeed;

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - m_rates.lastAirspeed;
    m_rates.rateAirspeed = 1.0 / dt.count();
    m_rates.lastAirspeed = now;
}

void StatesAggregator::updateGlobalPosition(const double lat, const double lon, const double alt) {
    std::lock_guard lock(m_mutex);
    m_state.latitudeDegree    = lat;
    m_state.longitudeDegree   = lon;
    m_state.altitudeAmslMeter = alt;

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - m_rates.lastGlobalPosition;
    m_rates.rateGlobalPosition = 1.0 / dt.count();
    m_rates.lastGlobalPosition = now;
}

uavStates StatesAggregator::getSnapshot() const {
    std::lock_guard lock(m_mutex);
    return m_state;
}

aggregatorRates StatesAggregator::getRates() const {
    std::lock_guard lock(m_mutex);
    return m_rates;
}