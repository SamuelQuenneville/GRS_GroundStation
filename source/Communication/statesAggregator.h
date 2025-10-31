/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef STATESAGGREGATOR_H
#define STATESAGGREGATOR_H

#pragma once

#include <mutex>
#include <chrono>

#include "Definitions/communicationStructures.h"

class StatesAggregator {
public:
    StatesAggregator();

    void updateAttitude(float roll, float pitch, float yaw);
    void updatePosition(float n, float e, float d);
    void updateVelocity(float vn, float ve, float vd);
    void updateAirspeed(float airspeed);
    void updateGlobalPosition(double lat, double lon, double alt);

    uavStates getSnapshot() const;
    std::chrono::steady_clock::time_point lastUpdate() const;

private:
    mutable std::mutex m_mutex;
    uavStates m_state{};
    std::chrono::steady_clock::time_point m_lastUpdate;
};

#endif //STATESAGGREGATOR_H
