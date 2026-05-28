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
    aggregatorRates getRates() const;

private:
    mutable std::mutex m_mutex;
    uavStates m_state{};

    aggregatorRates m_rates{};
};

#endif //STATESAGGREGATOR_H
