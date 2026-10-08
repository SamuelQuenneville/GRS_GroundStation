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

#include <chrono>
#include <mutex>
#include <optional>

#include "Definitions/communicationStructures.h"

// Latest state of one vehicle, merged from CONTROL_SYSTEM_STATE (controller
// state, GRS convention in grsMavlinkConventions.h) and GLOBAL_POSITION_INT.
class StatesAggregator {
public:
    // Returns false (state unchanged) for a sample not newer than the last one.
    bool updateControlState(uint64_t timeUsec, const float pos[3], const float vel[3], float airspeed,
                            const float q[4]);
    void updateGlobalPosition(double lat, double lon, double alt);

    uavStates getSnapshot() const;
    // Arrival time of the last control state; nullopt before the first.
    std::optional<std::chrono::steady_clock::time_point> lastStateTime() const;

private:
    mutable std::mutex m_mutex;
    uavStates m_state{};
    uint64_t m_lastTimeUsec = 0;
    std::optional<std::chrono::steady_clock::time_point> m_lastStateTime;
};

#endif //STATESAGGREGATOR_H
