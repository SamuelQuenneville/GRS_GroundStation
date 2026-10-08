/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef STATEVECTOR_H
#define STATEVECTOR_H

#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "Definitions/vehicleStructures.h"

// Joint state vector shared by the NMPC and the NMHE, same layout as the
// MATLAB models (grs*DynamicAugmented.m):
//
//   [ UAV1 (8) | UAV2 (8) | ... | payload (6, only if the model has one) ]
//   UAV block:     pN pE pD vN vE vD roll pitch   [m, m/s, rad]
//   payload block: pN pE pD vN vE vD              [m, m/s]
//
// UAV i is telemetry sysId i. The payload is the highest sysId above
// numUavs (same convention as ControlInterface::getPayloadGpsFix()).
namespace grs::control {

inline constexpr int kUavBlockSize = 8;
inline constexpr int kPayloadBlockSize = 6;

struct StateLayout {
    int numUavs = 1;
    bool hasPayload = false;

    [[nodiscard]] size_t size() const {
        return static_cast<size_t>(kUavBlockSize) * numUavs + (hasPayload ? kPayloadBlockSize : 0);
    }
    [[nodiscard]] static size_t uavOffset(const int uavIndex) { return static_cast<size_t>(kUavBlockSize) * uavIndex; }
    [[nodiscard]] size_t payloadOffset() const { return static_cast<size_t>(kUavBlockSize) * numUavs; }
};

// Which blocks fillStateVector() wrote this call.
struct StateFill {
    std::vector<bool> uav;  // one entry per UAV, index = sysId - 1
    bool payload = false;   // always false when the layout has no payload

    [[nodiscard]] bool complete(const StateLayout& layout) const;
};

// Writes each vehicle's telemetry into its own block of `out` (which must
// already have layout.size() entries). Blocks with no telemetry this call
// are left untouched, so the caller decides what a missing vehicle holds
// (its last value, zeros, a reference...).
StateFill fillStateVector(const std::map<uint8_t, uavStates>& states, const StateLayout& layout, std::vector<double>& out);

} // namespace grs::control

#endif //STATEVECTOR_H
