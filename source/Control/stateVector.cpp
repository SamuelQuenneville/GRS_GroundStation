/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "stateVector.h"

#include <algorithm>
#include <cassert>

#include "Mathematics/math.h"

namespace grs::control {

bool StateFill::complete(const StateLayout& layout) const {
    const bool allUavs = std::ranges::all_of(uav, [](const bool b) { return b; });
    return allUavs && (!layout.hasPayload || payload);
}

StateFill fillStateVector(const std::map<uint8_t, uavStates>& states, const StateLayout& layout, std::vector<double>& out) {
    assert(out.size() == layout.size());

    StateFill fill;
    fill.uav.assign(static_cast<size_t>(layout.numUavs), false);

    const uavStates* payload = nullptr;
    for (const auto& [sysId, s] : states) {
        if (sysId >= 1 && sysId <= layout.numUavs) {
            const size_t o = layout.uavOffset(sysId - 1);
            out[o + 0] = s.northMeter;
            out[o + 1] = s.eastMeter;
            out[o + 2] = s.downMeter;
            out[o + 3] = s.northMeterSecond;
            out[o + 4] = s.eastMeterSecond;
            out[o + 5] = s.downMeterSecond;
            out[o + 6] = grs::degToRad(s.rollDegree);
            out[o + 7] = grs::degToRad(s.pitchDegree);
            fill.uav[sysId - 1] = true;
        } else if (sysId > layout.numUavs) {
            payload = &s; // std::map is ordered: the highest sysId is kept
        }
    }

    if (layout.hasPayload && payload) {
        const size_t o = layout.payloadOffset();
        out[o + 0] = payload->northMeter;
        out[o + 1] = payload->eastMeter;
        out[o + 2] = payload->downMeter;
        out[o + 3] = payload->northMeterSecond;
        out[o + 4] = payload->eastMeterSecond;
        out[o + 5] = payload->downMeterSecond;
        fill.payload = true;
    }
    return fill;
}

} // namespace grs::control
