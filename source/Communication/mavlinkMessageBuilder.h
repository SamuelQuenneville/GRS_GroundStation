/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef MAVLINKMESSAGEBUILDER_H
#define MAVLINKMESSAGEBUILDER_H

#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <cmath>
#include <array>
#include <chrono>

#include "Definitions/communicationStructures.h"


class MavlinkMessageBuilder {
public:
    static mavlink_message_t buildSetAttitudeTarget(const MavlinkAddress& address, uint8_t channel, uint8_t targetSysid, uint8_t targetCompid, const uavCommandsFlags& target);

private:
    static void m_eulerToQuaternion(double rollDeg, double pitchDeg, double yawDeg, float q[4]);
};



#endif //MAVLINKMESSAGEBUILDER_H
