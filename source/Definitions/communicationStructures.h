/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef COMMUNICATIONSTRUCTURES_H
#define COMMUNICATIONSTRUCTURES_H

#pragma once

#include <limits>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <utility>
#include <vector>

// uavStates/uavCommands/uavCommandsFlags live in vehicleStructures.h (no
// MAVSDK dependency) so the controller core and grs_batchsim can use them
// without linking MAVSDK. Re-exported here so existing includes keep working.
#include "Definitions/vehicleStructures.h"

struct subscriptionHandles {
    mavsdk::Telemetry::HealthHandle                      healthHandle;
    mavsdk::Telemetry::HealthAllOkHandle                 healthAllOkHandle;
    mavsdk::Telemetry::ArmedHandle                       armedHandle;
    mavsdk::Telemetry::HomeHandle                        homeHandle;
    mavsdk::Telemetry::PositionHandle                    positionHandle;
    mavsdk::Telemetry::BatteryHandle                     batteryHandle;
    mavsdk::Telemetry::GpsInfoHandle                     gpsInfoHandle;
    mavsdk::Telemetry::RcStatusHandle                    rcStatusHandle;
    // Raw MAVLink subscriptions (message id, handle).
    std::vector<std::pair<uint16_t, mavsdk::MavlinkPassthrough::MessageHandle>> messageHandles;
};

// Non-numeric / low-rate UAV status, kept separate from uavStates (which is
// packed and shared verbatim with the MATLAB UDP link -- don't add fields
// there). This is what feeds the dashboard's "Status" and "System Health"
// cards; the control loop never touches it.
struct uavHealth {
    // Raw ArduPilot custom_mode, read directly off HEARTBEAT
    // (CommunicationManager::m_handleHeartbeat) rather than through
    // MAVSDK's own Telemetry::subscribe_flight_mode() -- GrsPlane is a
    // custom ArduPlane fork with its own mode numbering (gcsConfig.h's
    // FlightMode/flightModeMap), and MAVSDK's translation table is built
    // for stock ArduPilot/PX4 modes, so it can't be trusted for this
    // firmware's custom modes. See gcsConfig.h::flightModeToString() for
    // turning this back into the same name setMode()/setModeAll() accept.
    uint32_t customMode = 0;
    bool customModeReceived = false; // false until the first HEARTBEAT arrives
    mavsdk::Telemetry::Health health;
    bool isHealthy = false;
    bool isArmed = false;
    bool isConnected = false;

    float batteryRemainingPercent = std::numeric_limits<float>::quiet_NaN(); // [0, 100], NaN if unknown (mavsdk::Telemetry::Battery)
    float batteryVoltageVolt = 0.0f;

    int gpsNumSatellites = 0;
    mavsdk::Telemetry::FixType gpsFixType = mavsdk::Telemetry::FixType::NoGps;

    bool rcAvailable = false;
    float rcSignalPercent = 0.0f;
};


#endif //COMMUNICATIONSTRUCTURES_H
