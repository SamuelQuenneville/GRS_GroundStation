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

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>

struct subscriptionHandles {
    mavsdk::Telemetry::HealthHandle                      healthHandle;
    mavsdk::Telemetry::HealthAllOkHandle                 healthAllOkHandle;
    mavsdk::Telemetry::ArmedHandle                       armedHandle;
    mavsdk::Telemetry::HomeHandle                        homeHandle;
    mavsdk::Telemetry::FlightModeHandle                  flightModeHandle;
    mavsdk::Telemetry::AttitudeEulerHandle               attitudeHandle;
    mavsdk::Telemetry::AttitudeAngularVelocityBodyHandle angularVelocityHandle;
    mavsdk::Telemetry::PositionVelocityNedHandle         positionVelocityNedHandle;
    mavsdk::Telemetry::PositionHandle                    positionHandle;
    mavsdk::Telemetry::VelocityNedHandle                 velocityNedHandle;
    mavsdk::Telemetry::HeadingHandle                     headingHandle;
    mavsdk::Telemetry::FixedwingMetricsHandle            fixedwingMetricsHandle;
};

struct uavHealth {
    mavsdk::Telemetry::FlightMode flightMode = mavsdk::Telemetry::FlightMode::Unknown;
    mavsdk::Telemetry::Health health;
    bool isHealthy = false;
    bool isArmed = false;
};

struct uavStates {
    float airspeedMeterSecond;
    float northMeter;
    float eastMeter;
    float downMeter;
    float northMeterSecond;
    float eastMeterSecond;
    float downMeterSecond;
    float rollDegree;
    float pitchDegree;
    float yawDegree;
    double altitudeAmslMeter;
    double latitudeDegree;
    double longitudeDegree;
}__attribute__((packed));

struct uavCommands {
    float sysId;            // float mean easier encoding/decoding with matlab
    float rollDegree;
    float pitchDegree;
    float yawDegree;
    float thrust;           // [0 1]
}__attribute__((packed));

struct uavCommandsFlags {
    uavCommands commands{};
    std::optional<double> timestamp;
    std::optional<bool> F1Command;
    std::optional<bool> F2Command;
    std::optional<bool> F3Command;
};

struct aggregatorRates {
    std::chrono::steady_clock::time_point lastAttitude;
    std::chrono::steady_clock::time_point lastPosition;
    std::chrono::steady_clock::time_point lastVelocity;
    std::chrono::steady_clock::time_point lastAirspeed;
    std::chrono::steady_clock::time_point lastGlobalPosition;
    double rateAttitude = 0.0;
    double ratePosition = 0.0;
    double rateVelocity = 0.0;
    double rateAirspeed = 0.0;
    double rateGlobalPosition = 0.0;
};

#endif //COMMUNICATIONSTRUCTURES_H
