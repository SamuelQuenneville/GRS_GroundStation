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

#include <cstdint>

struct subscriptionHandles {
    mavsdk::Telemetry::HealthHandle                      healthHandle;
    mavsdk::Telemetry::HealthAllOkHandle                 healthAllOkHandle;
    mavsdk::Telemetry::ArmedHandle                       armedHandle;
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
    double latitudeDegree;
    double longitudeDegree;
    float altitudeAmslMeter;
    float rollDegree;
    float pitchDegree;
    float yawDegree;
    //float rpm; TODO add to mavsdk?
};

struct controllerOutput {
    uint8_t sysId;
    float quaternion[4];
    float altitude;
    float thrust;
};


#endif //COMMUNICATIONSTRUCTURES_H
