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

#include <stdint.h>

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

struct attitude {
    double roll;
    double pitch;
    double yaw;
};

struct controllerInput {

};

struct controllerOutput {
    uint8_t sysId;
    float quaternion[4];
    float altitude;
    float thrust;
};


#endif //COMMUNICATIONSTRUCTURES_H
