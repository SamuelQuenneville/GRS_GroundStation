/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef VEHICLESTRUCTURES_H
#define VEHICLESTRUCTURES_H

#pragma once

#include <optional>

// Per-vehicle state/command structs exchanged between the controller core
// (Controller/Estimator) and whatever drives it: the live GCS
// (communicationStructures.h re-exports this header) or grs_batchsim.
// Deliberately free of MAVSDK so the controller core builds without it.

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

// Feedforward for the onboard attitude loop. 0: none available.
struct uavEstimates {
    float aoaDegree = 0.0f;
    float tension = 0.0f;   // [N]
}__attribute__((packed));

// Bits of uavCommandsFlags::flags, as decoded by the ArduPilot SITL fork.
namespace commandFlag {
inline constexpr uint8_t kShouldMove = 1u << 0; // in-flight initialization, no launcher
inline constexpr uint8_t kEndSim     = 1u << 1; // end of a command file
inline constexpr uint8_t kLaunch     = 1u << 2; // trigger the launch in SITL
}

struct uavCommandsFlags {
    uavCommands commands{};
    std::optional<double> timestamp;
    uavEstimates estimates{};
    uint8_t flags = 0;      // commandFlag bits
};

#endif //VEHICLESTRUCTURES_H
