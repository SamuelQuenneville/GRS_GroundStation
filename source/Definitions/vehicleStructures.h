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

#include <cstdint>
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

struct uavCommandsFlags {
    uavCommands commands{};
    std::optional<double> timestamp;
    std::optional<bool> F1Command;
    std::optional<bool> F2Command;
    std::optional<bool> F3Command;
};

#endif //VEHICLESTRUCTURES_H
