/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef DASHBOARDTYPES_H
#define DASHBOARDTYPES_H

#pragma once

#include <string>

#include "jsonWriter.h"

enum class HealthStatus { Ok, Warn, Fail };

inline std::string toString(HealthStatus s) {
    switch (s) {
        case HealthStatus::Ok:   return "ok";
        case HealthStatus::Warn: return "warn";
        case HealthStatus::Fail: return "fail";
    }
    return "fail";
}

struct UavHealth {
    HealthStatus imu      = HealthStatus::Ok;
    HealthStatus baro     = HealthStatus::Ok;
    HealthStatus compass  = HealthStatus::Ok;
    HealthStatus gps      = HealthStatus::Ok;
    HealthStatus battery  = HealthStatus::Ok;
    HealthStatus rc       = HealthStatus::Ok;
};

struct UavTelemetrySnapshot {
    std::string id;                 // e.g. "UAV-01" -- stable key, used to match/create dashboard panels
    bool connected = false;
    bool armed = false;
    std::string mode;               // e.g. "OFFBOARD", "AUTO", "HOLD"

    double speed = 0.0;             // m/s, ground speed
    double altitude = 0.0;          // m, relative altitude
    double heading = 0.0;           // deg
    double climbRate = 0.0;         // m/s
    double voltage = 0.0;           // V
    double battery = 0.0;           // %
    double gpsHdop = 0.0;
    double throttle = 0.0;          // %

    std::string gpsFix;             // e.g. "3D Fix (10)"
    int satellites = 0;
    double rcSignal = 0.0;          // %
    std::string linkQuality;        // e.g. "Excellent", "Good", "Poor"
    int rssi = 0;                   // dBm

    UavHealth health;

    [[nodiscard]] std::string toJson() const {
        JsonWriter healthJson;
        healthJson.add("imu", toString(health.imu))
                  .add("baro", toString(health.baro))
                  .add("compass", toString(health.compass))
                  .add("gps", toString(health.gps))
                  .add("battery", toString(health.battery))
                  .add("rc", toString(health.rc));

        JsonWriter root;
        root.add("id", id)
            .add("connected", connected)
            .add("armed", armed)
            .add("mode", mode)
            .add("speed", speed)
            .add("altitude", altitude)
            .add("heading", heading)
            .add("climbRate", climbRate)
            .add("voltage", voltage)
            .add("battery", battery)
            .add("gpsHdop", gpsHdop)
            .add("throttle", throttle)
            .add("gpsFix", gpsFix)
            .add("satellites", satellites)
            .add("rcSignal", rcSignal)
            .add("linkQuality", linkQuality)
            .add("rssi", rssi)
            .addRaw("health", healthJson.str());
        return root.str();
    }
};

#endif //DASHBOARDTYPES_H
