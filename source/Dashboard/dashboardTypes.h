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

// Plain data describing UAV / payload state, as consumed by the web
// dashboard's app.js. Fill one of these from your MAVSDK telemetry
// subscriptions (see docs/MAVSDK_INTEGRATION.md) and hand it to
// DashboardServer::updateTelemetry() / updatePayloadTelemetry().
//
// Field names here are intentionally a 1:1 match with what js/app.js reads
// (setStat/setInfo/setHealth/setStatus calls) -- if you rename something in
// one place, rename it in the other.

#include <string>

#include "jsonWriter.h"

enum class HealthStatus { Ok, Warn, Fail };

inline std::string toString(const HealthStatus s) {
    switch (s) {
        case HealthStatus::Ok:   return "ok";
        case HealthStatus::Warn: return "warn";
        case HealthStatus::Fail: return "fail";
    }
    return "fail";
}

// Frontend renders launcher/catapult state as literal "yes"/"no" text
// (see setStatus() in app.js), not true/false -- so serialize it that way.
inline std::string yesNo(const bool v) { return v ? "yes" : "no"; }

struct UavHealth {
    HealthStatus imu     = HealthStatus::Ok;
    HealthStatus baro    = HealthStatus::Ok;
    HealthStatus compass = HealthStatus::Ok;
    HealthStatus gps     = HealthStatus::Ok;
    HealthStatus battery = HealthStatus::Ok;
    HealthStatus rc      = HealthStatus::Ok;
};

// Payload has no RC link, so no `rc` field -- matches the payload template's
// System Health card, which only lists IMU/Barometer/Compass/GPS/Battery.
struct PayloadHealth {
    HealthStatus imu     = HealthStatus::Ok;
    HealthStatus baro    = HealthStatus::Ok;
    HealthStatus compass = HealthStatus::Ok;
    HealthStatus gps     = HealthStatus::Ok;
    HealthStatus battery = HealthStatus::Ok;
};

// One catapult, embedded directly in the UAV it launches (matches the
// "Launcher" card inside the uav-panel-template, not a separate panel).
struct CatapultStateUi {
    bool connected = false;
    bool locked = false;
    bool armed = false;
    bool launched = false;
};

struct UavTelemetrySnapshot {
    std::string id;                 // e.g. "UAV-01" -- stable key, used to match/create dashboard panels
    bool connected = false;
    bool armed = false;
    std::string mode;               // e.g. "OFFBOARD", "AUTO", "HOLD"

    double airspeed = 0.0;          // m/s, true airspeed -- "TAS" card
    double groundspeed = 0.0;       // m/s -- "GS" card
    double altitude = 0.0;          // m, relative altitude -- "Alt" card
    double roll = 0.0;              // deg
    double pitch = 0.0;             // deg
    double rpm = 0.0;               // engine/motor RPM
    double cl = 0.0;                // lift coefficient
    double battery = 0.0;           // %
    double gpsHdop = 0.0;

    std::string gpsFix;              // e.g. "3D Fix (10)"
    int satellites = 0;
    double rcSignal = 0.0;           // %
    std::string linkQuality;         // e.g. "Excellent", "Good", "Poor"

    UavHealth health;
    CatapultStateUi launcher;        // this UAV's catapult

    std::string toJson() const {
        JsonWriter healthJson;
        healthJson.add("imu", toString(health.imu))
                  .add("baro", toString(health.baro))
                  .add("compass", toString(health.compass))
                  .add("gps", toString(health.gps))
                  .add("battery", toString(health.battery))
                  .add("rc", toString(health.rc));

        JsonWriter launcherJson;
        launcherJson.add("connected", yesNo(launcher.connected))
                    .add("locked", yesNo(launcher.locked))
                    .add("armed", yesNo(launcher.armed))
                    .add("launched", yesNo(launcher.launched));

        JsonWriter root;
        root.add("type", "uav")
            .add("id", id)
            .add("connected", connected)
            .add("armed", armed)
            .add("mode", mode)
            .add("airspeed", airspeed)
            .add("groundspeed", groundspeed)
            .add("altitude", altitude)
            .add("roll", roll)
            .add("pitch", pitch)
            .add("rpm", rpm)
            .add("cl", cl)
            .add("battery", battery)
            .add("gpsHdop", gpsHdop)
            .add("gpsFix", gpsFix)
            .add("satellites", satellites)
            .add("rcSignal", rcSignal)
            .add("linkQuality", linkQuality)
            .addRaw("health", healthJson.str())
            .addRaw("launcher", launcherJson.str());
        return root.str();
    }
};

// There's only one payload panel (see #payload-panel in index.html, a fixed
// container rather than an id-keyed grid), so no `id` field is needed here.
struct PayloadTelemetrySnapshot {
    bool connected = false;
    bool armed = false;
    std::string mode;

    double groundspeed = 0.0;        // m/s -- "GS" card
    double altitude = 0.0;           // m
    double battery = 0.0;            // %
    double gpsHdop = 0.0;

    std::string gpsFix;
    int satellites = 0;
    std::string linkQuality;

    PayloadHealth health;

    std::string toJson() const {
        JsonWriter healthJson;
        healthJson.add("imu", toString(health.imu))
                  .add("baro", toString(health.baro))
                  .add("compass", toString(health.compass))
                  .add("gps", toString(health.gps))
                  .add("battery", toString(health.battery));

        JsonWriter root;
        root.add("type", "payload")
            .add("connected", connected)
            .add("armed", armed)
            .add("mode", mode)
            .add("groundspeed", groundspeed)
            .add("altitude", altitude)
            .add("battery", battery)
            .add("gpsHdop", gpsHdop)
            .add("gpsFix", gpsFix)
            .add("satellites", satellites)
            .add("linkQuality", linkQuality)
            .addRaw("health", healthJson.str());
        return root.str();
    }
};

#endif //DASHBOARDTYPES_H
