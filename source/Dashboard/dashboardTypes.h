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

// Joins already-serialized JSON fragments into a JSON array. JsonWriter
// itself deliberately stays flat-object-only (see jsonWriter.h) -- this is
// the one place multiple snapshots need array nesting, so it lives here
// rather than growing JsonWriter's scope.
inline std::string jsonArray(const std::vector<std::string>& items) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < items.size(); ++i) {
        if (i) oss << ",";
        oss << items[i];
    }
    oss << "]";
    return oss.str();
}

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
    // No launcher field here on purpose -- catapult state is decoupled from
    // the UAV panel and gets its own dashboard section (see
    // LauncherTelemetrySnapshot below).

    std::string toJson() const {
        JsonWriter healthJson;
        healthJson.add("imu", toString(health.imu))
                  .add("baro", toString(health.baro))
                  .add("compass", toString(health.compass))
                  .add("gps", toString(health.gps))
                  .add("battery", toString(health.battery))
                  .add("rc", toString(health.rc));

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
            .addRaw("health", healthJson.str());
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

// One catapult launcher, its own dashboard panel (decoupled from any UAV --
// see CatapultLauncher::setStatusCallback()). `id` matches the CatapultEndpoint
// id from config, stringified (e.g. "1", "2").
struct LauncherTelemetrySnapshot {
    std::string id;
    std::string state;        // e.g. "Disconnected", "Armed", "Countdown", "Fault" -- see CatapultState
    bool connected = false;
    bool cocked = false;
    bool armed = false;
    bool countdown = false;
    bool lowBattery = false;
    bool safetyPinIn = false;
    bool gcsTimeout = false;
    int battery = 0;

    std::string toJson() const {
        JsonWriter root;
        root.add("type", "launcher")
            .add("id", id)
            .add("state", state)
            .add("connected", yesNo(connected))
            .add("cocked", yesNo(cocked))
            .add("armed", yesNo(armed))
            .add("countdown", yesNo(countdown))
            .add("lowBattery", yesNo(lowBattery))
            .add("safetyPinIn", yesNo(safetyPinIn))
            .add("gcsTimeout", yesNo(gcsTimeout))
            .add("battery", battery);
        return root.str();
    }
};

// NMPC controller debug/health panel -- one snapshot for the whole
// controller (it solves for every UAV jointly), not per-UAV.
struct NmpcTelemetrySnapshot {
    bool launched = false;
    bool inFlight = false;
    bool endedTraj = false;
    bool violation = false;      // last solve hit a constraint/solver issue

    double lastSolveMs = 0.0;
    size_t trackingNumber = 0;   // solve iteration counter
    size_t trajectoryIndex = 0;
    size_t trajectoryTotal = 0;

    std::string toJson() const {
        JsonWriter root;
        root.add("type", "nmpc")
            .add("launched", launched)
            .add("inFlight", inFlight)
            .add("endedTraj", endedTraj)
            .add("violation", violation)
            .add("lastSolveMs", lastSolveMs)
            .add("trackingNumber", static_cast<int>(trackingNumber))
            .add("trajectoryIndex", static_cast<int>(trajectoryIndex))
            .add("trajectoryTotal", static_cast<int>(trajectoryTotal));
        return root.str();
    }
};

struct TrajectoryPointJson {
    double north = 0.0, east = 0.0, down = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double roll = 0.0, pitch = 0.0;

    std::string toJson() const {
        JsonWriter root;
        root.add("north", north).add("east", east).add("down", down)
            .add("vx", vx).add("vy", vy).add("vz", vz)
            .add("roll", roll).add("pitch", pitch);
        return root.str();
    }
};

struct TrajectoryVehicleSnapshot {
    std::string id;      // e.g. "uav1", "payload"
    std::string label;   // e.g. "UAV 1", "Payload"
    std::string color;   // "#rrggbb", used directly by the 3D view
    std::vector<TrajectoryPointJson> points;

    std::string toJson() const {
        std::vector<std::string> pointJsons;
        pointJsons.reserve(points.size());
        for (const auto& p : points) pointJsons.push_back(p.toJson());

        JsonWriter root;
        root.add("id", id).add("label", label).add("color", color)
            .addRaw("points", jsonArray(pointJsons));
        return root.str();
    }
};

// Origin + reference trajectory, as read once by the setup/orientation 3D
// view (dashboard/setup3d.html) via GET /api/origin and GET /api/trajectory
// -- not pushed over the WebSocket like everything else here, since
// neither changes at telemetry rates.
struct OriginSnapshot {
    bool hasOrigin = false;
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;

    std::string toJson() const {
        JsonWriter root;
        root.add("initialized", hasOrigin);
        if (hasOrigin) {
            root.add("latitude", latitude).add("longitude", longitude).add("altitude", altitude);
        }
        return root.str();
    }
};

struct TrajectorySnapshot {
    std::vector<TrajectoryVehicleSnapshot> vehicles;

    std::string toJson() const {
        std::vector<std::string> vehicleJsons;
        vehicleJsons.reserve(vehicles.size());
        for (const auto& v : vehicles) vehicleJsons.push_back(v.toJson());

        JsonWriter root;
        root.addRaw("vehicles", jsonArray(vehicleJsons));
        return root.str();
    }
};

#endif //DASHBOARDTYPES_H
