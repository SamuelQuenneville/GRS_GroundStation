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

#include "jsonReader.h"
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

    // "Is the loaded trajectory the one I think it is" confirmation --
    // there was previously no way to tell from the dashboard whether
    // trajectoryTotal above reflects a stale trajectory from a previous
    // session/auto-reload or the one just generated/applied in the UI.
    uint64_t trajectoryLoadedAtMs = 0; // wall-clock ms, 0 = nothing loaded/generated yet this run
    int numUavs = 0;
    bool hasPayload = false;

    // Control-loop period (1000/hlcFrequency), sent alongside every solve
    // so the frontend can (a) judge lastSolveMs against its real deadline
    // budget instead of a guessed threshold, and (b) size its own
    // trackingNumber-stall detector off the actual loop rate rather than a
    // hardcoded constant.
    double loopPeriodMs = 0.0;

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
            .add("trajectoryTotal", static_cast<int>(trajectoryTotal))
            .add("trajectoryLoadedAtMs", static_cast<double>(trajectoryLoadedAtMs))
            .add("numUavs", numUavs)
            .add("hasPayload", hasPayload)
            .add("loopPeriodMs", loopPeriodMs);
        return root.str();
    }
};

struct TrajectoryPointJson {
    double north = 0.0, east = 0.0, down = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double roll = 0.0, pitch = 0.0;

    // Mission time [s] for this sample, shared across every vehicle's point
    // at the same array index -- lets the dashboard plot speed/roll/pitch vs
    // time. Exact for a freshly-generated preview (copied straight from
    // GeneratedMission::time); for the post-Apply/GET /api/trajectory replay
    // -- which reads back the NMPC controller's flattened reference array,
    // with no timestamps of its own -- it's approximated as
    // `index * TrajectoryConfig::simDt` (see
    // GroundControlStation::m_buildTrajectorySnapshotFromController()). Good
    // enough for this debug plot; not a control-loop-accurate timestamp.
    double time = 0.0;

    std::string toJson() const {
        JsonWriter root;
        root.add("north", north).add("east", east).add("down", down)
            .add("vx", vx).add("vy", vy).add("vz", vz)
            .add("roll", roll).add("pitch", pitch)
            .add("time", time);
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

// Operator-adjustable subset of grs::trajgen::TrajectoryConfig, for the
// dashboard's trajectory-generation sidebar (ADR-001 Phase 2). Deliberately
// flat and self-contained -- dashboardTypes.h stays independent of
// Trajectory/, same reasoning as DebugInfo/TrajectoryPointView above; gcs.cpp
// is what maps this to/from an actual TrajectoryConfig. Defaults here mirror
// TrajectoryConfig's own defaults (which mirror config.m) so a client that
// never calls GET /api/trajectory/generator-defaults still gets a sane
// trajectory; keep the two in sync if either changes.
struct TrajectoryGenerationParams {
    double radiusMeters = 26.0;
    double velMeanMetersPerSecond = 28.0;

    double climbDistanceMeters = 25.0;
    double climbVelMetersPerSecond = 1.0;
    double climbAccelMetersPerSecondSq = 0.25;

    double moveDistanceMeters = 60.0;
    double moveVelMetersPerSecond = 3.0;
    double moveAccelMetersPerSecondSq = 0.25;
    double moveAngleDegrees = 65.0; // local mission heading (payload move direction), distinct from fieldHeadingDeg below

    double holdTimeSeconds = 5.0;
    double tetherLengthMeters = 30.0;
    double payloadMassKg = 20.0;

    // Field calibration (ADR-001 Phase 2).
    double fieldHeadingDeg = 0.0;
    double originNorthOffsetMeters = 0.0;
    double originEastOffsetMeters = 0.0;
    double originDownOffsetMeters = 0.0;

    // Real launch-position seeding (ADR-001 Phase 3). Fixed at two aircraft,
    // same hard-coded scope as the rest of the generator. Defaults mirror the
    // MATLAB source's assumed perfect 180-degree separation ({0, pi} in
    // TrajectoryConfig::AircraftPath::phaseRad) -- a no-op until "Capture
    // live positions" (or the operator) sets these to the real bearings.
    // These are LOCAL/pre-field-heading-rotation angles: TrajectoryGenerator
    // ::applyFieldCalibration rotates the whole generated mission (including
    // these phase angles) by fieldHeadingDeg afterward, so the real-world
    // bearing an operator measures on site is `localPhaseDeg + fieldHeadingDeg`
    // (see trajectoryGenerator.cpp's rotationZ convention) -- the capture
    // button in setup3d.html does that conversion, not this struct.
    double uav1PhaseDeg = 0.0;
    double uav2PhaseDeg = 180.0;

    // Slip-clutch tether payout (ADR-001 Phase 3, see TrajectoryConfig::
    // Tether). Defaults equal to tetherLengthMeters/TrajectoryConfig's own
    // default, i.e. no payout modeled until set shorter than tetherLengthMeters.
    double tetherLengthAtLaunchMeters = 30.0;
    double tetherPayoutDurationSeconds = 1.5;

    // Aircraft elevation on the launcher at t=0 (NED down, meters) -- was
    // previously TrajectoryConfig::AircraftPath::z0, hardcoded and never
    // reachable from the dashboard. theta0 = asin(z0 / tetherLengthAtLaunch)
    // sets the launch-instant tether elevation angle (trajectoryGenerator.cpp),
    // so a wrong z0 shifts every takeoff/loiter waypoint even when phase and
    // tetherLengthAtLaunchMeters are both correct. Default mirrors
    // TrajectoryConfig::AircraftPath::z0's own default. "Capture live
    // positions" derives this (and tetherLengthAtLaunchMeters) from the real
    // payload/anchor -> UAV offset instead of requiring a manual guess.
    double aircraftZ0Meters = 1.382;

    // ADR-001 follow-up: when true, GroundControlStation rigidly translates
    // each UAV's whole trajectory so its first (launch) waypoint lands
    // exactly on that UAV's *captured* position below -- see
    // TrajectoryGenerator::snapToLiveLaunchPositions(). Position-only: it
    // doesn't touch the velocity/climb-angle shape phase/aircraftZ0Meters/
    // tetherLengthAtLaunchMeters above imply, it just guarantees the launch
    // point itself is exactly where the UAV actually was when captured, per
    // UAV, with no guessing and no shared-parameter averaging across UAVs.
    // Defaults to false (strict no-op) so a client that never sets this
    // keeps today's behavior exactly.
    //
    // Deliberately does NOT re-read live telemetry at generate/apply time:
    // the operator clicks "Capture live positions" once (see setup3d.html),
    // which snapshots each UAV's NED position into the uavNSnap* fields
    // below, and that snapshot is what gets applied on every subsequent
    // Generate/Apply until the operator re-captures -- so toggling this
    // checkbox or repeatedly clicking Generate never silently re-samples a
    // UAV that has since been picked up, nudged, or repositioned on the
    // launcher.
    bool snapToLiveLaunchPosition = false;

    // Per-UAV captured launch position (NED, meters, NavigationFrameManager
    // frame), populated client-side by "Capture live positions" in
    // setup3d.html from the same /api/trajectory/live-positions read used
    // to draw the live map markers. uavNSnapCaptured=false means "never
    // captured" (or captured then invalidated) -- GroundControlStation
    // leaves that UAV's launch point untouched even when
    // snapToLiveLaunchPosition is true, same as if it had no live fix.
    bool uav1SnapCaptured = false;
    double uav1SnapNorthMeters = 0.0;
    double uav1SnapEastMeters = 0.0;
    double uav1SnapDownMeters = 0.0;
    bool uav2SnapCaptured = false;
    double uav2SnapNorthMeters = 0.0;
    double uav2SnapEastMeters = 0.0;
    double uav2SnapDownMeters = 0.0;

    // Reduced-order testing (ADR-001 Phase 4): apply/preview only a subset of
    // the full 2-UAV+payload mission -- e.g. to exercise a simplified NMPC
    // build compiled for one UAV tethered to a fixed ground anchor (no
    // payload), stopped partway through the mission (through the first
    // loiter, say). testEnabled=false (default) is a strict no-op: the
    // trajectory generator/apply path behaves exactly as if this section
    // didn't exist, deferring to the loaded NMPCController's own
    // hasPayload()/numUavs() -- see GroundControlStation::
    // m_paramsToSubsetSelection(). Only consulted when testEnabled=true.
    bool testEnabled = false;
    bool testIncludeUav1 = true;
    bool testIncludeUav2 = true;
    bool testIncludePayload = true;
    double testMaxDurationSeconds = 0.0; // <=0 = no limit (full mission)

    std::string toJson() const {
        JsonWriter root;
        root.add("radiusMeters", radiusMeters)
            .add("velMeanMetersPerSecond", velMeanMetersPerSecond)
            .add("climbDistanceMeters", climbDistanceMeters)
            .add("climbVelMetersPerSecond", climbVelMetersPerSecond)
            .add("climbAccelMetersPerSecondSq", climbAccelMetersPerSecondSq)
            .add("moveDistanceMeters", moveDistanceMeters)
            .add("moveVelMetersPerSecond", moveVelMetersPerSecond)
            .add("moveAccelMetersPerSecondSq", moveAccelMetersPerSecondSq)
            .add("moveAngleDegrees", moveAngleDegrees)
            .add("holdTimeSeconds", holdTimeSeconds)
            .add("tetherLengthMeters", tetherLengthMeters)
            .add("payloadMassKg", payloadMassKg)
            .add("fieldHeadingDeg", fieldHeadingDeg)
            .add("originNorthOffsetMeters", originNorthOffsetMeters)
            .add("originEastOffsetMeters", originEastOffsetMeters)
            .add("originDownOffsetMeters", originDownOffsetMeters)
            .add("uav1PhaseDeg", uav1PhaseDeg)
            .add("uav2PhaseDeg", uav2PhaseDeg)
            .add("tetherLengthAtLaunchMeters", tetherLengthAtLaunchMeters)
            .add("tetherPayoutDurationSeconds", tetherPayoutDurationSeconds)
            .add("aircraftZ0Meters", aircraftZ0Meters)
            .add("snapToLiveLaunchPosition", snapToLiveLaunchPosition)
            .add("uav1SnapCaptured", uav1SnapCaptured)
            .add("uav1SnapNorthMeters", uav1SnapNorthMeters)
            .add("uav1SnapEastMeters", uav1SnapEastMeters)
            .add("uav1SnapDownMeters", uav1SnapDownMeters)
            .add("uav2SnapCaptured", uav2SnapCaptured)
            .add("uav2SnapNorthMeters", uav2SnapNorthMeters)
            .add("uav2SnapEastMeters", uav2SnapEastMeters)
            .add("uav2SnapDownMeters", uav2SnapDownMeters)
            .add("testEnabled", testEnabled)
            .add("testIncludeUav1", testIncludeUav1)
            .add("testIncludeUav2", testIncludeUav2)
            .add("testIncludePayload", testIncludePayload)
            .add("testMaxDurationSeconds", testMaxDurationSeconds);
        return root.str();
    }

    // Missing fields fall back to this same struct's defaults (see class
    // comment) -- so a partial body (e.g. only fieldHeadingDeg changed)
    // still produces a complete, reasonable set of parameters.
    static TrajectoryGenerationParams fromJson(const std::string& body) {
        const JsonReader reader(body);
        TrajectoryGenerationParams p;
        p.radiusMeters = reader.getNumber("radiusMeters", p.radiusMeters);
        p.velMeanMetersPerSecond = reader.getNumber("velMeanMetersPerSecond", p.velMeanMetersPerSecond);
        p.climbDistanceMeters = reader.getNumber("climbDistanceMeters", p.climbDistanceMeters);
        p.climbVelMetersPerSecond = reader.getNumber("climbVelMetersPerSecond", p.climbVelMetersPerSecond);
        p.climbAccelMetersPerSecondSq = reader.getNumber("climbAccelMetersPerSecondSq", p.climbAccelMetersPerSecondSq);
        p.moveDistanceMeters = reader.getNumber("moveDistanceMeters", p.moveDistanceMeters);
        p.moveVelMetersPerSecond = reader.getNumber("moveVelMetersPerSecond", p.moveVelMetersPerSecond);
        p.moveAccelMetersPerSecondSq = reader.getNumber("moveAccelMetersPerSecondSq", p.moveAccelMetersPerSecondSq);
        p.moveAngleDegrees = reader.getNumber("moveAngleDegrees", p.moveAngleDegrees);
        p.holdTimeSeconds = reader.getNumber("holdTimeSeconds", p.holdTimeSeconds);
        p.tetherLengthMeters = reader.getNumber("tetherLengthMeters", p.tetherLengthMeters);
        p.payloadMassKg = reader.getNumber("payloadMassKg", p.payloadMassKg);
        p.fieldHeadingDeg = reader.getNumber("fieldHeadingDeg", p.fieldHeadingDeg);
        p.originNorthOffsetMeters = reader.getNumber("originNorthOffsetMeters", p.originNorthOffsetMeters);
        p.originEastOffsetMeters = reader.getNumber("originEastOffsetMeters", p.originEastOffsetMeters);
        p.originDownOffsetMeters = reader.getNumber("originDownOffsetMeters", p.originDownOffsetMeters);
        p.uav1PhaseDeg = reader.getNumber("uav1PhaseDeg", p.uav1PhaseDeg);
        p.uav2PhaseDeg = reader.getNumber("uav2PhaseDeg", p.uav2PhaseDeg);
        p.tetherLengthAtLaunchMeters = reader.getNumber("tetherLengthAtLaunchMeters", p.tetherLengthAtLaunchMeters);
        p.tetherPayoutDurationSeconds = reader.getNumber("tetherPayoutDurationSeconds", p.tetherPayoutDurationSeconds);
        p.aircraftZ0Meters = reader.getNumber("aircraftZ0Meters", p.aircraftZ0Meters);
        p.snapToLiveLaunchPosition = reader.getBool("snapToLiveLaunchPosition", p.snapToLiveLaunchPosition);
        p.uav1SnapCaptured = reader.getBool("uav1SnapCaptured", p.uav1SnapCaptured);
        p.uav1SnapNorthMeters = reader.getNumber("uav1SnapNorthMeters", p.uav1SnapNorthMeters);
        p.uav1SnapEastMeters = reader.getNumber("uav1SnapEastMeters", p.uav1SnapEastMeters);
        p.uav1SnapDownMeters = reader.getNumber("uav1SnapDownMeters", p.uav1SnapDownMeters);
        p.uav2SnapCaptured = reader.getBool("uav2SnapCaptured", p.uav2SnapCaptured);
        p.uav2SnapNorthMeters = reader.getNumber("uav2SnapNorthMeters", p.uav2SnapNorthMeters);
        p.uav2SnapEastMeters = reader.getNumber("uav2SnapEastMeters", p.uav2SnapEastMeters);
        p.uav2SnapDownMeters = reader.getNumber("uav2SnapDownMeters", p.uav2SnapDownMeters);
        p.testEnabled = reader.getBool("testEnabled", p.testEnabled);
        p.testIncludeUav1 = reader.getBool("testIncludeUav1", p.testIncludeUav1);
        p.testIncludeUav2 = reader.getBool("testIncludeUav2", p.testIncludeUav2);
        p.testIncludePayload = reader.getBool("testIncludePayload", p.testIncludePayload);
        p.testMaxDurationSeconds = reader.getNumber("testMaxDurationSeconds", p.testMaxDurationSeconds);
        return p;
    }
};

// One live vehicle fix -- real GPS-derived NED position (+ yaw for UAVs) at
// the moment "Capture live positions" was clicked in the trajectory
// generator sidebar (ADR-001 Phase 3). Same NED frame as everything else
// here (relative to the NavigationFrameManager origin), so it's directly
// comparable to TrajectoryPointJson / OriginSnapshot without conversion.
struct LiveVehicleFix {
    std::string id;      // e.g. "uav1", "payload"
    double north = 0.0, east = 0.0, down = 0.0;
    double rollDeg = 0.0, pitchDeg = 0.0, yawDeg = 0.0;

    std::string toJson() const {
        JsonWriter root;
        root.add("id", id).add("north", north).add("east", east).add("down", down)
            .add("rollDeg", rollDeg).add("pitchDeg", pitchDeg).add("yawDeg", yawDeg);
        return root.str();
    }
};

// `available` is false until the nav frame has a GPS-derived offset for at
// least one UAV (see NavigationFrameManager::isInitialized()) -- e.g. before
// GPS lock, or before controlMode==MPC has even started. `hasPayload` is
// separate: the payload may have its own real GPS link (ADR-001 Phase 3) but
// not be reporting yet even while the UAVs are.
struct LivePositionsSnapshot {
    bool available = false;
    std::vector<LiveVehicleFix> uavs;
    bool hasPayload = false;
    LiveVehicleFix payload;

    std::string toJson() const {
        std::vector<std::string> uavJsons;
        uavJsons.reserve(uavs.size());
        for (const auto& u : uavs) uavJsons.push_back(u.toJson());

        JsonWriter root;
        root.add("available", available)
            .addRaw("uavs", jsonArray(uavJsons))
            .add("hasPayload", hasPayload);
        if (hasPayload) root.addRaw("payload", payload.toJson());
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
