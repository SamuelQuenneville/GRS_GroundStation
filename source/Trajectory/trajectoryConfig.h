/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef TRAJECTORYCONFIG_H
#define TRAJECTORYCONFIG_H

#pragma once

#include <vector>

#include "Mathematics/math.h"

namespace grs::trajgen {

// C++ mirror of trajectory_generation/config/config.m -- field names and
// default numeric values are kept 1:1 with that MATLAB struct so the two can
// be diffed directly. See ADR-001 (docs/adr) for why this exists: this is
// the Phase 0 port (math only, no field-calibration wiring yet).
//
// Two additions beyond the MATLAB config, both no-ops at their defaults so
// Phase 0 output matches MATLAB exactly (originLatitude/Longitude unset,
// fieldHeadingDeg = 0):
//   - fieldHeadingDeg: yaw-rotates the whole generated trajectory about the
//     origin before it's handed to the solver, replacing MATLAB's implicit
//     "everything is relative to True North" assumption. Applied post-generation (see
//     TrajectoryGenerator::applyFieldCalibration), not baked into the core
//     math ported here.
//   - startPositionOverride: when set, replaces payloadPath.x0 as the mission
//     start point (e.g. a live GPS-derived NED position) instead of the
//     config's fixed origin. Phase 2 concern; unused by Phase 0.
struct TrajectoryConfig {

    // ---- Simulation ----
    double simDt = 0.05; // Time step of simulation [s]

    // ---- World ----
    struct World {
        grs::Vec3d wind = grs::Vec3d::zeros(); // Wind speed [m/s] (North, East, Down)
        double airDensity = 1.225;             // Air density [kg/m^3]
        double gravity = 9.81;                 // Gravitational acceleration [m/s^2]
    } world;

    // ---- Aircraft ----
    struct Aircraft {
        double mass = 4.475;         // Mass [kg]
        double wingArea = 0.53625;   // Wing surface [m^2]
        double aspectRatio = 4.924242; // Wing aspect ratio
        double oswald = 0.92;        // Oswald efficiency
        double CD0 = 0.05;           // Aircraft CD0
        double CL0 = 0.21385;        // Aircraft CL0
        double CLalpha = 4.32893;    // Aircraft CL_alpha [rad^-1]
    } aircraft;

    // ---- Tether ----
    struct Tether {
        double length = 30;        // Tether length [m] -- final/nominal, once fully paid out
        double linearMass = 0.0005; // Tether linear mass [kg/m]
        int nSegments = 10;        // Number of segments for discretization

        // ---- Slip-clutch payout (new vs. MATLAB; Phase 3) ----
        // The launcher's slip clutch pays out ~1m of tether as it goes taut,
        // so the aircraft is on a *shorter* tether right at launch than the
        // nominal `length` above. Modeled as a smooth ramp from
        // `lengthAtLaunch` (t=0) to `length` (t=payoutDurationSeconds, held
        // thereafter) purely in TrajectoryGenerator::generateAircraftTakeoff's
        // spherical->Cartesian reconstruction -- see that function's comments
        // for exactly what is and isn't re-derived for a time-varying radius.
        // Sentinel: a negative `lengthAtLaunch` means "not set", resolved to
        // `length` by finalize() below -- i.e. a no-op (no payout modeled)
        // unless a caller explicitly sets it shorter than `length`.
        double lengthAtLaunch = -1.0;       // Tether length at t=0 [m]; <0 = same as `length`
        double payoutDurationSeconds = 1.5; // Time to ramp lengthAtLaunch -> length [s]; only matters if lengthAtLaunch < length

        // Derived (computed by TrajectoryConfig::finalize(), mirrors config.m
        // deriving config.tether.segment.* from length/linearMass/nSegments).
        std::vector<double> segmentMass;
        std::vector<double> segmentLength;
        std::vector<double> segmentLinCoordNorm;
    } tether;

    // ---- Payload ----
    struct Payload {
        double mass = 20;  // Payload mass [kg]
        double CdA = 0.5;  // Drag coefficient * Area [m^2]
    } payload;

    // ---- Payload trajectory ----
    struct PayloadPath {
        grs::Vec3d x0 = grs::Vec3d::zeros(); // Payload origin [m] (NED)
        double timehold = 5;                 // Fixed payload time between maneuvers [s]

        double accClimb = 0.25;   // [m/s^2]
        double velClimb = 1;      // [m/s]
        double distanceClimb = 25; // [m], relative to origin

        double accMove = 0.25;    // [m/s^2]
        double velMove = 3;       // [m/s]
        double angleMoveDeg = 65; // Yaw angle for move maneuver [deg], from North toward East
        double distanceMove = 60; // [m]
    } payloadPath;

    // ---- Aircraft trajectory ----
    struct AircraftPath {
        int direction = -1;               // +1 = CCW, -1 = CW (top view)
        double velMean = 28;              // Aircraft mean speed [m/s]
        std::vector<double> phaseRad = {0.0, M_PI}; // Aircraft phase per UAV [rad], from North toward East
        double radius = 26;               // Aircraft path radius [m]
        double z0 = 1.382;                // Aircraft altitude (NED) on launcher [m]

        double takeoffTime = 10;          // Takeoff phase duration [s]
        double takeoffTimeBalistic = 1.6; // Decay time pitch0 -> pitchDecay [s]
        double takeoffTimeAcc = 8;        // Time to reach velMean [s]
        double takeoffVel0 = 12;          // Aircraft speed after launch [m/s]
        double takeoffAngle0Deg = 0;      // Azimuth of aircraft 1 on launcher [deg]
        double takeoffRoll0Deg = 10;      // Launcher roll angle [deg]
        // MATLAB: deg2rad(15) - 0.1687 -- the -0.1687 rad trim offset is an
        // empirical alpha/path-angle correction from the launcher rig, not a
        // clean constant, so it's kept as a named field here rather than
        // folded silently into takeoffPitch0Deg.
        double takeoffPitch0Deg = 15;
        double takeoffPitch0TrimRad = -0.1687;
        double takeoffPitchDecayFrac = 0.4; // Fraction of initial pitch to decay to
    } aircraftPath;

    // ---- Field calibration (new vs. MATLAB; Phase 2) ----
    // Both no-ops at their defaults, so TrajectoryGenerator::generate() output
    // is unaffected until a caller opts in. Applied post-generation by
    // TrajectoryGenerator::applyFieldCalibration() -- see that function's
    // comment in trajectoryGenerator.h for why this stays out of the core
    // math instead of being threaded through generate() itself.
    double fieldHeadingDeg = 0.0;
    Vec3d originOffsetNed = Vec3d::zeros(); // e.g. live GPS launch point, NED relative to the NavigationFrameManager origin

    // Populates tether.segment* from tether.length/linearMass/nSegments --
    // mirrors config.m's derivation of config.tether.segment.mass/length/
    // linCoordNorm. Call once after setting the raw tether fields (or after
    // loading from YAML).
    void finalize() {
        if (tether.lengthAtLaunch < 0.0) tether.lengthAtLaunch = tether.length;

        tether.segmentMass.assign(tether.nSegments, (tether.length * tether.linearMass) / tether.nSegments);
        tether.segmentLength.assign(tether.nSegments, tether.length / tether.nSegments);
        tether.segmentLinCoordNorm.resize(tether.nSegments);
        for (int i = 0; i < tether.nSegments; ++i) {
            // MATLAB: linspace(1/n*0.5, 1-(1/n), n)
            const double start = (1.0 / tether.nSegments) * 0.5;
            const double stop = 1.0 - (1.0 / tether.nSegments);
            tether.segmentLinCoordNorm[i] = (tether.nSegments == 1)
                ? start
                : start + (stop - start) * i / (tether.nSegments - 1);
        }
    }
};

}

#endif //TRAJECTORYCONFIG_H
