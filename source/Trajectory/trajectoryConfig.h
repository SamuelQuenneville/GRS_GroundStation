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

#include "Mathematics/math.h"

namespace grs::trajgen {

struct TrajectoryConfig {

    double simDt = 0.05; // Time step of simulation [s]

    struct World {
        grs::Vec3d wind = grs::Vec3d::zeros(); // Wind speed [m/s] (North, East, Down)
        double airDensity = 1.225;             // Air density [kg/m^3]
        double gravity = 9.81;                 // Gravitational acceleration [m/s^2]
    } world;

    struct Aircraft {
        // Calibration 2026-10-28 (GRS_Simulator sigCoeffs). Its CL = CL_TRIM + CL_ALPHA (a - a0)
        // + CL_de (de - de0) is reduced to CL0 + CLalpha a with the elevator at moment trim
        // (de - de0 = -Cm_ALPHA / Cm_de (a - a0)), q = 0.
        double mass = 4.613;           // Mass [kg]
        double wingArea = 0.53625;     // Wing surface [m^2]
        double aspectRatio = 4.924242; // Wing aspect ratio
        double oswald = 0.59263;       // Oswald efficiency
        double CD0 = 0.05076;          // Aircraft CD0
        double CL0 = -0.00279;         // Aircraft CL0
        double CLalpha = 4.32313;      // Aircraft CL_alpha [rad^-1]
    } aircraft;

    struct Tether {
        double length = 30;         // Tether length [m] -- final/nominal, once fully paid out
        double linearMass = 0.0005; // Tether linear mass [kg/m]
        int nSegments = 10;         // Number of segments for discretization

        /// @name Slip-clutch payout
        /// The launcher's slip clutch pays out ~1m of tether as it goes
        /// taut, so the aircraft is on a shorter tether right at launch
        /// than the nominal `length` above. Modeled as a smooth ramp from
        /// `lengthAtLaunch` (t=0) to `length` (t=payoutDurationSeconds,
        /// held thereafter), see generateAircraftTakeoff() in
        /// trajectoryGenerator.cpp for the exact spherical to Cartesian
        /// reconstruction. One value per UAV. A negative `lengthAtLaunch`
        /// means "not set", resolved to `length` by finalize() below (no
        /// payout modeled) unless explicitly set shorter than `length`.
        /// @{
        std::vector<double> lengthAtLaunch = {-1.0, -1.0}; // Distance payload to aircraft at rest on the launcher per UAV [m]; <0 = same as `length`
        double payoutDurationSeconds = 1.5; // Time to ramp lengthAtLaunch -> length [s]; only matters if lengthAtLaunch < length
        /// @}

        /// Derived by finalize() from length/linearMass/nSegments.
        std::vector<double> segmentMass;
        std::vector<double> segmentLinCoordNorm;
    } tether;

    struct Payload {
        double mass = 20;  // Payload mass [kg]
        double CdA = 0.5;  // Drag coefficient * Area [m^2]
    } payload;

    struct PayloadPath {
        grs::Vec3d x0 = grs::Vec3d::zeros(); // Payload origin [m] (NED)
        double timehold = 5;                 // Fixed payload time between maneuvers [s]

        double accClimb = 0.25;    // [m/s^2]
        double velClimb = 1;       // [m/s]
        double distanceClimb = 25; // [m], relative to origin

        double accMove = 0.25;    // [m/s^2]
        double velMove = 3;       // [m/s]
        double angleMoveDeg = 65; // Yaw angle for move maneuver [deg], from North toward East
        double distanceMove = 60; // [m]
    } payloadPath;

    struct AircraftPath {
        int direction = -1;                         // +1 = CW, -1 = CCW seen from above (azimuth from North toward East)
        double velMean = 26;                        // Aircraft mean speed [m/s]
        std::vector<double> phaseRad = {0.0, M_PI}; // Aircraft azimuth at rest on the launcher per UAV [rad], from North toward East
        double radius = 25;                         // Aircraft path radius [m]
        std::vector<double> z0 = {1.382, 1.382};    // Aircraft height above the payload at rest on the launcher per UAV [m], positive up

        double takeoffTime = 10;          // Takeoff phase duration [s]
        double takeoffTimeBalistic = 1.5; // Max duration of the pitch held at the rail angle after the release [s]
        double takeoffTimeAcc = 8;        // Time to reach velMean [s]
        double takeoffVel0 = 9.5;         // Aircraft speed at the catapult release [m/s], flight 2026-08-25
        double takeoffAcc0 = 1.75;        // Acceleration right after the release [m/s^2], flight 2026-08-25
        double railLength = 2.15;         // Catapult stroke [m], rail inclined at takeoffPitch0Deg: samples at rest to takeoffVel0 prepended, 0 = none
        double takeoffRoll0Deg = 10;      // Launcher roll angle [deg]

        double takeoffPitch0Deg = 19;           // Rail inclination [deg]: pitch held after the release, initial path angle
    } aircraftPath;

    /// @name Field calibration
    /// Both no-ops at their defaults. Applied post-generation by
    /// TrajectoryGenerator::applyFieldCalibration(), not threaded through
    /// generate() itself.
    /// @{
    double fieldHeadingDeg = 0.0;
    Vec3d originOffsetNed = Vec3d::zeros(); // e.g. live GPS launch point, NED relative to the NavigationFrameManager origin
    /// @}

    /// Populates tether.segment* from tether.length/linearMass/nSegments.
    /// Call once after setting the raw tether fields (or after loading from YAML).
    void finalize() {
        for (auto& L0 : tether.lengthAtLaunch) {
            if (L0 < 0.0) L0 = tether.length;
        }

        tether.segmentMass.assign(tether.nSegments, (tether.length * tether.linearMass) / tether.nSegments);
        tether.segmentLinCoordNorm.resize(tether.nSegments);
        for (int i = 0; i < tether.nSegments; ++i) {
            // MATLAB: linspace(1/n*0.5, 1-(1/n), n)
            const double start = (1.0 / tether.nSegments) * 0.5;
            const double stop = 1.0 - (1.0 / tether.nSegments);
            tether.segmentLinCoordNorm[i] = (tether.nSegments == 1) ? start : start + (stop - start) * i / (tether.nSegments - 1);
        }
    }
};

}

#endif //TRAJECTORYCONFIG_H
