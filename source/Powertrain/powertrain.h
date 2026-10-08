/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef POWERTRAIN_H
#define POWERTRAIN_H

// APC 16x8E propeller, thrust from wind-tunnel fit (powertrain.cpp).

// Throttle in [0, 1] (rpm / 9000) giving thrustTarget [N] at airspeed [m/s].
// Always finite: 0 for a non-finite target or one of 2 N or less (motor
// off); a non-finite or negative airspeed is taken as 0.
double thrust2rpm(float airspeed, float thrustTarget);

// Thrust [N] at rpmTarget and airspeed, the inverse of thrust2rpm.
double rpm2thrust(float airspeed, float rpmTarget);

// Thrust [N] at 9000 rpm.
float maxThrust(float airspeed);

#endif //POWERTRAIN_H