/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "powertrain.h"

#include <algorithm>
#include <cmath>

namespace {
    // thrust = (-X*u^2 - Y*u + Z) * rho * rps^2 * D^4, u = airspeed / (rps*D).
    // With n in rpm and phi = 60*airspeed/D (so u = phi/n):
    // thrust = psi * (Z*n^2 - Y*phi*n - X*phi^2), psi = rho*D^4/3600.
    constexpr double kAirDensity = 1.225;            // [kg/m^3]
    constexpr double kPropDiameter = 16.0 * 0.0254;  // [m]
    constexpr double kThrustCoeffX = 0.1588;
    constexpr double kThrustCoeffY = 0.0106;
    constexpr double kThrustCoeffZ = 0.0757;
    constexpr double kPsi = kAirDensity * kPropDiameter * kPropDiameter * kPropDiameter * kPropDiameter / 3600.0;
    constexpr double kMaxRpm = 9000.0;
}

double thrust2rpm(const float airspeed, const float thrustTarget) {
    if (!std::isfinite(thrustTarget)) {
        return 0.0;
    }

    const double v = std::isfinite(airspeed) ? std::max(static_cast<double>(airspeed), 0.0) : 0.0;
    const double phi = 60.0 * v / kPropDiameter;

    // Positive root of Z*n^2 - Y*phi*n - (X*phi^2 + T/psi) = 0.
    const double c = kThrustCoeffX * phi * phi + std::max(static_cast<double>(thrustTarget), 0.0) / kPsi;
    const double rpm = (kThrustCoeffY * phi + std::sqrt(kThrustCoeffY * kThrustCoeffY * phi * phi + 4.0 * kThrustCoeffZ * c)) / (2.0 * kThrustCoeffZ);

    return std::clamp(rpm, 0.0, kMaxRpm) / kMaxRpm;
}

double rpm2thrust(const float airspeed, const float rpmTarget) {
    const double phi = 60.0 * airspeed / kPropDiameter;
    const double n = rpmTarget;
    return kPsi * (kThrustCoeffZ * n * n - kThrustCoeffY * phi * n - kThrustCoeffX * phi * phi);
}

float maxThrust(const float airspeed) {
    return static_cast<float>(rpm2thrust(airspeed, kMaxRpm));
}
