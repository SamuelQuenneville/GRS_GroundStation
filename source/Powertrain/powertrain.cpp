/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "powertrain.h"

namespace {
    /// APC 16x8E propeller thrust model, fit from wind tunnel data:
    /// thrust = (-x*u^2 - y*u + z) * rho * rps^2 * D^4, u = airspeed / (rps*D).
    constexpr float kAirDensity = 1.225f;            // [kg/m^3]
    constexpr float kPropDiameter = 16.0f * 0.0254f; // [m]
    constexpr float kThrustCoeffX = 0.1588f;
    constexpr float kThrustCoeffY = 0.0106f;
    constexpr float kThrustCoeffZ = 0.0757f;
}


inline evalResult evalThrustModel(const float n, const float phi, const float psi, const float thrustTarget) {
    const float inv_n  = 1.0f / n;
    const float inv_n2 = inv_n * inv_n;
    const float inv_n3 = inv_n2 * inv_n;

    const float n2   = n * n;
    const float phi2 = phi * phi;

    const float inner = -kThrustCoeffX * phi2 * inv_n2 - kThrustCoeffY * phi * inv_n + kThrustCoeffZ;

    evalResult out{};

    // f
    out.f = inner * psi * n2 - thrustTarget;

    // df
    const float term1 = (2.0f * kThrustCoeffX * phi2 * inv_n3 + kThrustCoeffY * phi * inv_n2) * psi * n2;
    const float term2 = inner * 2.0f * psi * n;

    out.df = term1 + term2;

    return out;
}

float f(const float n, const float phi, const float psi, const float thrustTarget)
{
    return (-kThrustCoeffX*phi*phi/(n*n) - kThrustCoeffY*phi/n + kThrustCoeffZ) * psi*n*n - thrustTarget;
}

float df(const float n, const float phi, const float psi)
{
    return (2*kThrustCoeffX*phi*phi/(n*n*n) + kThrustCoeffY*phi/(n*n)) * psi*n*n + (-kThrustCoeffX*phi*phi/(n*n) - kThrustCoeffY*phi/n + kThrustCoeffZ) * 2*psi*n;
}

/*
 * Convert a desired thrust (Newton) at a given airspeed (m/s) to a rpm command
 * for an APC 16x8E propeller
 */
double thrust2rpm(const float airspeed, const float thrustTarget) {

    if (thrustTarget <= 2.0f)
        return 0.0;

    const float phi = 60*airspeed/kPropDiameter;
    constexpr float psi = kAirDensity*kPropDiameter*kPropDiameter*kPropDiameter*kPropDiameter/3600.0f;

    float x0 = 4000;        // initial guess about half range
    float x1 = 0;
    float res = 100;

    // Newton Raphson method
    int i = 0;
    while (std::fabs(res) > 10.0f && i++ < MAX_ITER_RPM)
    {
        const auto [f0, df0] = evalThrustModel(x0, phi, psi, thrustTarget);

        x1 = x0 - f0/df0;

        res = f(x1, phi, psi, thrustTarget);
        x0 = x1;
    }

    // Saturate desired RPM
    const float rpm = std::clamp(x0, 0.0f, 9000.0f);

    // Convert RPM to command between 0 and 1
    return rpm / 9000.0f;
}

double rpm2thrust(const float airspeed, const float rpmTarget) {
    const float rps = rpmTarget / 60.0f;
    const float airspeedOverRpsD = airspeed / (rps*kPropDiameter);

    return (-kThrustCoeffX * airspeedOverRpsD*airspeedOverRpsD - kThrustCoeffY * airspeedOverRpsD + kThrustCoeffZ) * kAirDensity * rps*rps * kPropDiameter*kPropDiameter*kPropDiameter*kPropDiameter;
}

float maxThrust(const float airspeed) {
    constexpr float maxRpm = 9000.0f;

    return static_cast<float>(rpm2thrust(airspeed, maxRpm));
}