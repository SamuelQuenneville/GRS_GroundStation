/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "powertrain.h"

inline evalResult evalThrustModel(const float n, const float x, const float y, const float z, const float phi, const float psi, const float thrustTarget) {
    const float inv_n  = 1.0f / n;
    const float inv_n2 = inv_n * inv_n;
    const float inv_n3 = inv_n2 * inv_n;

    const float n2   = n * n;
    const float phi2 = phi * phi;

    const float inner = -x * phi2 * inv_n2 - y * phi * inv_n + z;

    evalResult out{};

    // f
    out.f = inner * psi * n2 - thrustTarget;

    // df
    const float term1 = (2.0f * x * phi2 * inv_n3 + y * phi * inv_n2) * psi * n2;
    const float term2 = inner * 2.0f * psi * n;

    out.df = term1 + term2;

    return out;
}

float f(const float n, const float x, const float y, const float z, const float phi, const float psi, const float thrustTarget)
{
    return (-x*phi*phi/(n*n) - y*phi/n + z) * psi*n*n - thrustTarget;
}

float df(const float n, const float x, const float y, const float z, const float phi, const float psi)
{
    return (2*x*phi*phi/(n*n*n) + y*phi/(n*n)) * psi*n*n + (-x*phi*phi/(n*n) - y*phi/n + z) * 2*psi*n;
}

/*
 * Convert a desired thrust (Newton) at a given airspeed (m/s) to a rpm command
 * for an APC 16x8E propeller
 */
double thrust2rpm(const float airspeed, const float thrustTarget) {

    if (thrustTarget <= 2.0f)
        return 0.0;

    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter
    const float phi = 60*airspeed/D;
    constexpr float psi = rho*D*D*D*D/3600.0f;

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    float x0 = 4000;        // initial guess about half range
    float x1 = 0;
    float res = 100;

    // Newton Raphson method
    int i = 0;
    while (std::fabs(res) > 10.0f && i++ < MAX_ITER_RPM)
    {
        const auto [f0, df0] = evalThrustModel(x0, x, y, z, phi, psi, thrustTarget);

        x1 = x0 - f0/df0;

        res = f(x1,x,y,z,phi,psi,thrustTarget);
        x0 = x1;
    }

    // Saturate desired RPM
    const float rpm = std::clamp(x0, 0.0f, 9000.0f);

    // Convert RPM to command between 0 and 1
    return rpm / 9000.0f;
}

double rpm2thrust(const float airspeed, const float rpmTarget) {
    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter

    const float rps = rpmTarget / 60.0f;
    const float airspeedOverRpsD = airspeed / (rps*D);

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    return (-x * airspeedOverRpsD*airspeedOverRpsD - y * airspeedOverRpsD + z) * rho * rps*rps * D*D*D*D;
}

float maxThrust(const float airspeed) {
    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter

    constexpr float maxRps = 9000.0f / 60.0f;
    const float airspeedOverRpsD = airspeed / (maxRps*D);

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    return (-x * airspeedOverRpsD*airspeedOverRpsD - y * airspeedOverRpsD + z) * rho * maxRps*maxRps * D*D*D*D;
}