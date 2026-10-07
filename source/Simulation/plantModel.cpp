/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "plantModel.h"

namespace grs::sim {

namespace {

constexpr double kEps = 1e-6; // same eps as the MATLAB models

struct V3 {
    double x, y, z;
};

inline V3 operator+(const V3& a, const V3& b) { return {.x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z}; }
inline V3 operator-(const V3& a, const V3& b) { return {.x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z}; }
inline V3 operator*(const double s, const V3& a) { return {.x = s * a.x, .y = s * a.y, .z = s * a.z}; }
inline V3 operator/(const V3& a, const double s) { return {.x = a.x / s, .y = a.y / s, .z = a.z / s}; }
inline double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double sumsqr(const V3& a) { return dot(a, a); }
inline V3 cross(const V3& a, const V3& b) {
    return {.x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x};
}
inline V3 load(const double* p) { return {.x = p[0], .y = p[1], .z = p[2]}; }

// Thrust + aerodynamic + gravity + disturbance force on one UAV, and its
// angle of attack. xUav points at that UAV's 8-state block [p(3) v(3) roll
// pitch], thrust is its u(1). Disturbances: dFa along the airspeed and dCL
// on the lift coefficient (two-UAV model), dF a NED force (one-UAV model).
// Mirrors the per-UAV part of grs*DynamicAugmented.m line for line.
V3 uavForce(const AirframeParams& a, const RigParams& rig, const double* xUav, const double thrust,
            const V3& wind, const double dFa, const double dCL, const V3& dF, double& alpha) {
    const V3 v = load(xUav + 3);
    const double cphi = std::cos(xUav[6]);
    const double sphi = std::sin(xUav[6]);

    // alpha = pitch + gamma, gamma = atan2(vD, |v_horiz|) (see the model's
    // own comment on the sign convention)
    const double gamma = std::atan2(v.z, std::sqrt(v.x * v.x + v.y * v.y + kEps));
    alpha = xUav[7] + gamma;

    const V3 airspeed = v - wind;
    const double V2 = sumsqr(airspeed);
    const double V = std::sqrt(V2 + kEps);
    const V3 dir = airspeed / V;

    constexpr V3 down{.x = 0.0, .y = 0.0, .z = 1.0};
    const V3 eRight0 = cross(down, dir);
    const V3 eRight = eRight0 / std::sqrt(sumsqr(eRight0) + kEps);
    const V3 eUp = cross(eRight, dir);
    const V3 eLift = cphi * eUp + sphi * eRight;

    const double k = 1.0 / (M_PI * a.AR * a.e);
    const double qs = 0.5 * rig.rho * a.S;
    const double CL = a.CL0 + a.CL_alpha * alpha + dCL;
    const double CD = a.CD0 + k * CL * CL;
    const double L = qs * V2 * CL;
    const double D = qs * V2 * CD;

    const V3 gravity{.x = 0.0, .y = 0.0, .z = a.mass * rig.g};
    return (thrust - D + dFa) * dir + L * eLift + gravity + dF;
}

// Tether force on the UAV end (pull toward the other end), tanh-gated
// slack/taut spring + axial damper.
V3 tetherForce(const RigParams& rig, const V3& pRel, const V3& vRel, const double L0) {
    const double l = std::sqrt(sumsqr(pRel) + kEps);
    const V3 unit = pRel / l;
    const double dl = l - L0;
    const double gate = 1.0 + std::tanh(rig.tether_gate_gain * dl);
    const double kEff = 0.5 * rig.k_t * gate;
    return (-kEff * dl) * unit - (rig.b_t * dot(vRel, unit)) * unit;
}

void writeUavOde(const AirframeParams& a, const double* xUav, const V3& force, const double rollCmd,
                 const double pitchCmd, const double bRoll, const double bPitch, double* xdot) {
    xdot[0] = xUav[3];
    xdot[1] = xUav[4];
    xdot[2] = xUav[5];
    const double invM = 1.0 / a.mass;
    xdot[3] = force.x * invM;
    xdot[4] = force.y * invM;
    xdot[5] = force.z * invM;
    xdot[6] = (1.0 / a.T_roll) * (rollCmd + bRoll - xUav[6]);
    xdot[7] = (1.0 / a.T_pitch) * (pitchCmd + bPitch - xUav[7]);
}

} // namespace

TwoUavPayloadPlant::TwoUavPayloadPlant(const AirframeParams& uav1, const AirframeParams& uav2, const RigParams& rig)
    : m_uav{uav1, uav2}
    , m_rig(rig)
{
}

void TwoUavPayloadPlant::evaluate(const double* x, const double* u, const double* wind, const double* d, const double L0, double* xdot, double* alpha) const {
    const V3 w = load(wind);
    const double* x1 = x;        // UAV1 block
    const double* x2 = x + 8;    // UAV2 block
    const double* xp = x + 16;   // payload [p(3) v(3)]
    const V3 pPay = load(xp);
    const V3 vPay = load(xp + 3);

    // d = [dFa1 dCL1 b_roll1 b_pitch1  dFa2 dCL2 b_roll2 b_pitch2  dFz_pay]
    constexpr V3 none{.x = 0.0, .y = 0.0, .z = 0.0};
    double a1 = 0.0, a2 = 0.0;
    V3 f1 = uavForce(m_uav[0], m_rig, x1, u[0], w, d[0], d[1], none, a1);
    V3 f2 = uavForce(m_uav[1], m_rig, x2, u[3], w, d[4], d[5], none, a2);

    const V3 ft1 = tetherForce(m_rig, load(x1) - pPay, load(x1 + 3) - vPay, L0);
    const V3 ft2 = tetherForce(m_rig, load(x2) - pPay, load(x2 + 3) - vPay, L0);
    f1 = f1 + ft1;
    f2 = f2 + ft2;

    // Payload: gravity with its vertical disturbance, both tethers
    // (reaction), smooth one-sided ground contact gated on height.
    const double zPay = pPay.z;
    const double vzPay = vPay.z;
    const double gateGround = 0.5 * (1.0 + std::tanh(m_rig.kappa_ground * zPay));
    const double raw = m_rig.k_ground * zPay + m_rig.b_ground * vzPay;
    const double softplus = std::max(raw, 0.0) + std::log(1.0 + std::exp(-std::fabs(raw)));
    const V3 fGround{.x = 0.0, .y = 0.0, .z = -(gateGround * softplus)};
    const V3 fPay = V3{.x = 0.0, .y = 0.0, .z = m_rig.m_pay * m_rig.g + d[8]} - ft1 - ft2 + fGround;

    writeUavOde(m_uav[0], x1, f1, u[1], u[2], d[2], d[3], xdot);
    writeUavOde(m_uav[1], x2, f2, u[4], u[5], d[6], d[7], xdot + 8);

    const double invMPay = 1.0 / m_rig.m_pay;
    xdot[16] = vPay.x;
    xdot[17] = vPay.y;
    xdot[18] = vPay.z;
    xdot[19] = fPay.x * invMPay;
    xdot[20] = fPay.y * invMPay;
    xdot[21] = fPay.z * invMPay;

    if (alpha) {
        alpha[0] = a1;
        alpha[1] = a2;
    }
}

OneGroundPlant::OneGroundPlant(const AirframeParams& uav, const RigParams& rig)
    : m_uav(uav)
    , m_rig(rig)
{
}

void OneGroundPlant::evaluate(const double* x, const double* u, const double* wind, const double* d, const double L0, double* xdot, double* alpha) const {
    double a = 0.0;
    // d = [dFx dFy dFz b_roll b_pitch]
    V3 f = uavForce(m_uav, m_rig, x, u[0], load(wind), 0.0, 0.0, load(d), a);
    // Tether anchored at the NED origin.
    f = f + tetherForce(m_rig, load(x), load(x + 3), L0);
    writeUavOde(m_uav, x, f, u[1], u[2], d[3], d[4], xdot);
    if (alpha) alpha[0] = a;
}

IntegrationMethod parseIntegrationMethod(const std::string& name) {
    if (name == "rk4") return IntegrationMethod::RK4;
    if (name == "rk2") return IntegrationMethod::RK2;
    if (name == "euler") return IntegrationMethod::Euler;
    throw std::runtime_error("Unknown integration method '" + name + "' (rk4 | rk2 | euler)");
}

void integrate(const PlantModel& plant, std::vector<double>& x, const std::vector<double>& u, const std::vector<double>& wind, const std::vector<double>& d, const double L0, const double dt, const int nSub, const IntegrationMethod method) {
    const size_t n = x.size();
    const double h = dt / nSub;
    std::vector<double> k1(n), k2(n), k3(n), k4(n), tmp(n);

    auto f = [&](const std::vector<double>& xs, std::vector<double>& out) {
        plant.evaluate(xs.data(), u.data(), wind.data(), d.data(), L0, out.data(), nullptr);
    };

    for (int s = 0; s < nSub; ++s) {
        switch (method) {
            case IntegrationMethod::Euler:
                f(x, k1);
                for (size_t i = 0; i < n; ++i) x[i] = x[i] + h * k1[i];
                break;
            case IntegrationMethod::RK2:
                f(x, k1);
                for (size_t i = 0; i < n; ++i) tmp[i] = x[i] + h / 2 * k1[i];
                f(tmp, k2);
                for (size_t i = 0; i < n; ++i) x[i] = x[i] + h * k2[i];
                break;
            case IntegrationMethod::RK4:
                f(x, k1);
                for (size_t i = 0; i < n; ++i) tmp[i] = x[i] + h / 2 * k1[i];
                f(tmp, k2);
                for (size_t i = 0; i < n; ++i) tmp[i] = x[i] + h / 2 * k2[i];
                f(tmp, k3);
                for (size_t i = 0; i < n; ++i) tmp[i] = x[i] + h * k3[i];
                f(tmp, k4);
                for (size_t i = 0; i < n; ++i) x[i] = x[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
                break;
        }
    }
}

} // namespace grs::sim
