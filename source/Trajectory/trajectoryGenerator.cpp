/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "trajectoryGenerator.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <numeric>

namespace grs::trajgen {

namespace {

// ---------------------------------------------------------------------------
// Small numeric helpers (core/+math/*.m)
// ---------------------------------------------------------------------------

// core/+math/normalizeRows.m, single-row form.
Vec3d normalizeSafe(const Vec3d& v) {
    const double n = v.norm();
    const double denom = std::max(n, 1e-8);
    return v * (1.0 / denom);
}

// core/+math/phaseTime.m -- [dt, 2dt, ..., T] with T always included.
std::vector<double> phaseTime(double T, double dt) {
    if (T <= 0.0) return {};
    std::vector<double> tau;
    for (double t = dt; t <= T + 1e-12; t += dt) tau.push_back(t);
    if (tau.empty() || std::abs(tau.back() - T) > 1e-9) tau.push_back(T);
    return tau;
}

// Trapezoidal cumulative integral, matching MATLAB's cumtrapz(t, y) (scalar per sample).
std::vector<double> cumtrapzScalar(const std::vector<double>& t, const std::vector<double>& y) {
    std::vector<double> out(y.size(), 0.0);
    for (size_t i = 1; i < y.size(); ++i) {
        const double h = t[i] - t[i - 1];
        out[i] = out[i - 1] + 0.5 * h * (y[i - 1] + y[i]);
    }
    return out;
}

// MATLAB gradient(y, dt) on a uniformly-sampled vector: central difference on
// interior points, one-sided difference at the two ends.
std::vector<double> gradientUniform(const std::vector<double>& y, double dt) {
    const size_t n = y.size();
    std::vector<double> g(n, 0.0);
    if (n == 0) return g;
    if (n == 1) { g[0] = 0.0; return g; }
    g[0] = (y[1] - y[0]) / dt;
    g[n - 1] = (y[n - 1] - y[n - 2]) / dt;
    for (size_t i = 1; i + 1 < n; ++i) g[i] = (y[i + 1] - y[i - 1]) / (2.0 * dt);
    return g;
}

double quinticSmoothstep(double tau) { return 6*tau*tau*tau*tau*tau - 15*tau*tau*tau*tau + 10*tau*tau*tau; }
double quinticSmoothstepDeriv(double tau) { return 30*tau*tau*tau*tau - 60*tau*tau*tau + 30*tau*tau; }
// d^2/dtau^2 of quinticSmoothstep -- needed for the tether slip-clutch payout
// model's L''(t) (see generateAircraftTakeoff), nothing else uses it.
double quinticSmoothstepSecondDeriv(double tau) { return 120*tau*tau*tau - 180*tau*tau + 60*tau; }

// ---------------------------------------------------------------------------
// core/+traj/makeLoiter.m, sCurveProfile.m
// ---------------------------------------------------------------------------

struct Profile1D { std::vector<double> t, s, v, a; };

// core/+traj/makeLoiter.m
void appendLoiter(double t0, const Vec3d& xStart, double thold, double dt,
                   std::vector<double>& t, std::vector<KinematicSample>& samples, bool skipFirst) {
    const size_t n = static_cast<size_t>(std::floor(thold / dt + 1e-9)) + 1;
    for (size_t i = (skipFirst ? 1 : 0); i < n; ++i) {
        t.push_back(t0 + static_cast<double>(i) * dt);
        KinematicSample s;
        s.pos = xStart;
        samples.push_back(s);
    }
}

// core/+traj/sCurveProfile.m -- jerk-limited S-curve, 1D.
Profile1D sCurveProfile(double d, double vMax, double aMax, double jMax, double dt) {
    double tJ = aMax / jMax;
    double tA = vMax / aMax - tJ;

    if (tA < 0) {
        tJ = std::sqrt(vMax / jMax);
        tA = 0;
    }

    double dAcc = vMax*(tJ + tA) - aMax*(tJ*tJ + tA*(tJ + tA/2.0));
    const double dTotalMin = 2 * dAcc;
    double vPeak = vMax;
    double tV;

    if (dTotalMin > d) {
        tA = 0;
        tV = 0;
        vPeak = std::pow(d/2.0 * std::sqrt(jMax), 2.0/3.0);
        tJ = std::sqrt(vPeak / jMax);
        dAcc = d / 2.0;
    } else {
        tV = std::max(0.0, (d - 2*dAcc) / vPeak);
    }

    std::vector<double> t1 = {0.0};
    { auto p = phaseTime(tJ, dt); t1.insert(t1.end(), p.begin(), p.end()); }
    std::vector<double> t2 = phaseTime(tA, dt);
    std::vector<double> t3 = phaseTime(tJ, dt);
    std::vector<double> t4 = phaseTime(tV, dt);
    std::vector<double> t5 = phaseTime(tJ, dt);
    std::vector<double> t6 = phaseTime(tA, dt);
    std::vector<double> t7 = phaseTime(tJ, dt);

    const double T1 = 0, T2 = T1 + tJ, T3 = T2 + tA, T4 = T3 + tJ, T5 = T4 + tV, T6 = T5 + tJ, T7 = T6 + tA;

    Profile1D out;
    auto appendPhase = [&](const std::vector<double>& tp, double offset) {
        for (double v : tp) out.t.push_back(offset + v);
    };
    appendPhase(t1, T1); appendPhase(t2, T2); appendPhase(t3, T3);
    appendPhase(t4, T4); appendPhase(t5, T5); appendPhase(t6, T6); appendPhase(t7, T7);

    const size_t n = out.t.size();
    out.s.assign(n, 0.0); out.v.assign(n, 0.0); out.a.assign(n, 0.0);

    size_t e1 = t1.size(), e2 = e1 + t2.size(), e3 = e2 + t3.size();
    size_t e4 = e3 + t4.size(), e5 = e4 + t5.size(), e6 = e5 + t6.size(), e7 = e6 + t7.size();
    assert(e7 == n);

    // Phase 1: jerk up
    for (size_t i = 0; i < e1; ++i) {
        const double tau = t1[i];
        out.a[i] = jMax * tau;
        out.v[i] = 0.5 * jMax * tau*tau;
        out.s[i] = (1.0/6.0) * jMax * tau*tau*tau;
    }
    const double aE1 = e1 > 0 ? out.a[e1-1] : 0, vE1 = e1 > 0 ? out.v[e1-1] : 0, sE1 = e1 > 0 ? out.s[e1-1] : 0;
    (void)aE1;
    // Phase 2: constant acceleration
    for (size_t i = 0; i < t2.size(); ++i) {
        const double tau = t2[i]; const size_t idx = e1 + i;
        out.a[idx] = aMax;
        out.v[idx] = vE1 + aMax*tau;
        out.s[idx] = sE1 + vE1*tau + 0.5*aMax*tau*tau;
    }
    const double vE2 = e2 > e1 ? out.v[e2-1] : vE1, sE2 = e2 > e1 ? out.s[e2-1] : sE1;
    // Phase 3: jerk down
    for (size_t i = 0; i < t3.size(); ++i) {
        const double tau = t3[i]; const size_t idx = e2 + i;
        out.a[idx] = aMax - jMax*tau;
        out.v[idx] = vE2 + aMax*tau - 0.5*jMax*tau*tau;
        out.s[idx] = sE2 + vE2*tau + 0.5*aMax*tau*tau - (1.0/6.0)*jMax*tau*tau*tau;
    }
    const double vE3 = e3 > e2 ? out.v[e3-1] : vE2, sE3 = e3 > e2 ? out.s[e3-1] : sE2;
    // Phase 4: constant velocity
    for (size_t i = 0; i < t4.size(); ++i) {
        const double tau = t4[i]; const size_t idx = e3 + i;
        out.a[idx] = 0;
        out.v[idx] = vE3;
        out.s[idx] = sE3 + vE3*tau;
    }
    const double vE4 = e4 > e3 ? out.v[e4-1] : vE3, sE4 = e4 > e3 ? out.s[e4-1] : sE3;
    // Phase 5: jerk negative
    for (size_t i = 0; i < t5.size(); ++i) {
        const double tau = t5[i]; const size_t idx = e4 + i;
        out.a[idx] = -jMax*tau;
        out.v[idx] = vE4 - 0.5*jMax*tau*tau;
        out.s[idx] = sE4 + vE4*tau - (1.0/6.0)*jMax*tau*tau*tau;
    }
    const double vE5 = e5 > e4 ? out.v[e5-1] : vE4, sE5 = e5 > e4 ? out.s[e5-1] : sE4;
    // Phase 6: constant deceleration
    for (size_t i = 0; i < t6.size(); ++i) {
        const double tau = t6[i]; const size_t idx = e5 + i;
        out.a[idx] = -aMax;
        out.v[idx] = vE5 - aMax*tau;
        out.s[idx] = sE5 + vE5*tau - 0.5*aMax*tau*tau;
    }
    const double vE6 = e6 > e5 ? out.v[e6-1] : vE5, sE6 = e6 > e5 ? out.s[e6-1] : sE5;
    // Phase 7: jerk to zero
    for (size_t i = 0; i < t7.size(); ++i) {
        const double tau = t7[i]; const size_t idx = e6 + i;
        out.a[idx] = -aMax + jMax*tau;
        out.v[idx] = vE6 - aMax*tau + 0.5*jMax*tau*tau;
        out.s[idx] = sE6 + vE6*tau - 0.5*aMax*tau*tau + (1.0/6.0)*jMax*tau*tau*tau;
    }

    return out;
}

// ---------------------------------------------------------------------------
// core/+traj/generatePayloadPath.m (mission phases only -- pre-appendTakeoff)
// ---------------------------------------------------------------------------

struct PayloadMission { std::vector<double> t; std::vector<KinematicSample> samples; };

PayloadMission generatePayloadPath(const TrajectoryConfig& config) {
    const auto& p = config.payloadPath;
    const double dt = config.simDt;

    PayloadMission out;

    // 1) Initial hold
    appendLoiter(0.0, p.x0, p.timehold, dt, out.t, out.samples, false);

    // 2) Climb
    const Profile1D climb = sCurveProfile(p.distanceClimb, p.velClimb, p.accClimb, 0.08, dt);
    const double t1End = out.t.back();
    const Vec3d posAtHoldEnd = out.samples.back().pos;
    for (size_t i = 1; i < climb.t.size(); ++i) {
        out.t.push_back(t1End + climb.t[i]);
        KinematicSample s;
        s.pos = Vec3d{posAtHoldEnd[0], posAtHoldEnd[1], posAtHoldEnd[2] - climb.s[i]};
        s.vel = Vec3d{0, 0, -climb.v[i]};
        s.acc = Vec3d{0, 0, -climb.a[i]};
        out.samples.push_back(s);
    }

    // 3) Hold
    appendLoiter(out.t.back(), out.samples.back().pos, p.timehold, dt, out.t, out.samples, true);

    // 4) Horizontal move
    const double angleMove = grs::degToRad(p.angleMoveDeg);
    const Vec3d dir{std::cos(angleMove), std::sin(angleMove), 0.0};
    const Profile1D move = sCurveProfile(p.distanceMove, p.velMove, p.accMove, 0.05, dt);
    const double t3End = out.t.back();
    const Vec3d posAtHold3End = out.samples.back().pos;
    for (size_t i = 1; i < move.t.size(); ++i) {
        out.t.push_back(t3End + move.t[i]);
        KinematicSample s;
        s.pos = posAtHold3End + dir * move.s[i];
        s.vel = dir * move.v[i];
        s.acc = dir * move.a[i];
        out.samples.push_back(s);
    }

    // 5) Hold
    appendLoiter(out.t.back(), out.samples.back().pos, p.timehold, dt, out.t, out.samples, true);

    // 6) Declimb
    const Profile1D declimb = sCurveProfile(p.distanceClimb, p.velClimb, p.accClimb, 0.08, dt);
    const double t5End = out.t.back();
    const Vec3d posAtHold5End = out.samples.back().pos;
    for (size_t i = 1; i < declimb.t.size(); ++i) {
        out.t.push_back(t5End + declimb.t[i]);
        KinematicSample s;
        s.pos = Vec3d{posAtHold5End[0], posAtHold5End[1], posAtHold5End[2] + declimb.s[i]};
        s.vel = Vec3d{0, 0, declimb.v[i]};
        s.acc = Vec3d{0, 0, declimb.a[i]};
        out.samples.push_back(s);
    }

    // 7) Final hold
    appendLoiter(out.t.back(), out.samples.back().pos, p.timehold, dt, out.t, out.samples, true);

    return out;
}

// ---------------------------------------------------------------------------
// core/+traj/generateAircraftTakeoff.m
// ---------------------------------------------------------------------------

struct TakeoffMission {
    std::vector<double> t;
    std::vector<double> gamma, psi;
    std::vector<KinematicSample> payloadFrame; // relative to launcher (pre-x0)
    std::vector<KinematicSample> inertial;     // + config.payloadPath.x0
};

TakeoffMission generateAircraftTakeoff(const TrajectoryConfig& config, double phase) {
    const auto& ap = config.aircraftPath;
    const double Lfinal = config.tether.length;
    const double dt = config.simDt;
    const double T = ap.takeoffTime;

    // ---- Slip-clutch tether payout (see TrajectoryConfig::Tether comment) ----
    // Scope of this model, deliberately: the angular guidance law below
    // (theta/thetaDot/thetaDdot/psi/gamma -- the boundary-value-matched pitch
    // profile and the resulting azimuth-rate ODE) is left exactly as
    // validated against MATLAB, computed with `Lfinal` throughout, EXCEPT for
    // theta0/the phase-1 rate formulas just below, which use the true
    // launch-instant tether length `L0` since they describe the aircraft's
    // actual physical elevation angle/rate right at t=0 -- using `Lfinal`
    // there would be geometrically wrong the moment lengthAtLaunch != length.
    // The payout itself (i.e. the tether's radial length actually changing
    // over time) is applied only when converting the resulting spherical
    // path (L(t), theta(t), psi(t)) to Cartesian position/velocity/
    // acceleration below, via the ordinary product rule on L(t) -- a real
    // outward-radial motion layered on top of the unchanged angular path,
    // not a re-solve of the underlying boundary-value problem.
    const double L0 = config.tether.lengthAtLaunch; // finalize() resolves <0 to Lfinal
    const double Tpo = std::max(config.tether.payoutDurationSeconds, 0.0);
    const double dL = Lfinal - L0;
    auto payoutTau = [&](double time) { return Tpo > 1e-9 ? std::clamp(time / Tpo, 0.0, 1.0) : 1.0; };
    auto lengthAt = [&](double time) { return L0 + dL * quinticSmoothstep(payoutTau(time)); };
    auto lengthDotAt = [&](double time) {
        return (Tpo > 1e-9 && time < Tpo) ? dL * quinticSmoothstepDeriv(payoutTau(time)) / Tpo : 0.0;
    };
    auto lengthDdotAt = [&](double time) {
        return (Tpo > 1e-9 && time < Tpo) ? dL * quinticSmoothstepSecondDeriv(payoutTau(time)) / (Tpo * Tpo) : 0.0;
    };

    const size_t n = static_cast<size_t>(std::floor(T / dt + 1e-9)) + 1;
    std::vector<double> t(n);
    for (size_t i = 0; i < n; ++i) t[i] = i * dt;

    const double V0 = ap.takeoffVel0;
    const double Vf = ap.velMean;
    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) {
        const double tauV = std::min(t[i] / ap.takeoffTimeAcc, 1.0);
        v[i] = V0 + (Vf - V0) * quinticSmoothstep(tauV);
    }

    const double t1 = ap.takeoffTimeBalistic;
    const double t2 = T - t1;

    std::vector<double> gamma(n, 0.0), theta(n, 0.0), thetaDot(n, 0.0), thetaDdot(n, 0.0);

    const double gamma0 = grs::degToRad(ap.takeoffPitch0Deg) + ap.takeoffPitch0TrimRad;
    const double gammaRelax = ap.takeoffPitchDecayFrac * gamma0;

    const double theta0 = std::asin(ap.z0 / L0);
    const double zTarget = std::sqrt(Lfinal*Lfinal - ap.radius*ap.radius);
    const double thetaF = std::asin(zTarget / Lfinal);

    size_t iSwitch = 0;
    for (size_t i = 0; i < n; ++i) if (t[i] <= t1) iSwitch = i;

    // Phase 1
    std::vector<double> thetaDotPhase1(iSwitch + 1), tPhase1(iSwitch + 1);
    for (size_t i = 0; i <= iSwitch; ++i) {
        const double tau1 = t[i] / t1;
        const double s1 = quinticSmoothstep(tau1);
        gamma[i] = gamma0 + (gammaRelax - gamma0) * s1;
        const double ds1 = quinticSmoothstepDeriv(tau1);
        thetaDotPhase1[i] = (v[i] * std::sin(gamma[i])) / (L0 * std::max(std::cos(theta0), 1e-6));
        thetaDdot[i] = ((v[i] * (gammaRelax - gamma0) * ds1) / t1) / (L0 * std::max(std::cos(theta0), 1e-6));
        tPhase1[i] = t[i];
    }
    const std::vector<double> thetaPhase1 = [&]{
        auto cum = cumtrapzScalar(tPhase1, thetaDotPhase1);
        for (auto& c : cum) c += theta0;
        return cum;
    }();
    for (size_t i = 0; i <= iSwitch; ++i) { theta[i] = thetaPhase1[i]; thetaDot[i] = thetaDotPhase1[i]; }

    // Phase 2: 7th-order polynomial matching position/vel/accel/jerk at the boundary
    const double theta1 = thetaPhase1[iSwitch];
    const double thetaDot1 = thetaDotPhase1[iSwitch];
    const double thetaDdot1 = thetaDdot[iSwitch];
    const size_t prevIdx = iSwitch > 0 ? iSwitch - 1 : 0;
    const double thetaJerk1 = (thetaDdot[iSwitch] - thetaDdot[prevIdx]) / dt;

    const double b0 = theta1;
    const double b1 = thetaDot1 * t2;
    const double b2 = 0.5 * thetaDdot1 * t2 * t2;
    const double b3 = (1.0/6.0) * thetaJerk1 * t2*t2*t2;

    // A * [b4 b5 b6 b7]' = rhs  (4x4 solve, closed-form via Cramer-free Gaussian elimination)
    double A[4][4] = {
        {1,1,1,1},
        {4,5,6,7},
        {12,20,30,42},
        {24,60,120,210}
    };
    double rhs[4] = {
        thetaF - (b0+b1+b2+b3),
        0.0 - (b1 + 2*b2 + 3*b3),
        0.0 - (2*b2 + 6*b3),
        0.0 - (6*b3)
    };
    // Gaussian elimination with partial pivoting, 4x4.
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        for (int r = col+1; r < 4; ++r) if (std::abs(A[r][col]) > std::abs(A[piv][col])) piv = r;
        if (piv != col) { std::swap(A[piv], A[col]); std::swap(rhs[piv], rhs[col]); }
        for (int r = col+1; r < 4; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int c = col; c < 4; ++c) A[r][c] -= f * A[col][c];
            rhs[r] -= f * rhs[col];
        }
    }
    double b4567[4];
    for (int r = 3; r >= 0; --r) {
        double sum = rhs[r];
        for (int c = r+1; c < 4; ++c) sum -= A[r][c] * b4567[c];
        b4567[r] = sum / A[r][r];
    }
    const double b4 = b4567[0], b5 = b4567[1], b6 = b4567[2], b7 = b4567[3];

    for (size_t i = iSwitch + 1; i < n; ++i) {
        const double tau2 = (t[i] - t1) / t2;
        const double tau2_2 = tau2*tau2, tau2_3 = tau2_2*tau2, tau2_4 = tau2_3*tau2, tau2_5 = tau2_4*tau2, tau2_6 = tau2_5*tau2;
        theta[i] = b0 + b1*tau2 + b2*tau2_2 + b3*tau2_3 + b4*tau2_4 + b5*tau2_5 + b6*tau2_6 + b7*tau2_6*tau2;
        thetaDot[i] = (b1 + 2*b2*tau2 + 3*b3*tau2_2 + 4*b4*tau2_3 + 5*b5*tau2_4 + 6*b6*tau2_5 + 7*b7*tau2_6) / t2;
        thetaDdot[i] = (2*b2 + 6*b3*tau2 + 12*b4*tau2_2 + 20*b5*tau2_3 + 30*b6*tau2_4 + 42*b7*tau2_5) / (t2*t2);
        const double ratio = (Lfinal * std::cos(theta[i]) * thetaDot[i]) / std::max(v[i], 1e-6);
        gamma[i] = std::asin(std::clamp(ratio, -1.0, 1.0));
    }

    // Azimuth
    std::vector<double> psiDot(n), psiDdotSrc(n);
    for (size_t i = 0; i < n; ++i) {
        const double vOverL = v[i] / Lfinal;
        const double under = std::max(vOverL*vOverL - thetaDot[i]*thetaDot[i], 0.0);
        psiDot[i] = config.aircraftPath.direction * std::sqrt(under) / std::max(std::cos(theta[i]), 1e-6);
    }
    const std::vector<double> psiDdot = gradientUniform(psiDot, dt);
    const std::vector<double> psiCum = cumtrapzScalar(t, psiDot);
    std::vector<double> psi(n);
    for (size_t i = 0; i < n; ++i) psi[i] = phase + psiCum[i];

    TakeoffMission out;
    out.t = t; out.gamma = gamma; out.psi = psi;
    out.payloadFrame.resize(n);
    out.inertial.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const double cTheta = std::cos(theta[i]), sTheta = std::sin(theta[i]);
        const double cPsi = std::cos(psi[i]), sPsi = std::sin(psi[i]);

        // Tether length at this sample and its time derivatives -- L0 (or
        // Lfinal, whichever, since either way this collapses to a constant
        // Lfinal with LDot=LDdot=0) whenever no payout is configured, making
        // this an exact no-op vs. the original constant-L formulas below.
        const double Lt = lengthAt(t[i]);
        const double LDot = lengthDotAt(t[i]);
        const double LDdot = lengthDdotAt(t[i]);

        // Unit-tether-length (L=1) angular kinematics -- everything below is
        // just L(t)*u + 2*L'(t)*uDot + L''(t)*u for the position, and the
        // original per-axis bracket (unchanged) is exactly L''(t)*u's
        // "acceleration of a unit vector" term, i.e. ordinary product rule
        // applied to pos(t) = L(t) * u(theta(t), psi(t)).
        const double x = cTheta*cPsi, y = cTheta*sPsi, z = -sTheta;
        const double xDot = -sTheta*thetaDot[i]*cPsi - cTheta*sPsi*psiDot[i];
        const double yDot = -sTheta*thetaDot[i]*sPsi + cTheta*cPsi*psiDot[i];
        const double zDot = -(cTheta*thetaDot[i]);
        const double xDdot =
            -cTheta*thetaDot[i]*thetaDot[i]*cPsi
            -sTheta*thetaDdot[i]*cPsi
            +2*sTheta*thetaDot[i]*sPsi*psiDot[i]
            -cTheta*cPsi*psiDot[i]*psiDot[i]
            -cTheta*sPsi*psiDdot[i];
        const double yDdot =
            -cTheta*thetaDot[i]*thetaDot[i]*sPsi
            -sTheta*thetaDdot[i]*sPsi
            -2*sTheta*thetaDot[i]*cPsi*psiDot[i]
            -cTheta*sPsi*psiDot[i]*psiDot[i]
            +cTheta*cPsi*psiDdot[i];
        const double zDdot = -(-sTheta*thetaDot[i]*thetaDot[i] + cTheta*thetaDdot[i]);

        out.payloadFrame[i].pos = { Lt*x, Lt*y, Lt*z };
        out.payloadFrame[i].vel = {
            LDot*x + Lt*xDot,
            LDot*y + Lt*yDot,
            LDot*z + Lt*zDot,
        };
        out.payloadFrame[i].acc = {
            LDdot*x + 2*LDot*xDot + Lt*xDdot,
            LDdot*y + 2*LDot*yDot + Lt*yDdot,
            LDdot*z + 2*LDot*zDot + Lt*zDdot,
        };

        out.inertial[i].pos = out.payloadFrame[i].pos + config.payloadPath.x0;
        out.inertial[i].vel = out.payloadFrame[i].vel;
        out.inertial[i].acc = out.payloadFrame[i].acc;
    }

    return out;
}

// ---------------------------------------------------------------------------
// core/+traj/generateAircraftPath.m (mission phase, uses payloadMission.t)
// ---------------------------------------------------------------------------

struct AircraftMissionRaw { AircraftTimeline timeline; TakeoffMission takeoff; };

AircraftMissionRaw generateAircraftPathForUav(const TrajectoryConfig& config, double phase, const PayloadMission& payloadMission) {
    const auto& ap = config.aircraftPath;
    const double R = ap.radius;
    const double omega = ap.direction * ap.velMean / R;
    const double zpay = -std::sqrt(config.tether.length*config.tether.length - R*R);

    AircraftMissionRaw out;
    out.takeoff = generateAircraftTakeoff(config, phase);
    const double psiEnd = out.takeoff.psi.back();

    const size_t n = payloadMission.t.size();
    out.timeline.payloadFrame.resize(n);
    out.timeline.inertial.resize(n);

    for (size_t i = 0; i < n; ++i) {
        const double theta = omega * payloadMission.t[i] + psiEnd;
        const double xPay = R*std::cos(theta), yPay = R*std::sin(theta);
        const double vxPay = -R*omega*std::sin(theta), vyPay = R*omega*std::cos(theta);
        const double axPay = -R*omega*omega*std::cos(theta), ayPay = -R*omega*omega*std::sin(theta);

        KinematicSample rel;
        rel.pos = {xPay, yPay, zpay};
        rel.vel = {vxPay, vyPay, 0.0};
        rel.acc = {axPay, ayPay, 0.0};
        out.timeline.payloadFrame[i] = rel;

        KinematicSample inertial;
        inertial.pos = payloadMission.samples[i].pos + rel.pos;
        inertial.vel = payloadMission.samples[i].vel + rel.vel;
        inertial.acc = payloadMission.samples[i].acc + rel.acc;
        out.timeline.inertial[i] = inertial;
    }

    return out;
}

// ---------------------------------------------------------------------------
// core/+dyn/computeForcesOnPayload.m
// ---------------------------------------------------------------------------

std::vector<Vec3d> computeForcesOnPayload(const TrajectoryConfig& config, const PayloadMission& payloadMission) {
    std::vector<Vec3d> fTotal(payloadMission.samples.size(), Vec3d::zeros());
    for (size_t i = 0; i < fTotal.size(); ++i) {
        const Vec3d vAir = payloadMission.samples[i].vel - config.world.wind;
        const double vAirNorm = vAir.norm();
        const Vec3d fDrag = vAir * (-0.5 * config.world.airDensity * config.payload.CdA * vAirNorm);
        const Vec3d fGravity{0, 0, config.payload.mass * config.world.gravity};
        fTotal[i] = fDrag + fGravity;
    }
    return fTotal;
}

// ---------------------------------------------------------------------------
// core/+dyn/computeForcesOnTethers.m -- static (time-invariant): gravity only.
// ---------------------------------------------------------------------------

struct TetherStaticForces { std::vector<Vec3d> onSegments; Vec3d total = Vec3d::zeros(); };

TetherStaticForces computeForcesOnTethers(const TrajectoryConfig& config) {
    TetherStaticForces f;
    f.onSegments.assign(config.tether.nSegments, Vec3d::zeros());
    for (int i = 0; i < config.tether.nSegments; ++i) {
        f.onSegments[i] = Vec3d{0, 0, config.tether.segmentMass[i] * config.world.gravity};
    }
    Vec3d total = Vec3d::zeros();
    for (const auto& s : f.onSegments) total += s;
    f.total = total;
    return f;
}

// ---------------------------------------------------------------------------
// Dense linear solve (Gaussian elimination, partial pivoting) -- needed for
// core/+dyn/solveTetherForcesTwoAircraft.m's 15-unknown linear system, which
// exceeds grs::Matrix's 4x4 cap.
// ---------------------------------------------------------------------------

template <size_t N>
std::array<double, N> solveLinearSystem(std::array<std::array<double, N>, N> A, std::array<double, N> b) {
    for (size_t col = 0; col < N; ++col) {
        size_t piv = col;
        for (size_t r = col + 1; r < N; ++r) if (std::abs(A[r][col]) > std::abs(A[piv][col])) piv = r;
        if (piv != col) { std::swap(A[piv], A[col]); std::swap(b[piv], b[col]); }
        for (size_t r = col + 1; r < N; ++r) {
            const double f = A[r][col] / A[col][col];
            for (size_t c = col; c < N; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    std::array<double, N> x{};
    for (size_t ri = 0; ri < N; ++ri) {
        const size_t r = N - 1 - ri;
        double sum = b[r];
        for (size_t c = r + 1; c < N; ++c) sum -= A[r][c] * x[c];
        x[r] = sum / A[r][r];
    }
    return x;
}

// ---------------------------------------------------------------------------
// core/+dyn/solveTetherForcesTwoAircraft.m
// ---------------------------------------------------------------------------

struct TetherForcePair { Vec3d f1 = Vec3d::zeros(), f2 = Vec3d::zeros(); double stabilization = 0.0; };

TetherForcePair solveTetherForcesTwoAircraft(const TrajectoryConfig& config, const TetherStaticForces& staticForces,
                                              const Vec3d& fOnPayload, const Vec3d& payloadPos, const Vec3d& payloadAcc,
                                              const Vec3d& ac1Pos, const Vec3d& ac2Pos) {
    const Vec3d a1 = ac1Pos - payloadPos;
    const Vec3d a2 = ac2Pos - payloadPos;

    const Vec3d v = cross(a2, a1);
    const Vec3d vUnit = normalizeSafe(v);

    constexpr Vec3d north{1, 0, 0}, east{0, 1, 0}, down{0, 0, -1};

    std::array<std::array<double, 15>, 15> A{};
    std::array<double, 15> B{};

    // A1 (rows 0-2) -- sum of forces on the pin between payload and tethers.
    const double vN = dot(vUnit, north), vE = dot(vUnit, east), vD = dot(vUnit, down);
    A[0][1] = 1; A[0][8]  = 1; A[0][14] = vN;
    A[1][3] = 1; A[1][10] = 1; A[1][14] = vE;
    A[2][5] = 1; A[2][12] = 1; A[2][14] = vD;

    const double totalTetherMass = 2.0 * std::accumulate(config.tether.segmentMass.begin(), config.tether.segmentMass.end(), 0.0);
    const Vec3d fP = payloadAcc * (config.payload.mass + 0.5 * totalTetherMass) - fOnPayload;
    B[0] = -fP[0]; B[1] = -fP[1]; B[2] = -fP[2];

    // A2 (per tether: sum of forces) -- rows 3-5 (tether1), rows 9-11 (tether2)
    auto fillA2 = [&](int rowBase, int colBase) {
        A[rowBase+0][colBase+0] = 1; A[rowBase+0][colBase+1] = 1;
        A[rowBase+1][colBase+2] = 1; A[rowBase+1][colBase+3] = 1;
        A[rowBase+2][colBase+4] = 1; A[rowBase+2][colBase+5] = 1;
    };
    fillA2(3, 0);
    fillA2(9, 7);

    B[3] = staticForces.total[0]; B[4] = staticForces.total[1]; B[5] = staticForces.total[2];
    B[9] = staticForces.total[0]; B[10] = staticForces.total[1]; B[11] = staticForces.total[2];

    // A3 (per tether: sum of moments about the tip) -- rows 6-8, 12-14.
    auto fillA3AndB3 = [&](int rowBase, int colBase, const Vec3d& armVec) {
        const Vec3d segOrient = normalizeSafe(armVec);
        A[rowBase+0][colBase+3] = armVec[2];  A[rowBase+0][colBase+5] = -armVec[1]; A[rowBase+0][colBase+6] = segOrient[0];
        A[rowBase+1][colBase+1] = -armVec[2]; A[rowBase+1][colBase+5] = armVec[0];  A[rowBase+1][colBase+6] = segOrient[1];
        A[rowBase+2][colBase+1] = armVec[1];  A[rowBase+2][colBase+3] = -armVec[0]; A[rowBase+2][colBase+6] = segOrient[2];

        Vec3d mExtSum = Vec3d::zeros();
        for (int i = 0; i < config.tether.nSegments; ++i) {
            const Vec3d arm = armVec * config.tether.segmentLinCoordNorm[i];
            mExtSum += cross(arm, staticForces.onSegments[i]);
        }
        B[rowBase+0] = mExtSum[0]; B[rowBase+1] = mExtSum[1]; B[rowBase+2] = mExtSum[2];
    };
    fillA3AndB3(6, 0, a1);
    fillA3AndB3(12, 7, a2);

    const auto x = solveLinearSystem<15>(A, B);

    TetherForcePair out;
    out.f1 = Vec3d{-x[0], -x[2], -x[4]};
    out.f2 = Vec3d{-x[7], -x[9], -x[11]};
    out.stabilization = x[14];
    return out;
}

// ---------------------------------------------------------------------------
// core/+dyn/aeroFromLift.m, computeAngleOfAttackThrust.m, inverseDynamic.m
// ---------------------------------------------------------------------------

struct AeroFromLift { double CL, CD, drag, angleOfAttack; };

AeroFromLift aeroFromLift(const TrajectoryConfig& config, double lift, double airspeed) {
    const double q_S = 0.5 * config.world.airDensity * airspeed * airspeed * config.aircraft.wingArea;
    AeroFromLift out{};
    out.CL = lift / q_S;
    const double K = 1.0 / (M_PI * config.aircraft.aspectRatio * config.aircraft.oswald);
    out.CD = config.aircraft.CD0 + K * out.CL * out.CL;
    out.drag = q_S * out.CD;
    out.angleOfAttack = (out.CL - config.aircraft.CL0) / config.aircraft.CLalpha;
    return out;
}

struct AngleOfAttackThrust { double angleOfAttack = 0.0, thrust = 0.0; Vec3d liftDir = Vec3d::zeros(); };

AngleOfAttackThrust computeAngleOfAttackThrust(const TrajectoryConfig& config, const Vec3d& vel, const Vec3d& acc, const Vec3d& forceTether) {
    const Vec3d weight{0, 0, config.aircraft.mass * config.world.gravity};
    const Vec3d fAero = acc * config.aircraft.mass - forceTether - weight;

    const Vec3d freestream = vel + config.world.wind;
    const Vec3d freestreamUnit = normalizeSafe(freestream);

    const double f1 = dot(fAero, freestreamUnit);
    const Vec3d f1vec = freestreamUnit * f1;
    const Vec3d f2 = fAero - f1vec;
    const double lift = std::max(f2.norm(), 1e-8);
    const Vec3d liftDir = f2 * (1.0 / lift);

    const auto aero = aeroFromLift(config, lift, freestream.norm());

    AngleOfAttackThrust out{};
    out.angleOfAttack = aero.angleOfAttack;
    out.thrust = aero.drag + f1;
    out.liftDir = liftDir;
    return out;
}

ControlSample inverseDynamic(const TrajectoryConfig& config, const KinematicSample& sample, const Vec3d& externalForce) {
    const auto aoaThrust = computeAngleOfAttackThrust(config, sample.vel, sample.acc, externalForce);

    const Vec3d freestream = sample.vel + config.world.wind;
    const Vec3d freestreamUnit = normalizeSafe(freestream);

    constexpr Vec3d zWorld{0, 0, 1};
    Vec3d latAxis = cross(zWorld, freestreamUnit);
    latAxis = latAxis * (1.0 / std::max(latAxis.norm(), 1e-8));
    const Vec3d liftProjPlane = cross(latAxis, freestreamUnit);

    const Vec3d bankCross = cross(aoaThrust.liftDir, liftProjPlane);
    double bank = std::asin(std::clamp(bankCross.norm(), -1.0, 1.0));
    const double signBank = dot(bankCross, freestreamUnit) >= 0 ? 1.0 : -1.0;
    bank *= signBank;

    const double vXy = std::hypot(freestream[0], freestream[1]);
    const double pathAngle = -std::atan2(freestream[2], vXy);

    ControlSample out;
    out.yawRad = std::atan2(freestream[1], freestream[0]);
    out.pitchRad = aoaThrust.angleOfAttack + pathAngle;
    out.rollRad = -bank;
    out.thrustNewton = aoaThrust.thrust;
    out.angleOfAttackRad = aoaThrust.angleOfAttack;
    out.liftDirection = aoaThrust.liftDir;
    return out;
}

// ---------------------------------------------------------------------------
// core/+dyn/computeControlsTakeoff.m
// ---------------------------------------------------------------------------

std::vector<ControlSample> computeControlsTakeoff(const TrajectoryConfig& config, double rollLoiter, const TakeoffMission& takeoff) {
    const auto& ap = config.aircraftPath;
    const double T = ap.takeoffTime;
    const size_t n = takeoff.t.size();

    const double zTarget = std::sqrt(config.tether.length*config.tether.length - ap.radius*ap.radius);
    const double fTarget = (0.5 * config.payload.mass * config.world.gravity) / (zTarget / config.tether.length);

    constexpr double tDelay = 2.0, tRamp = 6.0;

    std::vector<ControlSample> out(n);
    for (size_t i = 0; i < n; ++i) {
        const double t = takeoff.t[i];
        const double tau = std::clamp(t / T, 0.0, 1.0);
        const double s = quinticSmoothstep(tau);
        const double phi = (1 - s) * grs::degToRad(ap.takeoffRoll0Deg) + s * rollLoiter;

        const double tauF = std::clamp((t - tDelay) / tRamp, 0.0, 1.0);
        const double sF = quinticSmoothstep(tauF);
        const double F = (0.1 + 0.9 * sF) * fTarget;

        Vec3d et = config.payloadPath.x0 - takeoff.inertial[i].pos;
        et = normalizeSafe(et);
        const Vec3d externalForce = et * F;

        const auto aoaThrust = computeAngleOfAttackThrust(config, takeoff.inertial[i].vel, takeoff.inertial[i].acc, externalForce);

        ControlSample& c = out[i];
        c.rollRad = phi;
        c.pitchRad = aoaThrust.angleOfAttack + takeoff.gamma[i];
        c.yawRad = takeoff.psi[i];
        c.thrustNewton = aoaThrust.thrust;
        c.angleOfAttackRad = aoaThrust.angleOfAttack;
        c.liftDirection = aoaThrust.liftDir;
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// TrajectoryGenerator
// ---------------------------------------------------------------------------

TrajectoryGenerator::TrajectoryGenerator(TrajectoryConfig config) : m_config(std::move(config)) {
    m_config.finalize();
}

GeneratedMission TrajectoryGenerator::generate() const {
    // apps/main.m, in order.
    const PayloadMission payloadMission = generatePayloadPath(m_config);

    const size_t nUavs = m_config.aircraftPath.phaseRad.size();
    std::vector<AircraftMissionRaw> aircraftRaw(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        aircraftRaw[k] = generateAircraftPathForUav(m_config, m_config.aircraftPath.phaseRad[k], payloadMission);
    }

    // dyn.getTethersForces: static per-tether gravity + per-sample tether pull at each aircraft.
    const TetherStaticForces staticForces = computeForcesOnTethers(m_config);
    const std::vector<Vec3d> fOnPayload = computeForcesOnPayload(m_config, payloadMission);

    assert(nUavs == 2 && "Tether force solve is hard-coded for two aircraft, matching the MATLAB source.");

    const size_t nMission = payloadMission.t.size();
    std::vector<Vec3d> tetherF1(nMission, Vec3d::zeros()), tetherF2(nMission, Vec3d::zeros());
    for (size_t i = 0; i < nMission; ++i) {
        const auto pair = solveTetherForcesTwoAircraft(
            m_config, staticForces, fOnPayload[i], payloadMission.samples[i].pos, payloadMission.samples[i].acc,
            aircraftRaw[0].timeline.inertial[i].pos, aircraftRaw[1].timeline.inertial[i].pos);
        tetherF1[i] = pair.f1;
        tetherF2[i] = pair.f2;
    }

    // dyn.getAircraftControls: mission-phase inverse dynamics, then prepend takeoff controls.
    std::vector<std::vector<ControlSample>> missionControls(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        missionControls[k].resize(nMission);
        const std::vector<Vec3d>& forces = (k == 0) ? tetherF1 : tetherF2;
        for (size_t i = 0; i < nMission; ++i) {
            missionControls[k][i] = inverseDynamic(m_config, aircraftRaw[k].timeline.inertial[i], forces[i]);
        }
    }
    const double rollLoiter = missionControls[0].front().rollRad;

    std::vector<std::vector<ControlSample>> takeoffControls(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        takeoffControls[k] = computeControlsTakeoff(m_config, rollLoiter, aircraftRaw[k].takeoff);
    }

    // traj.appendTakeoff: prepend takeoff kinematics to each aircraft, pad payload with zeros.
    GeneratedMission out;
    const size_t nTakeoff = aircraftRaw[0].takeoff.t.size();
    const size_t nTotal = nTakeoff + nMission;

    out.time.resize(nTotal);
    for (size_t i = 0; i < nTakeoff; ++i) out.time[i] = i * m_config.simDt;
    for (size_t i = 0; i < nMission; ++i) out.time[nTakeoff + i] = payloadMission.t[i] + m_config.aircraftPath.takeoffTime + m_config.simDt;

    out.payload.assign(nTotal, KinematicSample{});
    for (size_t i = 0; i < nMission; ++i) out.payload[nTakeoff + i] = payloadMission.samples[i];

    out.aircraft.resize(nUavs);
    out.controls.resize(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        out.aircraft[k].payloadFrame.resize(nTotal);
        out.aircraft[k].inertial.resize(nTotal);
        for (size_t i = 0; i < nTakeoff; ++i) {
            out.aircraft[k].payloadFrame[i] = aircraftRaw[k].takeoff.payloadFrame[i];
            out.aircraft[k].inertial[i] = aircraftRaw[k].takeoff.inertial[i];
        }
        for (size_t i = 0; i < nMission; ++i) {
            out.aircraft[k].payloadFrame[nTakeoff + i] = aircraftRaw[k].timeline.payloadFrame[i];
            out.aircraft[k].inertial[nTakeoff + i] = aircraftRaw[k].timeline.inertial[i];
        }

        out.controls[k].resize(nTotal);
        for (size_t i = 0; i < nTakeoff; ++i) out.controls[k][i] = takeoffControls[k][i];
        for (size_t i = 0; i < nMission; ++i) out.controls[k][nTakeoff + i] = missionControls[k][i];
    }

    return out;
}

void TrajectoryGenerator::applyFieldCalibration(GeneratedMission& mission, const double fieldHeadingDeg, const Vec3d& originOffset) {
    if (fieldHeadingDeg == 0.0 && originOffset == Vec3d::zeros()) return;

    const Matrix3d R = rotationZ(grs::degToRad(fieldHeadingDeg));
    auto rotateTranslate = [&](KinematicSample& s, bool translate) {
        s.pos = R * s.pos;
        s.vel = R * s.vel;
        s.acc = R * s.acc;
        if (translate) s.pos += originOffset;
    };

    for (auto& s : mission.payload) rotateTranslate(s, true);
    for (auto& ac : mission.aircraft) {
        for (auto& s : ac.inertial) rotateTranslate(s, true);
        for (auto& s : ac.payloadFrame) rotateTranslate(s, false); // relative vector, no translation
    }
    for (auto& controlTrack : mission.controls) {
        for (auto& c : controlTrack) {
            c.yawRad += grs::degToRad(fieldHeadingDeg);
            c.liftDirection = R * c.liftDirection;
        }
    }
}

std::vector<double> TrajectoryGenerator::toSolverReference(const GeneratedMission& mission, const bool hasPayload) {
    const size_t nUavs = mission.aircraft.size();
    const size_t n = mission.time.size();

    constexpr size_t kUavBlockSize = 8;
    constexpr size_t kPayloadBlockSize = 6;
    constexpr size_t kControlsPerUav = 3;

    const size_t nx = nUavs * kUavBlockSize + (hasPayload ? kPayloadBlockSize : 0);
    const size_t nu = nUavs * kControlsPerUav;
    const size_t stride = nx + nu;

    std::vector<double> out(n * stride, 0.0);

    for (size_t i = 0; i < n; ++i) {
        double* row = out.data() + i * stride;
        size_t offset = 0;

        for (size_t k = 0; k < nUavs; ++k) {
            const auto& s = mission.aircraft[k].inertial[i];
            const auto& c = mission.controls[k][i];
            row[offset++] = s.pos[0]; row[offset++] = s.pos[1]; row[offset++] = s.pos[2];
            row[offset++] = s.vel[0]; row[offset++] = s.vel[1]; row[offset++] = s.vel[2];
            row[offset++] = c.rollRad; row[offset++] = c.pitchRad;
        }

        if (hasPayload) {
            const auto& p = mission.payload[i];
            row[offset++] = p.pos[0]; row[offset++] = p.pos[1]; row[offset++] = p.pos[2];
            row[offset++] = p.vel[0]; row[offset++] = p.vel[1]; row[offset++] = p.vel[2];
        }

        for (size_t k = 0; k < nUavs; ++k) {
            const auto& c = mission.controls[k][i];
            row[offset++] = c.thrustNewton; row[offset++] = c.rollRad; row[offset++] = c.pitchRad;
        }

        assert(offset == stride);
    }

    return out;
}

}
