/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "trajectoryGenerator.h"

#include <array>
#include <cassert>
#include <numeric>
#include <tuple>
#include <utility>

namespace grs::trajgen {

namespace {

// ---------------------------------------------------------------------------
// Small numeric helpers
// ---------------------------------------------------------------------------

Vec3d normalizeSafe(const Vec3d& v) {
    const double n = v.norm();
    const double denom = std::max(n, 1e-8);
    return v * (1.0 / denom);
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

double quinticSmoothstep(const double tau) {
    return 6*tau*tau*tau*tau*tau - 15*tau*tau*tau*tau + 10*tau*tau*tau;
}

double quinticSmoothstepDeriv(const double tau) {
    return 30*tau*tau*tau*tau - 60*tau*tau*tau + 30*tau*tau;
}

// d^2/dtau^2 of quinticSmoothstep: the tether payout's L''(t) (generateAircraftTakeoff)
// and the launch offset fade (snapToLiveLaunchPositions).
double quinticSmoothstepSecondDeriv(const double tau) {
    return 120*tau*tau*tau - 180*tau*tau + 60*tau;
}

// Quintic from x0 with slope0 (per unit tau) and zero curvature at tau=0, to
// x1 with zero slope and curvature at tau=1. Returns {x, dx/dtau}.
std::pair<double, double> quinticFromSlope(const double x0, const double slope0, const double x1, const double tau) {
    const double d = x1 - x0 - slope0;
    const double c3 = 10 * d + 4 * slope0, c4 = -15 * d - 7 * slope0, c5 = 6 * d + 3 * slope0;
    return {x0 + tau * (slope0 + tau * tau * (c3 + tau * (c4 + tau * c5))),
            slope0 + tau * tau * (3 * c3 + tau * (4 * c4 + tau * 5 * c5))};
}

// Tether pull on the aircraft during the takeoff [N]: 10 % of the loiter
// tension until 2 s, then ramped to the full tension over 6 s.
double takeoffTetherForce(const TrajectoryConfig& config, const double t) {
    const auto& ap = config.aircraftPath;
    const double zTarget = std::sqrt(config.tether.length*config.tether.length - ap.radius*ap.radius);
    const double fTarget = (0.5 * config.payload.mass * config.world.gravity) / (zTarget / config.tether.length);
    constexpr double tDelay = 2.0;
    constexpr double tRamp = 6.0;
    return (0.1 + 0.9 * quinticSmoothstep(std::clamp((t - tDelay) / tRamp, 0.0, 1.0))) * fTarget;
}

// ---------------------------------------------------------------------------
// sCurveProfile
// ---------------------------------------------------------------------------

struct Profile1D { std::vector<double> t, s, v, a; };

void appendLoiter(const double t0, const Vec3d& xStart, const double thold, const double dt, std::vector<double>& t, std::vector<KinematicSample>& samples, const bool skipFirst) {
    const size_t n = static_cast<size_t>(std::floor(thold / dt + 1e-9)) + 1;
    for (size_t i = (skipFirst ? 1 : 0); i < n; ++i) {
        t.push_back(t0 + static_cast<double>(i) * dt);
        KinematicSample s;
        s.pos = xStart;
        samples.push_back(s);
    }
}

// Jerk-limited S-curve, 1D, from rest to rest over the distance d. Sampled
// every dt from t = 0; the last sample, at or just after the end of the
// motion, holds the end state, so every sample lies on the trajectory's grid.
Profile1D sCurveProfile(double d, double vMax, double aMax, double jMax, double dt) {
    double tJ = aMax / jMax;
    double tA = vMax / aMax - tJ;

    if (tA < 0) {
        tJ = std::sqrt(vMax / jMax);
        tA = 0;
    }

    // Distance to accelerate from 0 to vMax (symmetric profile).
    double dAcc = 0.5 * vMax * (2.0*tJ + tA);
    double tV;

    if (2 * dAcc > d) {
        // vMax not reached: jerk phases only, peak speed from d/2 = vPeak tJ.
        tA = 0;
        tV = 0;
        const double vPeak = std::pow(d/2.0 * std::sqrt(jMax), 2.0/3.0);
        tJ = std::sqrt(vPeak / jMax);
    } else {
        tV = (d - 2*dAcc) / vMax;
    }

    const std::array<double, 7> duration{tJ, tA, tJ, tV, tJ, tA, tJ};
    const std::array<double, 7> jerk{jMax, 0.0, -jMax, 0.0, -jMax, 0.0, jMax};
    const double tEnd = std::accumulate(duration.begin(), duration.end(), 0.0);
    const auto n = static_cast<size_t>(std::ceil(tEnd / dt - 1e-9)) + 1;

    Profile1D out;
    for (size_t k = 0; k < n; ++k) {
        double t = std::min(static_cast<double>(k) * dt, tEnd);
        double sk = 0.0, vk = 0.0, ak = 0.0;
        for (size_t p = 0; p < duration.size() && t > 0.0; ++p) {
            const double h = std::min(duration[p], t);
            const double j = jerk[p];
            sk += vk*h + 0.5*ak*h*h + j*h*h*h/6.0;
            vk += ak*h + 0.5*j*h*h;
            ak += j*h;
            t -= h;
        }
        out.t.push_back(static_cast<double>(k) * dt);
        out.s.push_back(sk);
        out.v.push_back(vk);
        out.a.push_back(ak);
    }
    return out;
}

// ---------------------------------------------------------------------------
// (mission phases only -- pre-appendTakeoff)
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
// Aircraft Takeoff
// ---------------------------------------------------------------------------

// Catapult stroke: samples from rest to the release and their length [m]
// (prependCatapultStroke), rounded to the time grid. 0 without a stroke.
std::pair<size_t, double> catapultStroke(const TrajectoryConfig& config) {
    const double V0 = config.aircraftPath.takeoffVel0;
    const double dt = config.simDt;
    if (!(config.aircraftPath.railLength > 0.0) || !(V0 > 0.0)) return {0, 0.0};
    const auto n = static_cast<size_t>(std::max(1.0, std::round(1.5 * config.aircraftPath.railLength / V0 / dt)));
    return {n, 2.0 * V0 * static_cast<double>(n) * dt / 3.0};
}

// Rail direction: horizontal direction of the release velocity, inclined at takeoffPitch0Deg.
Vec3d railDirection(const TrajectoryConfig& config, const Vec3d& releaseVel) {
    const double rail = grs::degToRad(config.aircraftPath.takeoffPitch0Deg);
    return normalizeSafe(Vec3d(releaseVel[0], releaseVel[1], 0.0)) * std::cos(rail) + Vec3d(0.0, 0.0, -std::sin(rail));
}

struct TakeoffMission {
    std::vector<double> t;
    std::vector<double> gamma, psi;
    size_t phase1End = 0; // last sample with the pitch held at the rail angle
    std::vector<KinematicSample> payloadFrame; // relative to launcher (pre-x0)
    std::vector<KinematicSample> inertial;     // + config.payloadPath.x0
};

TakeoffMission generateAircraftTakeoff(const TrajectoryConfig& config, const double phase, const double z0, const double L0, const double dPsiEnd) {
    const auto& ap = config.aircraftPath;
    const double Lfinal = config.tether.length;
    const double dt = config.simDt;
    const double T = ap.takeoffTime;

    // Slip-clutch tether payout, see TrajectoryConfig::Tether. Scope of this
    // model, deliberately: the angular guidance law below (theta/thetaDot/
    // thetaDdot/psi/gamma, the boundary-value-matched pitch profile and the
    // resulting azimuth-rate ODE) is left exactly as validated against
    // MATLAB, computed with `Lfinal` throughout, except for theta0 and the
    // phase-1 rate formulas just below, which use the true launch-instant
    // tether length `L0` since they describe the aircraft's actual physical
    // elevation angle/rate right at t=0. The payout itself
    // (the tether's radial length actually changing over time) is applied
    // only when converting the resulting spherical path (L(t), theta(t),
    // psi(t)) to Cartesian position/velocity/acceleration below, via the
    // ordinary product rule on L(t): a real outward-radial motion layered
    // on top of the unchanged angular path, not a re-solve of the
    // underlying boundary-value problem.
    const double Tpo = std::max(config.tether.payoutDurationSeconds, 0.0);
    const double dL = Lfinal - L0;
    auto payoutTau = [&](const double time) { return Tpo > 1e-9 ? std::clamp(time / Tpo, 0.0, 1.0) : 1.0; };
    auto lengthAt = [&](const double time) { return L0 + dL * quinticSmoothstep(payoutTau(time)); };
    auto lengthDotAt = [&](const double time) {
        return (Tpo > 1e-9 && time < Tpo) ? dL * quinticSmoothstepDeriv(payoutTau(time)) / Tpo : 0.0;
    };
    auto lengthDdotAt = [&](const double time) {
        return (Tpo > 1e-9 && time < Tpo) ? dL * quinticSmoothstepSecondDeriv(payoutTau(time)) / (Tpo * Tpo) : 0.0;
    };

    const size_t n = static_cast<size_t>(std::floor(T / dt + 1e-9)) + 1;
    std::vector<double> t(n);
    for (size_t i = 0; i < n; ++i) {
        t[i] = i * dt;
    }

    const double V0 = ap.takeoffVel0;
    const double Vf = ap.velMean;
    // Starts with the acceleration measured after the release (flight
    // 2026-08-25: 10.2 m/s at 0.4 s, 11.5 at 1 s, 15 at 2 s).
    const double Tacc = ap.takeoffTimeAcc;
    auto speedAt = [&](const double time) -> std::pair<double, double> {
        if (time >= Tacc) return {Vf, 0.0};
        const auto [vi, dv] = quinticFromSlope(V0, ap.takeoffAcc0 * Tacc, Vf, time / Tacc);
        return {vi, dv / Tacc};
    };
    std::vector<double> v(n), vDot(n);
    for (size_t i = 0; i < n; ++i) {
        std::tie(v[i], vDot[i]) = speedAt(t[i]);
    }

    std::vector<double> gamma(n, 0.0), theta(n, 0.0), thetaDot(n, 0.0), thetaDdot(n, 0.0);

    const double theta0 = std::asin(z0 / L0);
    const double zTarget = std::sqrt(Lfinal*Lfinal - ap.radius*ap.radius);
    const double thetaF = std::asin(zTarget / Lfinal);

    // Phase 1, from the release to the lowest path angle: pitch held at the
    // rail angle (the autopilot holds the surfaces at 0 until 12 m/s), path
    // angle from the lift at AoA = rail - gamma, the tether pull and the turn
    // around the anchor. Flight 2026-08-25: 20, 14, 11, 7, 5 deg over 0.6 s.
    const double railPitch = grs::degToRad(ap.takeoffPitch0Deg);
    const double mass = config.aircraft.mass;
    const double halfRhoS = 0.5 * config.world.airDensity * config.aircraft.wingArea;
    const double F0 = takeoffTetherForce(config, 0.0);
    const double cTheta0 = std::max(std::cos(theta0), 1e-6);
    // The tether holds the aircraft on its sphere (turn around the anchor and
    // payout), at least at the takeoff pull: the wing only takes the excess.
    auto gammaRate = [&](const double gam, const double V, const double time) {
        const double lift = halfRhoS * V * V * (config.aircraft.CL0 + config.aircraft.CLalpha * (railPitch - gam));
        const double vh = V * std::cos(gam);
        const double inward = mass * (vh * vh / (lengthAt(time) * cTheta0) - lengthDdotAt(time) * cTheta0);
        const double tether = std::max(inward / std::cos(theta0), F0);
        const double side = inward - tether * std::cos(theta0);
        const double up = std::sqrt(std::max(lift * lift - side * side, 0.0));
        return (up - (mass * config.world.gravity + tether * std::sin(theta0)) * std::cos(gam)) / (mass * V);
    };

    const size_t iMax = std::min(n - 1, static_cast<size_t>(std::floor(ap.takeoffTimeBalistic / dt + 1e-9)));
    constexpr int nSub = 10;
    std::vector<double> thetaDotPhase1, tPhase1;
    size_t iSwitch = 0;
    double gam = railPitch;
    for (size_t i = 0;; ++i) {
        const double gd = gammaRate(gam, v[i], t[i]);
        gamma[i] = gam;
        const double Li = lengthAt(t[i]);
        thetaDotPhase1.push_back((v[i] * std::sin(gam)) / (Li * cTheta0));
        thetaDdot[i] = (vDot[i] * std::sin(gam) + v[i] * std::cos(gam) * gd) / (Li * cTheta0)
                     - thetaDotPhase1.back() * lengthDotAt(t[i]) / Li;
        tPhase1.push_back(t[i]);
        if (gd >= 0.0 || i == iMax) {
            iSwitch = i;
            break;
        }
        for (int k = 0; k < nSub; ++k) {
            const double tk = t[i] + k * dt / nSub;
            gam += gammaRate(gam, speedAt(tk).first, tk) * dt / nSub;
        }
    }
    const double t1 = t[iSwitch];
    const double t2 = T - t1;
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
    // Jerk of the phase-1 path at the boundary, from the derivatives of the
    // path angle ODE and of the speed ramp (smooth functions, not the time grid).
    const double thetaJerk1 = [&] {
        constexpr double h = 1e-4;
        const double g1 = gamma[iSwitch];
        const double gd1 = gammaRate(g1, v[iSwitch], t1);
        const auto [vh, vDoth] = speedAt(t1 + h);
        const double gdd1 = (gammaRate(g1 + gd1 * h, vh, t1 + h) - gd1) / h;
        const double vdd1 = (vDoth - vDot[iSwitch]) / h;
        return (vdd1 * std::sin(g1) + 2 * vDot[iSwitch] * std::cos(g1) * gd1 + v[iSwitch] * std::cos(g1) * gdd1
                - v[iSwitch] * std::sin(g1) * gd1 * gd1) / (lengthAt(t1) * cTheta0);
    }();

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
        for (int r = col+1; r < 4; ++r) {
            if (std::abs(A[r][col]) > std::abs(A[piv][col])) {
                piv = r;
            }
        }
        if (piv != col) {
            std::swap(A[piv], A[col]); std::swap(rhs[piv], rhs[col]);
        }
        for (int r = col+1; r < 4; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int c = col; c < 4; ++c) {
                A[r][c] -= f * A[col][c];
            }
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

    // Azimuth: the speed left after the elevation and payout rates, on the
    // actual tether length, so the total speed is v. psiDdot is its analytic
    // derivative (a finite difference is one-sided at the release).
    std::vector<double> psiDot(n, 0.0), psiDdot(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const double Lt = lengthAt(t[i]), LDot = lengthDotAt(t[i]), LDdot = lengthDdotAt(t[i]);
        const double cTheta = std::max(std::cos(theta[i]), 1e-6);
        const double w = v[i]*v[i] - Lt*Lt*thetaDot[i]*thetaDot[i] - LDot*LDot;
        if (w <= 1e-12) continue;
        const double wDot = 2 * v[i] * vDot[i] - 2 * Lt * thetaDot[i] * (LDot * thetaDot[i] + Lt * thetaDdot[i]) - 2 * LDot * LDdot;
        const double den = Lt * cTheta;
        const double denDot = LDot * cTheta - Lt * std::sin(theta[i]) * thetaDot[i];
        psiDot[i] = config.aircraftPath.direction * std::sqrt(w) / den;
        psiDdot[i] = config.aircraftPath.direction * (wDot / (2 * std::sqrt(w) * den) - std::sqrt(w) * denDot / (den * den));
    }
    const std::vector<double> psiCum = cumtrapzScalar(t, psiDot);
    std::vector<double> psi(n);
    for (size_t i = 0; i < n; ++i) psi[i] = phase + psiCum[i];
    // Azimuth correction faded in over the takeoff (zero rate and acceleration at both ends).
    for (size_t i = 0; i < n; ++i) {
        const double tau = std::clamp(t[i] / T, 0.0, 1.0);
        psi[i] += dPsiEnd * quinticSmoothstep(tau);
        psiDot[i] += dPsiEnd * quinticSmoothstepDeriv(tau) / T;
        psiDdot[i] += dPsiEnd * quinticSmoothstepSecondDeriv(tau) / (T * T);
    }

    TakeoffMission out;
    out.t = t; out.gamma = gamma; out.psi = psi;
    out.phase1End = iSwitch;
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

// Takeoff of a UAV given at rest on the launcher (phase, height above the
// payload, distance to it, as captured live): the release is found so that
// the stroke back along the rail (prependCatapultStroke) ends at that rest
// position. The rail follows the release velocity, which turns with the
// release azimuth, so a few iterations converge to well below a millimetre.
TakeoffMission generateTakeoffFromRest(const TrajectoryConfig& config, const double phase, const double z0, const double L0, const double dPsiEnd) {
    const double r = std::sqrt(std::max(L0*L0 - z0*z0, 0.0));
    const Vec3d rest{r * std::cos(phase), r * std::sin(phase), -z0};
    const double stroke = catapultStroke(config).second;
    Vec3d release = rest;
    TakeoffMission takeoff = generateAircraftTakeoff(config, phase, z0, L0, dPsiEnd);
    for (int it = 0; it < 4 && stroke > 0.0; ++it) {
        release = rest + railDirection(config, takeoff.payloadFrame.front().vel) * stroke;
        takeoff = generateAircraftTakeoff(config, std::atan2(release[1], release[0]), -release[2], release.norm(), dPsiEnd);
    }
    return takeoff;
}

struct AircraftMissionRaw { AircraftTimeline timeline; TakeoffMission takeoff; };

AircraftMissionRaw generateAircraftPathForUav(const TrajectoryConfig& config, const double phase, const double z0, const double L0, const double dPsiEnd, const PayloadMission& payloadMission) {
    const auto& ap = config.aircraftPath;
    const double R = ap.radius;
    const double omega = ap.direction * ap.velMean / R;
    const double zpay = -std::sqrt(config.tether.length*config.tether.length - R*R);

    AircraftMissionRaw out;
    out.takeoff = generateTakeoffFromRest(config, phase, z0, L0, dPsiEnd);
    const double psiEnd = out.takeoff.psi.back();

    const size_t n = payloadMission.t.size();
    out.timeline.payloadFrame.resize(n);
    out.timeline.inertial.resize(n);

    for (size_t i = 0; i < n; ++i) {
        // payloadMission.t[0] is one sample after the end of the takeoff (generate()).
        const double theta = omega * (payloadMission.t[i] + config.simDt) + psiEnd;
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
// Compute Forces On Payload
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
// static (time-invariant): gravity only.
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
        for (size_t r = col + 1; r < N; ++r) {
            if (std::abs(A[r][col]) > std::abs(A[piv][col])) {
                piv = r;
            }
        }
        if (piv != col) {
            std::swap(A[piv], A[col]); std::swap(b[piv], b[col]);
        }
        for (size_t r = col + 1; r < N; ++r) {
            const double f = A[r][col] / A[col][col];
            for (size_t c = col; c < N; ++c) {
                A[r][c] -= f * A[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    std::array<double, N> x{};
    for (size_t ri = 0; ri < N; ++ri) {
        const size_t r = N - 1 - ri;
        double sum = b[r];
        for (size_t c = r + 1; c < N; ++c) {
            sum -= A[r][c] * x[c];
        }
        x[r] = sum / A[r][r];
    }
    return x;
}

// ---------------------------------------------------------------------------
// Solve Tether Forces Two Aircraft
// ---------------------------------------------------------------------------

struct TetherForcePair { Vec3d f1 = Vec3d::zeros(), f2 = Vec3d::zeros(); };

TetherForcePair solveTetherForcesTwoAircraft(const TrajectoryConfig& config, const TetherStaticForces& staticForces, const Vec3d& fOnPayload, const Vec3d& payloadPos, const Vec3d& payloadAcc, const Vec3d& ac1Pos, const Vec3d& ac2Pos) {
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
    auto fillA2 = [&](const int rowBase, const int colBase) {
        A[rowBase+0][colBase+0] = 1; A[rowBase+0][colBase+1] = 1;
        A[rowBase+1][colBase+2] = 1; A[rowBase+1][colBase+3] = 1;
        A[rowBase+2][colBase+4] = 1; A[rowBase+2][colBase+5] = 1;
    };
    fillA2(3, 0);
    fillA2(9, 7);

    B[3] = staticForces.total[0]; B[4] = staticForces.total[1]; B[5] = staticForces.total[2];
    B[9] = staticForces.total[0]; B[10] = staticForces.total[1]; B[11] = staticForces.total[2];

    // A3 (per tether: sum of moments about the tip) -- rows 6-8, 12-14.
    auto fillA3AndB3 = [&](const int rowBase, const int colBase, const Vec3d& armVec) {
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
    return out;
}

// ---------------------------------------------------------------------------
// Compute Angle Of Attack/Thrust, Inverse Dynamic
// ---------------------------------------------------------------------------

struct AeroFromLift { double CL, CD, drag, angleOfAttack; };

AeroFromLift aeroFromLift(const TrajectoryConfig& config, const double lift, const double airspeed) {
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

    const Vec3d freestream = vel - config.world.wind;
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
    const auto [angleOfAttack, thrust, liftDir] = computeAngleOfAttackThrust(config, sample.vel, sample.acc, externalForce);

    const Vec3d freestream = sample.vel - config.world.wind;
    const Vec3d freestreamUnit = normalizeSafe(freestream);

    constexpr Vec3d zWorld{0, 0, 1};
    Vec3d latAxis = cross(zWorld, freestreamUnit);
    latAxis = latAxis * (1.0 / std::max(latAxis.norm(), 1e-8));
    const Vec3d liftProjPlane = cross(latAxis, freestreamUnit);

    const Vec3d bankCross = cross(liftDir, liftProjPlane);
    double bank = std::asin(std::clamp(bankCross.norm(), -1.0, 1.0));
    const double signBank = dot(bankCross, freestreamUnit) >= 0 ? 1.0 : -1.0;
    bank *= signBank;

    const double vXy = std::hypot(freestream[0], freestream[1]);
    const double pathAngle = -std::atan2(freestream[2], vXy);

    ControlSample out;
    out.yawRad = std::atan2(freestream[1], freestream[0]);
    out.pitchRad = angleOfAttack + pathAngle;
    out.rollRad = -bank;
    out.thrustNewton = thrust;
    out.angleOfAttackRad = angleOfAttack;
    out.liftDirection = liftDir;
    return out;
}

// ---------------------------------------------------------------------------
// Compute Controls Takeoff
// ---------------------------------------------------------------------------

std::vector<ControlSample> computeControlsTakeoff(const TrajectoryConfig& config, const double rollLoiter, const TakeoffMission& takeoff) {
    const auto& ap = config.aircraftPath;
    const double T = ap.takeoffTime;
    const size_t n = takeoff.t.size();
    const double railPitch = grs::degToRad(ap.takeoffPitch0Deg);

    std::vector<ControlSample> out(n);
    for (size_t i = 0; i < n; ++i) {
        const double t = takeoff.t[i];
        const double tau = std::clamp(t / T, 0.0, 1.0);
        const double s = quinticSmoothstep(tau);
        const double phi = (1 - s) * grs::degToRad(ap.takeoffRoll0Deg) + s * rollLoiter;

        // At least the pull that holds the aircraft on its sphere: the tether,
        // not the wing, takes the turn around the anchor and the payout.
        Vec3d et = config.payloadPath.x0 - takeoff.inertial[i].pos;
        et = normalizeSafe(et);
        const Vec3d weight{0, 0, config.aircraft.mass * config.world.gravity};
        const double hold = dot(takeoff.inertial[i].acc * config.aircraft.mass - weight, et);
        const Vec3d externalForce = et * std::max(hold, takeoffTetherForce(config, t));

        const auto [angleOfAttack, thrust, liftDir] = computeAngleOfAttackThrust(config, takeoff.inertial[i].vel, takeoff.inertial[i].acc, externalForce);
        const Vec3d freestream = takeoff.inertial[i].vel - config.world.wind;
        const double pathAngle = -std::atan2(freestream[2], std::hypot(freestream[0], freestream[1]));

        ControlSample& c = out[i];
        c.rollRad = phi;
        c.yawRad = std::atan2(freestream[1], freestream[0]);
        c.thrustNewton = thrust;
        c.liftDirection = liftDir;
        // Phase 1 of generateAircraftTakeoff: pitch held at the rail angle, angle of attack = rail - path angle.
        c.pitchRad = i <= takeoff.phase1End ? railPitch : angleOfAttack + pathAngle;
        c.angleOfAttackRad = i <= takeoff.phase1End ? railPitch - takeoff.gamma[i] : angleOfAttack;
    }
    return out;
}

// Catapult stroke along the rail (azimuth of the launch velocity, inclined at
// takeoffPitch0Deg), from rest to takeoffVel0, ending on the first takeoff
// sample. The acceleration decays linearly to 0 at the release, as measured
// on the flight of 2026-08-25: T = 3 L / (2 V0). The first sample is then the
// UAV at rest on the launcher. Rounded to the time grid, so the stroke is
// 2 V0 n dt / 3 rather than exactly railLength. Controls hold the release
// values, with the pitch of the rail and zero angle of attack.
void prependCatapultStroke(const TrajectoryConfig& config, GeneratedMission& mission) {
    const double V0 = config.aircraftPath.takeoffVel0;
    const double dt = config.simDt;
    const auto [n, stroke] = catapultStroke(config);
    if (n == 0 || mission.time.empty()) return;

    const double T = static_cast<double>(n) * dt;
    const double a0 = 2.0 * V0 / T;

    for (auto& t : mission.time) t += static_cast<double>(n) * dt;
    std::vector<double> time(n);
    for (size_t i = 0; i < n; ++i) time[i] = static_cast<double>(i) * dt;
    mission.time.insert(mission.time.begin(), time.begin(), time.end());
    mission.payload.insert(mission.payload.begin(), n, mission.payload.front());

    for (size_t k = 0; k < mission.aircraft.size(); ++k) {
        const KinematicSample release = mission.aircraft[k].inertial.front();
        const double railPitch = grs::degToRad(config.aircraftPath.takeoffPitch0Deg);
        const Vec3d u = railDirection(config, release.vel);
        std::vector<KinematicSample> inertial(n), payloadFrame(n);
        for (size_t i = 0; i < n; ++i) {
            const double t = time[i];
            inertial[i].pos = release.pos - u * (stroke - a0 * (t * t / 2.0 - t * t * t / (6.0 * T)));
            inertial[i].vel = u * (a0 * (t - t * t / (2.0 * T)));
            inertial[i].acc = u * (a0 * (1.0 - t / T));
            payloadFrame[i] = inertial[i];
            payloadFrame[i].pos = inertial[i].pos - config.payloadPath.x0; // as the takeoff samples
        }
        auto& tl = mission.aircraft[k];
        tl.inertial.insert(tl.inertial.begin(), inertial.begin(), inertial.end());
        tl.payloadFrame.insert(tl.payloadFrame.begin(), payloadFrame.begin(), payloadFrame.end());
        ControlSample onRail = mission.controls[k].front();
        onRail.pitchRad = railPitch;
        onRail.angleOfAttackRad = 0.0;
        mission.controls[k].insert(mission.controls[k].begin(), n, onRail);
    }
    mission.releaseIndex = n;
}

} // namespace

// ---------------------------------------------------------------------------
// Trajectory Generator
// ---------------------------------------------------------------------------

TrajectoryGenerator::TrajectoryGenerator(TrajectoryConfig config) : m_config(std::move(config)) {
    m_config.finalize();
}

GeneratedMission TrajectoryGenerator::generate() const {
    // apps/main.m, in order.
    const PayloadMission payloadMission = generatePayloadPath(m_config);

    const auto& ap = m_config.aircraftPath;
    const size_t nUavs = ap.phaseRad.size();
    assert(ap.z0.size() == nUavs && "One launcher height (z0) per UAV.");
    assert(m_config.tether.lengthAtLaunch.size() == nUavs && "One launcher distance (lengthAtLaunch) per UAV.");
    assert(nUavs == 2 && "Tether force solve is hard-coded for two aircraft, matching the MATLAB source.");

    // Takeoffs from launchers at different heights and distances end at azimuths
    // that are not exactly opposite: each is turned by half the gap so the UAVs
    // circle on opposite sides of the payload.
    std::array<double, 2> psiEnd{};
    for (size_t k = 0; k < nUavs; ++k) {
        psiEnd[k] = generateTakeoffFromRest(m_config, ap.phaseRad[k], ap.z0[k], m_config.tether.lengthAtLaunch[k], 0.0).psi.back();
    }
    const double gap = std::remainder(psiEnd[0] - psiEnd[1] - M_PI, 2.0 * M_PI);
    const std::array<double, 2> dPsiEnd{-0.5 * gap, 0.5 * gap};

    std::vector<AircraftMissionRaw> aircraftRaw(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        aircraftRaw[k] = generateAircraftPathForUav(m_config, ap.phaseRad[k], ap.z0[k], m_config.tether.lengthAtLaunch[k], dPsiEnd[k], payloadMission);
    }

    // dyn.getTethersForces: static per-tether gravity + per-sample tether pull at each aircraft.
    const TetherStaticForces staticForces = computeForcesOnTethers(m_config);
    const std::vector<Vec3d> fOnPayload = computeForcesOnPayload(m_config, payloadMission);

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
    std::vector<std::vector<ControlSample>> takeoffControls(nUavs);
    for (size_t k = 0; k < nUavs; ++k) {
        takeoffControls[k] = computeControlsTakeoff(m_config, missionControls[k].front().rollRad, aircraftRaw[k].takeoff);
    }

    // traj.appendTakeoff: prepend takeoff kinematics to each aircraft, payload at rest meanwhile.
    GeneratedMission out;
    const size_t nTakeoff = aircraftRaw[0].takeoff.t.size();
    const size_t nTotal = nTakeoff + nMission;

    out.time.resize(nTotal);
    for (size_t i = 0; i < nTakeoff; ++i) out.time[i] = i * m_config.simDt;
    for (size_t i = 0; i < nMission; ++i) out.time[nTakeoff + i] = static_cast<double>(nTakeoff) * m_config.simDt + payloadMission.t[i];

    out.payload.assign(nTotal, payloadMission.samples.front()); // at rest at x0 during the takeoff
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

    prependCatapultStroke(m_config, out);
    out.takeoffEndIndex = out.releaseIndex + nTakeoff;
    return out;
}

void TrajectoryGenerator::applyFieldCalibration(GeneratedMission& mission, const double fieldHeadingDeg, const Vec3d& originOffset) {
    if (fieldHeadingDeg == 0.0 && originOffset == Vec3d::zeros()) return;

    const Matrix3d R = rotationZ(grs::degToRad(fieldHeadingDeg));
    auto rotateTranslate = [&](KinematicSample& s, const bool translate) {
        s.pos = R * s.pos;
        s.vel = R * s.vel;
        s.acc = R * s.acc;
        if (translate) s.pos += originOffset;
    };

    for (auto& s : mission.payload) {
        rotateTranslate(s, true);
    }

    for (auto&[payloadFrame, inertial] : mission.aircraft) {
        for (auto& s : inertial) {
            rotateTranslate(s, true);
        }
        for (auto& s : payloadFrame) {
            rotateTranslate(s, false); // relative vector, no translation
        }
    }

    for (auto& controlTrack : mission.controls) {
        for (auto& c : controlTrack) {
            c.yawRad += grs::degToRad(fieldHeadingDeg);
            c.liftDirection = R * c.liftDirection;
        }
    }
}

void TrajectoryGenerator::snapToLiveLaunchPositions(GeneratedMission& mission,
    const std::vector<std::optional<Vec3d>>& liveLaunchPositionsNed) {
    const size_t n = std::min(mission.aircraft.size(), liveLaunchPositionsNed.size());
    for (size_t k = 0; k < n; ++k) {
        if (!liveLaunchPositionsNed[k]) continue;

        auto& timeline = mission.aircraft[k].inertial;
        if (timeline.empty()) continue;

        // Full offset up to the release, then a quintic fade to zero at the end of the takeoff,
        // velocity and acceleration included: the mission circles stay centred on the payload.
        const Vec3d delta = *liveLaunchPositionsNed[k] - timeline.front().pos;
        const double t0 = mission.time[mission.releaseIndex];
        const double T = mission.time[mission.takeoffEndIndex] - t0;
        for (size_t i = 0; i < timeline.size(); ++i) {
            const double s = std::clamp((mission.time[i] - t0) / T, 0.0, 1.0);
            timeline[i].pos += delta * (1.0 - quinticSmoothstep(s));
            timeline[i].vel -= delta * (quinticSmoothstepDeriv(s) / T);
            timeline[i].acc -= delta * (quinticSmoothstepSecondDeriv(s) / (T * T));
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

GeneratedMission TrajectoryGenerator::extractSubset(const GeneratedMission& mission, const SubsetSelection& selection) {
    const size_t n = (selection.maxSamples > 0) ? std::min(selection.maxSamples, mission.time.size()) : mission.time.size();

    std::vector<size_t> indices;
    if (selection.uavIndices.has_value()) {
        indices = *selection.uavIndices;
    } else {
        indices.resize(mission.aircraft.size());
        std::iota(indices.begin(), indices.end(), 0);
    }

    GeneratedMission out;
    out.time.assign(mission.time.begin(), mission.time.begin() + n);
    out.releaseIndex = std::min(mission.releaseIndex, n);
    out.takeoffEndIndex = std::min(mission.takeoffEndIndex, n);
    out.payload.assign(mission.payload.begin(), mission.payload.begin() + n);

    out.aircraft.reserve(indices.size());
    out.controls.reserve(indices.size());
    for (const size_t idx : indices) {
        if (idx >= mission.aircraft.size()) continue; // defensive: ignore an out-of-range index rather than throwing

        AircraftTimeline timeline;
        const auto&[payloadFrame, inertial] = mission.aircraft[idx];
        timeline.payloadFrame.assign(payloadFrame.begin(), payloadFrame.begin() + n);
        timeline.inertial.assign(inertial.begin(), inertial.begin() + n);
        out.aircraft.push_back(std::move(timeline));

        out.controls.emplace_back(mission.controls[idx].begin(), mission.controls[idx].begin() + n);
    }

    return out;
}

}
