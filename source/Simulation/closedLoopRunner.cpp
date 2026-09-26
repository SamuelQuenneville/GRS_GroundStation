/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "closedLoopRunner.h"

namespace grs::sim {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kUavBlock = 8;

// Measured joint state -> the per-vehicle telemetry map the GCS feeds its
// controller (NavigationFrameManager output): UAVs as sysId 1..numUavs,
// payload as numUavs+1. Same float precision and degree units as live
// telemetry, so the controller sees exactly what it would see in flight.
std::map<uint8_t, uavStates> toTelemetry(const std::vector<double>& x, const int numUavs, const bool hasPayload,
                                        const std::vector<double>& wind) {
    std::map<uint8_t, uavStates> out;
    auto fill = [&](const size_t o, const bool attitude) {
        uavStates s{};
        s.northMeter = static_cast<float>(x[o + 0]);
        s.eastMeter = static_cast<float>(x[o + 1]);
        s.downMeter = static_cast<float>(x[o + 2]);
        s.northMeterSecond = static_cast<float>(x[o + 3]);
        s.eastMeterSecond = static_cast<float>(x[o + 4]);
        s.downMeterSecond = static_cast<float>(x[o + 5]);
        const double ax = x[o + 3] - wind[0], ay = x[o + 4] - wind[1], az = x[o + 5] - wind[2];
        s.airspeedMeterSecond = static_cast<float>(std::sqrt(ax * ax + ay * ay + az * az));
        s.yawDegree = static_cast<float>(grs::radToDeg(std::atan2(x[o + 4], x[o + 3])));
        if (attitude) {
            s.rollDegree = static_cast<float>(grs::radToDeg(x[o + 6]));
            s.pitchDegree = static_cast<float>(grs::radToDeg(x[o + 7]));
        }
        return s;
    };
    for (int i = 0; i < numUavs; ++i) {
        out[static_cast<uint8_t>(i + 1)] = fill(static_cast<size_t>(i * kUavBlock), true);
    }
    if (hasPayload) {
        out[static_cast<uint8_t>(numUavs + 1)] = fill(static_cast<size_t>(numUavs * kUavBlock), false);
    }
    return out;
}

double mean(const std::vector<double>& v) {
    double s = 0.0;
    size_t n = 0;
    for (const double x : v) {
        if (std::isnan(x)) continue;
        s += x;
        ++n;
    }
    return n ? s / static_cast<double>(n) : kNaN;
}

double maxOmitNan(const std::vector<double>& v) {
    double m = kNaN;
    for (const double x : v) {
        if (std::isnan(x)) continue;
        if (std::isnan(m) || x > m) m = x;
    }
    return m;
}

// Nearest-rank percentile, NaNs dropped (mc_metrics_twoUav.m's pct()).
double pct(std::vector<double> v, const double p) {
    v.erase(std::remove_if(v.begin(), v.end(), [](const double x) { return std::isnan(x); }), v.end());
    if (v.empty()) return kNaN;
    std::sort(v.begin(), v.end());
    const auto k = static_cast<size_t>(std::max(1.0, std::ceil(p / 100.0 * static_cast<double>(v.size()))));
    return v[k - 1];
}

double norm3(const double* a, const double* b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct History {
    std::vector<std::vector<double>> states;   // n+1 true states (index 0 = initial)
    std::vector<std::vector<double>> controls; // n applied controls
    std::vector<std::vector<double>> alpha;    // n, truth-plant AoA after each step
    std::vector<std::vector<double>> windHat;  // n
    std::vector<std::vector<double>> dHat;     // n
    std::vector<double> ctrlMs, mheMs;         // n
    std::vector<bool> ok;                      // n
};

// Port of mc_metrics_twoUav.m (same names, same definitions), generalized
// to the one-UAV layout where a metric has a natural counterpart (payload
// and second-UAV metrics are NaN without a payload / second UAV). Iteration
// counts are NaN: the codegen'd C API only returns a status flag.
std::vector<std::pair<std::string, double>> computeMetrics(
    const History& h, const std::vector<double>& ref, const solverConfig& sc, const TruthSpec& truth,
    const bool hasPayload, const bool useEst) {
    const int nx = sc.nx, nu = sc.nu, numUavs = sc.numUavs;
    const size_t stride = static_cast<size_t>(nx + nu);
    const size_t n = h.controls.size();
    std::vector<std::pair<std::string, double>> m;
    auto put = [&](const std::string& k, const double v) { m.emplace_back(k, v); };

    auto X = [&](const size_t k) -> const std::vector<double>& { return h.states[k + 1]; }; // after control k
    auto R = [&](const size_t k) { return ref.data() + (k + 1) * stride; };

    const size_t payOff = static_cast<size_t>(numUavs * kUavBlock);

    // --- Tracking
    if (hasPayload && n > 0) {
        std::vector<double> en(n), ealt(n), ehor(n);
        for (size_t k = 0; k < n; ++k) {
            const double* xp = X(k).data() + payOff;
            const double* rp = R(k) + payOff;
            const double ex = xp[0] - rp[0], ey = xp[1] - rp[1], ez = xp[2] - rp[2];
            en[k] = std::sqrt(ex * ex + ey * ey + ez * ez);
            ealt[k] = ez;
            ehor[k] = std::sqrt(ex * ex + ey * ey);
        }
        auto rms = [](const std::vector<double>& v, const size_t from) {
            double s = 0.0;
            for (size_t i = from; i < v.size(); ++i) s += v[i] * v[i];
            return std::sqrt(s / static_cast<double>(v.size() - from));
        };
        put("pay_rms_3d", rms(en, 0));
        put("pay_peak_3d", *std::max_element(en.begin(), en.end()));
        put("pay_rms_alt", rms(ealt, 0));
        put("pay_rms_horiz", rms(ehor, 0));
        const size_t q = std::max<size_t>(1, static_cast<size_t>(std::floor(0.75 * static_cast<double>(n)))) - 1;
        put("pay_rms_3d_last25", rms(en, q));
    } else {
        for (const char* k : {"pay_rms_3d", "pay_peak_3d", "pay_rms_alt", "pay_rms_horiz", "pay_rms_3d_last25"}) put(k, kNaN);
    }

    for (int i = 0; i < 2; ++i) {
        const std::string key = "uav" + std::to_string(i + 1) + "_rms_3d";
        if (i >= numUavs || n == 0) {
            put(key, kNaN);
            continue;
        }
        double s = 0.0;
        for (size_t k = 0; k < n; ++k) {
            const double e = norm3(X(k).data() + i * kUavBlock, R(k) + i * kUavBlock);
            s += e * e;
        }
        put(key, std::sqrt(s / static_cast<double>(n)));
    }

    // --- Angle of attack (truth plant)
    double aMax = 0.0;
    size_t aViol = 0;
    for (const auto& a : h.alpha) {
        bool any = false;
        for (const double v : a) {
            aMax = std::max(aMax, std::fabs(v));
            any = any || std::fabs(v) > sc.alphaMax;
        }
        aViol += any ? 1 : 0;
    }
    put("alpha_max_deg", n ? grs::radToDeg(aMax) : kNaN);
    put("alpha_margin_min_deg", n ? grs::radToDeg(sc.alphaMax) - grs::radToDeg(aMax) : kNaN);
    put("alpha_violation_frac", n ? static_cast<double>(aViol) / static_cast<double>(n) : kNaN);

    // --- Tethers (true L0). One-UAV: tether anchored at the origin.
    if (n > 0) {
        double stretchMax = -std::numeric_limits<double>::infinity();
        size_t lifted = 0, slackLifted = 0;
        const double origin[3] = {0.0, 0.0, 0.0};
        for (size_t k = 0; k < n; ++k) {
            const double* anchor = hasPayload ? X(k).data() + payOff : origin;
            bool slack = false;
            for (int i = 0; i < numUavs; ++i) {
                const double dl = norm3(X(k).data() + i * kUavBlock, anchor) - truth.L0Plant;
                stretchMax = std::max(stretchMax, dl);
                slack = slack || dl < 0.0;
            }
            if (hasPayload && X(k)[payOff + 2] < -0.5) {
                ++lifted;
                slackLifted += slack ? 1 : 0;
            }
        }
        put("tether_stretch_max", stretchMax);
        put("tether_tension_max", truth.rig.k_t * std::max(stretchMax, 0.0));
        put("tether_slack_frac_airborne", lifted ? static_cast<double>(slackLifted) / static_cast<double>(lifted) : kNaN);
        put("payload_lifted", lifted > 0 ? 1.0 : 0.0);
    } else {
        for (const char* k : {"tether_stretch_max", "tether_tension_max", "tether_slack_frac_airborne"}) put(k, kNaN);
        put("payload_lifted", 0.0);
    }

    // --- Control effort / saturation. Bounds from the solver config (the
    // same bounds the NLP enforces).
    if (n > 0) {
        const int perUavNu = nu / numUavs;
        size_t satCount = 0, thrustSat = 0;
        for (const auto& u : h.controls) {
            bool anyThrust = false;
            for (int i = 0; i < nu; ++i) {
                const double lb = sc.lbxControls[i], ub = sc.ubxControls[i];
                const double tol = 1e-3 * (ub - lb);
                const bool sat = u[i] <= lb + tol || u[i] >= ub - tol;
                satCount += sat ? 1 : 0;
                if (i % perUavNu == 0) anyThrust = anyThrust || sat;
            }
            thrustSat += anyThrust ? 1 : 0;
        }
        put("sat_frac", static_cast<double>(satCount) / static_cast<double>(n * nu));
        put("thrust_sat_frac", static_cast<double>(thrustSat) / static_cast<double>(n));
        if (n > 1) {
            double s = 0.0;
            for (size_t k = 1; k < n; ++k) {
                for (int i = 0; i < nu; ++i) {
                    const double du = (h.controls[k][i] - h.controls[k - 1][i]) / sc.ubxControls[i];
                    s += du * du;
                }
            }
            put("du_rms", std::sqrt(s / static_cast<double>(n - 1)));
        } else {
            put("du_rms", kNaN);
        }
    } else {
        for (const char* k : {"sat_frac", "thrust_sat_frac", "du_rms"}) put(k, kNaN);
    }

    // --- Solver
    size_t fails = 0;
    size_t miss = 0;
    for (size_t k = 0; k < n; ++k) {
        fails += h.ok[k] ? 0 : 1;
        miss += h.ctrlMs[k] > sc.dt * 1000.0 ? 1 : 0;
    }
    put("fail_rate", n ? static_cast<double>(fails) / static_cast<double>(n) : kNaN);
    put("iter_mean", kNaN);
    put("iter_max", kNaN);
    put("online_ms_mean", mean(h.ctrlMs));
    put("online_ms_p95", pct(h.ctrlMs, 95.0));
    put("online_ms_max", maxOmitNan(h.ctrlMs));
    put("deadline_miss_frac", n ? static_cast<double>(miss) / static_cast<double>(n) : kNaN);
    put("mhe_ms_mean", mean(h.mheMs));
    put("mhe_ms_max", maxOmitNan(h.mheMs));

    // --- Estimation quality (offset-free runs only), second half of the run
    if (useEst && n > 0) {
        const size_t from = std::max<size_t>(1, static_cast<size_t>(std::floor(0.5 * static_cast<double>(n)))) - 1;
        const size_t cnt = n - from;
        double sw = 0.0, sf = 0.0, sb = 0.0;
        for (size_t k = from; k < n; ++k) {
            for (size_t i = 0; i < truth.windTrue.size(); ++i) {
                const double e = h.windHat[k][i] - truth.windTrue[i];
                sw += e * e;
            }
            for (size_t i = 0; i < truth.dTrue.size(); ++i) {
                const double e = h.dHat[k][i] - truth.dTrue[i];
                ((i % 5) < 3 ? sf : sb) += e * e; // per UAV: dF(3), b_roll, b_pitch
            }
        }
        put("wind_err_rms", std::sqrt(sw / static_cast<double>(cnt)));
        put("dF_err_rms", std::sqrt(sf / static_cast<double>(cnt)));
        put("btrim_err_rms_deg", grs::radToDeg(std::sqrt(sb / static_cast<double>(cnt))));
    } else {
        for (const char* k : {"wind_err_rms", "dF_err_rms", "btrim_err_rms_deg"}) put(k, kNaN);
    }
    return m;
}

} // namespace

bool controllerUsesEstimator(const std::string& controller) {
    if (controller == "nmpc_naive") return false;
    if (controller == "nmpc_of") return true;
    throw std::runtime_error("unknown controller '" + controller + "' (nmpc_naive | nmpc_of)");
}

RunResult runClosedLoop(YAML::Node config, const std::string& controller, const std::vector<double>& reference,
                        const TruthSpec& truth, const int sampleId, const RunOptions& opts) {
    const auto wallStart = std::chrono::steady_clock::now();
    const bool useEst = controllerUsesEstimator(controller);

    // Fresh controller/estimator per run, built exactly as the GCS builds them.
    ControlStack stack = buildControlStack(config, useEst);
    if (useEst && !stack.estimatorInstance) {
        throw std::runtime_error(controller + " needs an EstimatorConfiguration section in the YAML");
    }
    const solverConfig& sc = stack.solver;
    const int nx = sc.nx, nu = sc.nu, N = sc.N, numUavs = sc.numUavs;
    const double dt = sc.dt;
    const size_t stride = static_cast<size_t>(nx + nu);

    if (reference.size() % stride != 0) {
        throw std::runtime_error("reference size is not a multiple of nx+nu=" + std::to_string(stride));
    }
    const size_t nPoints = reference.size() / stride;
    if (nPoints <= static_cast<size_t>(N) + 1) throw std::runtime_error("reference shorter than the horizon");

    stack.controller->setReferenceTrajectory(reference);
    std::unique_ptr<EstimatorRunner> runner;
    if (stack.estimatorInstance) {
        runner = std::make_unique<DeferredEstimatorRunner>(*stack.estimatorInstance, 1.0 / dt, opts.nmheFrequency,
                                                           opts.nmheLatencyMs);
    }
    // Declared after `stack`, so destroyed before the estimator it references.
    ControlStep step(*stack.controller, std::move(runner), stack.estimator ? stack.estimator->nu : 0);

    const auto plant = makePlant(truth, numUavs);
    if (plant->nx() != nx || plant->nu() != nu || plant->nd() != sc.nd || plant->np() != sc.np) {
        throw std::runtime_error("truth plant dimensions do not match SolverConfiguration");
    }
    const bool hasPayload = plant->hasPayload();

    size_t steps = nPoints - static_cast<size_t>(N);  // same truncation as the MATLAB sims
    if (opts.tEnd) steps = std::min(steps, static_cast<size_t>(std::lround(*opts.tEnd / dt)));

    std::mt19937_64 rng(static_cast<uint64_t>(opts.seed) * 7919ULL + static_cast<uint64_t>(sampleId));
    std::normal_distribution<double> gauss(0.0, 1.0);

    std::vector<double> xTrue(reference.begin(), reference.begin() + nx);
    History h;
    h.states.push_back(xTrue);

    const size_t trackOff = hasPayload ? static_cast<size_t>(numUavs * kUavBlock) : 0;
    std::string abortReason;
    std::vector<double> u(nu), alpha(numUavs), xdot(nx), xMeas(nx);

    // Command actually acting on the plant, and commands computed but not yet
    // in effect (only with a command delay). Before the first command, the
    // reference's first control (launch trim).
    std::vector<double> uActive(reference.begin() + nx, reference.begin() + nx + nu);
    std::deque<std::pair<double, std::vector<double>>> pendingCmds;
    double lastActivation = 0.0;

    stack.controller->initLaunch();

    for (size_t it = 0; it < steps; ++it) {
        for (int i = 0; i < nx; ++i) {
            xMeas[i] = xTrue[i] + (opts.measNoiseStd > 0.0 ? opts.measNoiseStd * gauss(rng) : 0.0);
        }
        const auto telemetry = toTelemetry(xMeas, numUavs, hasPayload, truth.windTrue);

        std::map<uint8_t, uavCommandsFlags> cmds;
        try {
            cmds = step.tick(telemetry);
        } catch (const std::exception& e) {
            abortReason = std::string("solver_exception: ") + e.what();
            break;
        }

        const int perUavNu = nu / numUavs;
        for (int i = 0; i < numUavs; ++i) {
            const auto& c = cmds.at(static_cast<uint8_t>(i + 1)).commands;
            u[i * perUavNu + 0] = c.thrust;
            u[i * perUavNu + 1] = grs::degToRad(static_cast<double>(c.rollDegree));
            u[i * perUavNu + 2] = grs::degToRad(static_cast<double>(c.pitchDegree));
        }
        if (std::any_of(u.begin(), u.end(), [](const double v) { return !std::isfinite(v); })) {
            abortReason = "nonfinite_control";
            break;
        }

        const auto dbg = stack.controller->getDebugInfo();
        h.controls.push_back(u);
        h.ctrlMs.push_back(dbg.lastSolveMs);
        h.ok.push_back(dbg.lastFlag == 0 && !dbg.violation);
        const EstimatorRunner* er = step.estimatorRunner();
        h.mheMs.push_back(er && er->stats().solvedThisTick ? er->stats().lastSolveMs : kNaN);
        if (er && !step.appliedEstimate().wind.empty()) {
            // The estimate the controller actually used this tick.
            h.windHat.push_back(step.appliedEstimate().wind);
            h.dHat.push_back(step.appliedEstimate().d);
        } else {
            h.windHat.emplace_back(sc.np, 0.0);
            h.dHat.emplace_back(sc.nd, 0.0);
        }

        // Truth plant: sampled params, true wind/d, true L0. The new command
        // takes effect `delay` after this tick's measurement (never before
        // the previous one); until then the previous command is held. With
        // no delay this is one integration over the whole interval.
        const double tk = static_cast<double>(it) * dt;
        const double delay = (opts.cmdDelayMeasured ? dbg.lastSolveMs : opts.cmdDelayMs) / 1000.0;
        const double activation = std::max(tk + std::max(0.0, delay), lastActivation);
        lastActivation = activation;
        pendingCmds.emplace_back(activation, u);
        for (double t = tk, tEnd = tk + dt; t < tEnd - 1e-12;) {
            while (!pendingCmds.empty() && pendingCmds.front().first <= t + 1e-12) {
                uActive = pendingCmds.front().second;
                pendingCmds.pop_front();
            }
            const double tNext = pendingCmds.empty() ? tEnd : std::min(tEnd, pendingCmds.front().first);
            const double seg = tNext - t;
            const int nSeg = std::max(1, static_cast<int>(std::ceil(opts.nSub * seg / dt - 1e-9)));
            integrate(*plant, xTrue, uActive, truth.windTrue, truth.dTrue, truth.L0Plant, seg, nSeg, opts.method);
            t = tNext;
        }
        plant->evaluate(xTrue.data(), uActive.data(), truth.windTrue.data(), truth.dTrue.data(), truth.L0Plant,
                        xdot.data(), alpha.data());
        h.states.push_back(xTrue);
        h.alpha.push_back(alpha);

        // Divergence checks (mc_run_one_twoUav.m)
        if (std::any_of(xTrue.begin(), xTrue.end(), [](const double v) { return !std::isfinite(v); })) {
            abortReason = "nonfinite_state";
            break;
        }
        if (norm3(xTrue.data() + trackOff, reference.data() + (it + 1) * stride + trackOff) > opts.abortErrM) {
            abortReason = hasPayload ? "payload_error_exceeded" : "uav_error_exceeded";
            break;
        }
        bool ground = false;
        for (int i = 0; i < numUavs; ++i) ground = ground || xTrue[i * kUavBlock + 2] > 1.0; // NED: pD > 0 is below ground
        if (ground) {
            abortReason = "uav_ground_impact";
            break;
        }
    }

    const size_t nDone = h.controls.size();
    RunResult r;
    r.metrics = computeMetrics(h, reference, sc, truth, hasPayload, useEst);
    r.completed = abortReason.empty() && nDone == steps;
    r.abortReason = abortReason;
    r.metrics.emplace_back("n_steps_planned", static_cast<double>(steps));
    r.metrics.emplace_back("n_steps_done", static_cast<double>(nDone));
    r.metrics.emplace_back("completed", r.completed ? 1.0 : 0.0);
    r.metrics.emplace_back("wall_s", std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count());

    if (opts.storeTrajDecim > 0) {
        r.trajHeader.emplace_back("t");
        for (int i = 0; i < nx; ++i) r.trajHeader.push_back("x" + std::to_string(i + 1));
        for (int i = 0; i < nu; ++i) r.trajHeader.push_back("u" + std::to_string(i + 1));
        for (int i = 0; i < numUavs; ++i) r.trajHeader.push_back("alpha" + std::to_string(i + 1));
        for (int i = 0; i < sc.np; ++i) r.trajHeader.push_back("wind_hat" + std::to_string(i + 1));
        for (int i = 0; i < sc.nd; ++i) r.trajHeader.push_back("d_hat" + std::to_string(i + 1));
        r.trajHeader.emplace_back("ctrl_ms");
        r.trajHeader.emplace_back("ok");
        // Row k: state at t=k*dt and the control/diagnostics applied from it
        // (NaN on the last state row, which has no control after it).
        for (size_t k = 0; k <= nDone; k += static_cast<size_t>(opts.storeTrajDecim)) {
            std::vector<double> row{static_cast<double>(k) * dt};
            row.insert(row.end(), h.states[k].begin(), h.states[k].end());
            if (k < nDone) {
                row.insert(row.end(), h.controls[k].begin(), h.controls[k].end());
                row.insert(row.end(), h.alpha[k].begin(), h.alpha[k].end());
                row.insert(row.end(), h.windHat[k].begin(), h.windHat[k].end());
                row.insert(row.end(), h.dHat[k].begin(), h.dHat[k].end());
                row.push_back(h.ctrlMs[k]);
                row.push_back(h.ok[k] ? 1.0 : 0.0);
            } else {
                row.insert(row.end(), static_cast<size_t>(nu + numUavs + sc.np + sc.nd + 2), kNaN);
            }
            r.trajRows.push_back(std::move(row));
        }
    }
    return r;
}

} // namespace grs::sim
