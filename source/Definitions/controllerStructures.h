/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLLERSTRUCTURES_H
#define CONTROLLERSTRUCTURES_H

#pragma once

#include <vector>

struct solverConfig {
    int nx;         // JOINT state dimension across all vehicles (all UAVs'
                     // blocks plus the payload's, if present) -- not
                     // per-vehicle. See "State vector layout" in
                     // docs/ARCHITECTURE.md.
    int nu;         // JOINT control dimension across all UAVs -- not
                     // per-vehicle. Same convention as nx above.
    int np;         // wind parameter count (3, shared -- not per UAV)
    int nd;         // disturbance-bias parameter count (from NMHE/EKF; zero
                     // vector until an estimator is wired in -- see the
                     // wind/disturbance gap in gcs-sitl-integration-plan.md)
    int nL0;         // tether rest-length parameter count (1 today)
    int N;          // prediction horizon
    int numUavs;    // number of UAVs in the solver dynamic model
    double dt;      // shooting interval [s] the solver was built with --
                     // bookkeeping only (baked into the codegen'd dynamics
                     // already), but tracked so a config/solver mismatch is
                     // at least visible instead of silently wrong (ADR-001
                     // action item #1).
    double tetherL0; // tether rest length [m], packed into the solver's L0
                     // parameter every solve. Fixed from config for now --
                     // see tether-length-parameterization.md for the
                     // GCS-settable version this will become.
    double alphaMax; // angle-of-attack path-constraint bound [rad],
                     // symmetric -- must match the alpha_max the solver was
                     // built with (build_nlp_oneGround_nmpc.m /
                     // build_nlp_twoUavPayload_nmpc.m's own 'alpha_max'
                     // option; deg2rad(14) one-UAV / deg2rad(25) two-UAV by
                     // MATLAB default, but re-verify per aircraft/profile).
                     // Used to repack g's per-stage alpha inequality rows
                     // in MpcController::m_packInequalityBounds() -- see
                     // that method's comment for why this exists at all.
    // How MpcController picks the reference window start each solve once
    // launched:
    //   Nearest -- (default, the GCS's original behavior) search forward
    //              from the last index for the reference point closest in
    //              north/east to UAV 1, so the reference waits for a vehicle
    //              that falls behind.
    //   Time    -- advance exactly one reference sample per solve, i.e. a
    //              time-indexed reference, as every MATLAB closed-loop sim
    //              does (sim_*_nmpc.m, mc_run_one_twoUav.m). Used by
    //              grs_batchsim to reproduce MATLAB runs; optional YAML key
    //              REFERENCE_INDEXING: nearest | time.
    enum class ReferenceIndexing { Nearest, Time };
    ReferenceIndexing referenceIndexing = ReferenceIndexing::Nearest;

    std::vector<double> weight;
    std::vector<double> lbxStates;
    std::vector<double> ubxStates;
    std::vector<double> lbxControls;
    std::vector<double> ubxControls;
    std::vector<double> scalesStates;
    std::vector<double> scalesControls;
    std::vector<double> invScalesStates;
    std::vector<double> invScalesControls;
};

// Static configuration for one codegen'd NMHE solver (nmhe_oneGround /
// nmhe_twoUavPayload) -- the estimator-side counterpart to solverConfig
// above. Deliberately its own struct, not solverConfig reused: NMHE's
// decision vector is the augmented per-stage state xi_k = [x_k; wind_k;
// d_k] (see nmhe-fatrop-stage-structure.md), its horizon is a sliding
// MEASUREMENT window of length M+1 (not a reference trajectory of length
// N+1), and it has no control decision variables at all (nu=0 in the NLP
// itself -- see build_nmhe_oneGround.m's own header comment) even though
// the GCS still needs to know each UAV's control dimension to size the
// applied-control window it feeds in as a parameter. Kept structurally
// parallel to solverConfig anyway (same field names where the concept
// matches) so the two are easy to read side by side.
struct estimatorConfig {
    int nx;          // per-window state dimension (same convention as
                      // solverConfig::nx -- joint across all vehicles)
    int nu;           // joint control dimension across all UAVs -- sizes
                       // the applied-control window NmheEstimator feeds in;
                       // the NLP itself has no free control decision (nu=0
                       // in the Fatrop stage structure, see above)
    int np;           // wind parameter/state-block count (3, shared)
    int nd;           // disturbance-bias state-block count
    int nL0;          // tether rest-length parameter count (1 today)
    int nxi;          // nx + np + nd -- augmented per-stage state size
    int M;            // estimation window length in steps (M+1 stages)
    int numUavs;
    double dt;        // sample interval [s] the solver was built with --
                       // bookkeeping only, same caveat as solverConfig::dt
    double tetherL0;   // tether rest length [m], packed into L0 every solve

    // Per-stage arrival-cost/measurement-fit weights -- W_meas(nx),
    // W_windp(np), W_dp(nd), matching build_nmhe_*.m's Objective section.
    std::vector<double> wMeas;
    std::vector<double> wWindPrior;
    std::vector<double> wDPrior;

    // Must match the wind_max/dF_max/b_att_max build_nmhe_*.m was exported
    // with; packed into lbx/ubx by NmheEstimator.
    double windMax;
    double dFMax;
    double bAttMax;

    std::vector<double> xScale;
    std::vector<double> windScale;
    std::vector<double> dScale;
    std::vector<double> invXScale;
};

#endif //CONTROLLERSTRUCTURES_H
