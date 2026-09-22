/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ESTIMATORBACKEND_H
#define ESTIMATORBACKEND_H

#pragma once

#include <cstddef>
#include <vector>

#include "Definitions/controllerStructures.h"

// Estimator-side counterpart to SolverBackend (see that header's own
// comment for the full rationale -- this repeats only what differs).
// Abstracts one codegen'd NMHE solver's C API (nmhe_oneGround,
// nmhe_twoUavPayload) so NmheEstimator can be built against whichever one
// a config selects at startup without calling that solver's global C
// symbols directly, and without NmheEstimator ever needing to know this
// NLP's parameter-vector layout.
//
// Deliberately NOT a SolverBackend: NMHE's parameter vector packs a
// sliding MEASUREMENT window (M+1 states, M applied controls) plus a
// wind/d arrival-cost prior, not a reference trajectory -- a genuinely
// different shape from SolverBackend::packParameters()'s NMPC-shaped
// signature (initialStates/referenceTrajectory/refOffset/uPrev), so
// forcing NMHE through that interface would mean padding it with unused
// arguments instead of describing what this NLP actually needs. The raw
// nlpsol C-call mechanics (8-in/6-out, workspace sizing) are identical in
// shape to SolverBackend's, just duplicated here rather than factored into
// a shared base -- two small, independently-readable interfaces beat one
// interface with parts that don't apply to half its implementations.
class EstimatorBackend {
public:
    virtual ~EstimatorBackend() = default;

    // Length (in doubles) of nlpsol input i (0=x0 .. 7=lam_g0).
    [[nodiscard]] virtual long long inputSize(int i) const = 0;

    // Length (in doubles) of nlpsol output i (0=x .. 5=lam_p).
    [[nodiscard]] virtual long long outputSize(int i) const = 0;

    [[nodiscard]] virtual size_t workIntSize() const = 0;
    [[nodiscard]] virtual size_t workRealSize() const = 0;

    // Same fixed 8-in/6-out nlpsol convention SolverBackend::solve() uses.
    virtual int solve(const double** arg, double** res, long long* iw, double* w) = 0;

    // Fills `p` (already sized to inputSize(1)) with this NLP's parameter
    // vector for one solve.
    //   config               -- static estimator configuration (nx, np,
    //                           nd, nL0, M, weights, tetherL0, ...)
    //   measurementWindow     -- (M+1)*nx, oldest-to-newest, physical units
    //   appliedControlWindow  -- M*nu, oldest-to-newest, physical units --
    //                           appliedControlWindow[k] is the control
    //                           applied going from measurementWindow[k] to
    //                           measurementWindow[k+1]
    //   windPrior/dPrior      -- arrival-cost prior (this window's Wind_prior/
    //                           D_prior), physical units, length np/nd --
    //                           NmheEstimator's job to carry forward from
    //                           the previous solve, not this backend's
    virtual void packParameters(
        const estimatorConfig& config,
        const std::vector<double>& measurementWindow,
        const std::vector<double>& appliedControlWindow,
        const std::vector<double>& windPrior,
        const std::vector<double>& dPrior,
        std::vector<double>& p) const = 0;

    // Fills `lbx`/`ubx` (already sized to inputSize(2)/inputSize(3)) with
    // this NLP's decision-variable bounds, tiled across all M+1 stages:
    // x-part unbounded (-inf/inf, matching build_nmhe_*.m's own bounds --
    // state estimates are corrected by the measurement-fit cost, not a box
    // bound), wind/d-part from config.windMax/dFMax/bAttMax.
    virtual void packBounds(
        const estimatorConfig& config,
        std::vector<double>& lbx,
        std::vector<double>& ubx) const = 0;

    // Identifier for logging/debug panels (e.g. "nmhe_oneGround").
    [[nodiscard]] virtual const char* name() const = 0;
};

#endif //ESTIMATORBACKEND_H
