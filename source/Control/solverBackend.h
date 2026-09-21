/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef SOLVERBACKEND_H
#define SOLVERBACKEND_H

#pragma once

#include <cstddef>

// Abstraction over one CasADi/Fatrop-codegen'd NLP solver (nlpsol), so
// NMPCController (and a future NMHE estimator) can be built against
// whichever solver a config selects at startup without calling that
// solver's global C symbols directly.
//
// Why this exists: every nlpsol(...) call in GRS_Controller historically
// used the same internal name ('solver'), so every codegen'd solver
// exported IDENTICAL global C symbols (solver(), solver_checkout(), ...)
// -- even after the .c/.h FILE was renamed (solver_oneGround.c here still
// exports bare `solver`/`solver_checkout`/... internally). Two such static
// libraries cannot link into one binary. That's fixed on the MATLAB side
// now (each solver has a distinct nlpsol name -- see
// gcs-sitl-integration-plan.md §3), but NMPCController itself was still
// written to call one hardcoded solver's bare global functions and macros
// directly. This interface is the small piece that was actually missing:
// each concrete backend's .cpp includes only its own generated header
// (solver_oneGround_nmpc.h, solver_twoUavPayload_nmpc.h, ...) so those
// now-distinct macro/symbol names never collide at the C++ level either,
// and NMPCController talks only to this interface.
//
// Every concrete backend wraps the same standard `nlpsol` C API: 8 dense
// inputs (0=x0, 1=p, 2=lbx, 3=ubx, 4=lbg, 5=ubg, 6=lam_x0, 7=lam_g0) and
// 6 dense outputs (0=x, 1=f, 2=g, 3=lam_x, 4=lam_g, 5=lam_p), in that fixed
// order -- that's part of what `nlpsol(...)` always generates, not a
// per-backend choice, so callers can rely on it.
class SolverBackend {
public:
    virtual ~SolverBackend() = default;

    // Length (in doubles) of nlpsol input i (0=x0 .. 7=lam_g0).
    [[nodiscard]] virtual long long inputSize(int i) const = 0;

    // Length (in doubles) of nlpsol output i (0=x .. 5=lam_p).
    [[nodiscard]] virtual long long outputSize(int i) const = 0;

    // Workspace sizes this backend's solve() needs -- size the caller's
    // iw/w buffers to these once, before the first solve().
    [[nodiscard]] virtual size_t workIntSize() const = 0;
    [[nodiscard]] virtual size_t workRealSize() const = 0;

    // Run one solve. arg/res must be 8/6-element pointer arrays already
    // bound to caller-owned buffers sized per inputSize()/outputSize();
    // iw/w are caller-owned scratch sized per workIntSize()/workRealSize().
    // Returns the raw solver return-status flag (0 == converged, matching
    // NMPCController's existing m_solutionIsValid() check).
    virtual int solve(const double** arg, double** res, long long* iw, double* w) = 0;

    // Identifier for logging/debug panels (e.g. "solver_oneGround_nmpc").
    [[nodiscard]] virtual const char* name() const = 0;
};

#endif //SOLVERBACKEND_H
