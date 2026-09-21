/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ONEUAVNMPCBACKEND_H
#define ONEUAVNMPCBACKEND_H

#pragma once

#include "solverBackend.h"

// Thin wrapper around the codegen'd solver_oneGround_nmpc_* C API
// (source/Control/solver_oneGround_nmpc.c/.h -- manually copied from
// GRS_Controller/03_generated/01_oneUav/nmpc/, see the repo-vendoring
// decision in gcs-sitl-integration-plan.md). Replaces the retired
// solver_oneGround.c/.h (bare `solver`/`solver_checkout`/... symbols
// despite the file's own name). Only this .cpp includes
// solver_oneGround_nmpc.h, so its macros (solver_oneGround_nmpc_SZ_IW,
// ...) and global symbols never reach the rest of the codebase -- that
// isolation is what lets a second, differently-named backend
// (TwoUavPayloadNmpcBackend, Phase 3) coexist in the same binary.
class OneUavNmpcBackend final : public SolverBackend {
public:
    OneUavNmpcBackend();
    ~OneUavNmpcBackend() override;

    OneUavNmpcBackend(const OneUavNmpcBackend&) = delete;
    OneUavNmpcBackend& operator=(const OneUavNmpcBackend&) = delete;

    [[nodiscard]] long long inputSize(int i) const override;
    [[nodiscard]] long long outputSize(int i) const override;
    [[nodiscard]] size_t workIntSize() const override;
    [[nodiscard]] size_t workRealSize() const override;
    int solve(const double** arg, double** res, long long* iw, double* w) override;

    // P_optim layout for build_nlp_oneGround_nmpc.m's NLP:
    //   [x0_ref; {X_ref_k, U_ref_k}_{k=1..N}, X_ref_{N+1};
    //    Wind_est(np); D_est(nd); Weight(nx+nu+nx+nu+nu); U_prev(nu); L0(nL0)]
    // (see the "Parameter vector layout" comment at the top of that file).
    void packParameters(
        const solverConfig& config,
        const std::vector<double>& initialStates,
        const std::vector<double>& referenceTrajectory,
        size_t refOffset,
        const std::vector<double>& uPrev,
        std::vector<double>& p) const override;

    void packBounds(
        const solverConfig& config,
        std::vector<double>& lbx,
        std::vector<double>& ubx) const override;

    [[nodiscard]] const char* name() const override;

private:
    int m_mem = -1;
};

#endif //ONEUAVNMPCBACKEND_H
