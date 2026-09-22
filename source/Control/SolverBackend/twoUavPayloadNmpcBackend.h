/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef TWOUAVPAYLOADNMPCBACKEND_H
#define TWOUAVPAYLOADNMPCBACKEND_H

#pragma once

#include "solverBackend.h"

// Thin wrapper around the codegen'd solver_twoUavPayload_nmpc_* C API
// (source/Control/CasadiSolver/solver_twoUavPayload_nmpc.c/.h -- manually
// copied from GRS_Controller/03_generated/02_twoUav/nmpc/, same
// repo-vendoring convention as OneUavNmpcBackend). Only this .cpp includes
// solver_twoUavPayload_nmpc.h, for the same symbol-isolation reason
// documented on OneUavNmpcBackend.
//
// build_nlp_twoUavPayload_nmpc.m's P_optim layout is structurally IDENTICAL
// to build_nlp_oneGround_nmpc.m's -- same order (x0_ref; strided reference;
// Wind_est; D_est; Weight; U_prev; L0), just with two-UAV-sized dims
// (nx=22 = [UAV1(8); UAV2(8); payload(6)], nu=6 = [UAV1(3); UAV2(3)], np=3
// shared wind, nd=10 = [d_uav1(5); d_uav2(5)], nL0=1) -- confirmed directly
// against that builder's own P_optim assembly and
// solver_twoUavPayload_nmpc_meta.h, not assumed to carry over (see
// gcs-sitl-integration-plan.md Phase 3). Because OneUavNmpcBackend's
// packParameters()/packBounds() were already written generically off
// config.nx/nu/np/nd/nL0/N rather than hardcoded one-UAV constants, the
// packing logic itself is identical between the two backends -- only the
// wrapped solver's C symbols differ. Duplicated here rather than shared via
// a common base, since a future backend with a genuinely different P_optim
// order (a different controller family, e.g. LMPC) would otherwise have to
// awkwardly opt out of inherited packing logic that no longer applies to it.
class TwoUavPayloadNmpcBackend final : public SolverBackend {
public:
    TwoUavPayloadNmpcBackend();
    ~TwoUavPayloadNmpcBackend() override;

    TwoUavPayloadNmpcBackend(const TwoUavPayloadNmpcBackend&) = delete;
    TwoUavPayloadNmpcBackend& operator=(const TwoUavPayloadNmpcBackend&) = delete;

    [[nodiscard]] long long inputSize(int i) const override;
    [[nodiscard]] long long outputSize(int i) const override;
    [[nodiscard]] size_t workIntSize() const override;
    [[nodiscard]] size_t workRealSize() const override;
    int solve(const double** arg, double** res, long long* iw, double* w) override;

    // P_optim layout for build_nlp_twoUavPayload_nmpc.m's NLP:
    //   [x0_ref; {X_ref_k, U_ref_k}_{k=1..N}, X_ref_{N+1};
    //    Wind_est(np=3); D_est(nd=10); Weight(nx+nu+nx+nu+nu); U_prev(nu=6); L0(nL0=1)]
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

#endif //TWOUAVPAYLOADNMPCBACKEND_H
