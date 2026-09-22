/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef TWOUAVPAYLOADNMHEBACKEND_H
#define TWOUAVPAYLOADNMHEBACKEND_H

#pragma once

#include "estimatorBackend.h"

// Thin wrapper around the codegen'd nmhe_twoUavPayload_* C API -- see
// OneUavNmheBackend.h for the full isolation rationale, mirrored here.
class TwoUavPayloadNmheBackend final : public EstimatorBackend {
public:
    TwoUavPayloadNmheBackend();
    ~TwoUavPayloadNmheBackend() override;

    TwoUavPayloadNmheBackend(const TwoUavPayloadNmheBackend&) = delete;
    TwoUavPayloadNmheBackend& operator=(const TwoUavPayloadNmheBackend&) = delete;

    [[nodiscard]] long long inputSize(int i) const override;
    [[nodiscard]] long long outputSize(int i) const override;
    [[nodiscard]] size_t workIntSize() const override;
    [[nodiscard]] size_t workRealSize() const override;
    int solve(const double** arg, double** res, long long* iw, double* w) override;

    // P_optim layout for build_nmhe_twoUavPayload.m's NLP -- structurally
    // identical to OneUavNmheBackend's (same order), just this config's own
    // (larger) nx/nu/np/nd -- confirmed against the two-UAV builder
    // directly, not assumed to match (same verification standard as
    // TwoUavPayloadNmpcBackend).
    void packParameters(
        const estimatorConfig& config,
        const std::vector<double>& measurementWindow,
        const std::vector<double>& appliedControlWindow,
        const std::vector<double>& windPrior,
        const std::vector<double>& dPrior,
        std::vector<double>& p) const override;

    void packBounds(
        const estimatorConfig& config,
        std::vector<double>& lbx,
        std::vector<double>& ubx) const override;

    [[nodiscard]] const char* name() const override;

private:
    int m_mem = -1;
};

#endif //TWOUAVPAYLOADNMHEBACKEND_H
