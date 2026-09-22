/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ONEUAVNMHEBACKEND_H
#define ONEUAVNMHEBACKEND_H

#pragma once

#include "estimatorBackend.h"

// Thin wrapper around the codegen'd nmhe_oneGround_* C API
// (source/Control/CasadiSolver/nmhe_oneGround.c/.h -- manually copied from
// GRS_Controller/03_generated/01_oneUav/nmhe/, same repo-vendoring
// convention as OneUavNmpcBackend). Only this .cpp includes
// nmhe_oneGround.h, for the same symbol-isolation reason OneUavNmpcBackend
// only includes solver_oneGround_nmpc.h -- see that header's comment.
class OneUavNmheBackend final : public EstimatorBackend {
public:
    OneUavNmheBackend();
    ~OneUavNmheBackend() override;

    OneUavNmheBackend(const OneUavNmheBackend&) = delete;
    OneUavNmheBackend& operator=(const OneUavNmheBackend&) = delete;

    [[nodiscard]] long long inputSize(int i) const override;
    [[nodiscard]] long long outputSize(int i) const override;
    [[nodiscard]] size_t workIntSize() const override;
    [[nodiscard]] size_t workRealSize() const override;
    int solve(const double** arg, double** res, long long* iw, double* w) override;

    // P_optim layout for build_nmhe_oneGround.m's NLP:
    //   [{Xmeas_k, (Uapp_k if k<=M)}_{k=1..M+1}; Wind_prior; D_prior;
    //    W_meas; W_windp; W_dp; L0_param]
    // (see that file's own P_optim assembly).
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

#endif //ONEUAVNMHEBACKEND_H
