/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "twoUavPayloadNmheBackend.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>

// Only translation unit that ever includes this header -- see the note in
// TwoUavPayloadNmheBackend.h on why that isolation matters.
#include "../CasadiSolver/nmhe_twoUavPayload.h"

TwoUavPayloadNmheBackend::TwoUavPayloadNmheBackend() {
    m_mem = nmhe_twoUavPayload_checkout();
    nmhe_twoUavPayload_init_mem(m_mem);
}

TwoUavPayloadNmheBackend::~TwoUavPayloadNmheBackend() {
    if (m_mem >= 0) {
        nmhe_twoUavPayload_release(m_mem);
        m_mem = -1;
    }
}

long long TwoUavPayloadNmheBackend::inputSize(const int i) const {
    return nmhe_twoUavPayload_sparsity_in(i)[0];
}

long long TwoUavPayloadNmheBackend::outputSize(const int i) const {
    return nmhe_twoUavPayload_sparsity_out(i)[0];
}

size_t TwoUavPayloadNmheBackend::workIntSize() const {
    return nmhe_twoUavPayload_SZ_IW;
}

size_t TwoUavPayloadNmheBackend::workRealSize() const {
    return nmhe_twoUavPayload_SZ_W;
}

int TwoUavPayloadNmheBackend::solve(const double** arg, double** res, long long* iw, double* w) {
    return nmhe_twoUavPayload(
        reinterpret_cast<const casadi_real**>(arg),
        reinterpret_cast<casadi_real**>(res),
        reinterpret_cast<casadi_int*>(iw),
        reinterpret_cast<casadi_real*>(w),
        m_mem);
}

void TwoUavPayloadNmheBackend::packParameters(
    const estimatorConfig& config,
    const std::vector<double>& measurementWindow,
    const std::vector<double>& appliedControlWindow,
    const std::vector<double>& windPrior,
    const std::vector<double>& dPrior,
    std::vector<double>& p) const
{
    // Same interleaved layout as OneUavNmheBackend -- see that .cpp's
    // comment.
    size_t offset = 0;
    double* dst = p.data();

    const size_t nx = config.nx;
    const size_t nu = config.nu;
    const size_t M  = config.M;

    for (size_t k = 0; k < M; ++k) {
        std::memcpy(dst + offset, measurementWindow.data() + k * nx, nx * sizeof(double));
        offset += nx;

        std::memcpy(dst + offset, appliedControlWindow.data() + k * nu, nu * sizeof(double));
        offset += nu;
    }
    std::memcpy(dst + offset, measurementWindow.data() + M * nx, nx * sizeof(double));
    offset += nx;

    std::memcpy(dst + offset, windPrior.data(), config.np * sizeof(double));
    offset += config.np;

    std::memcpy(dst + offset, dPrior.data(), config.nd * sizeof(double));
    offset += config.nd;

    std::memcpy(dst + offset, config.wMeas.data(), config.wMeas.size() * sizeof(double));
    offset += config.wMeas.size();

    std::memcpy(dst + offset, config.wWindPrior.data(), config.wWindPrior.size() * sizeof(double));
    offset += config.wWindPrior.size();

    std::memcpy(dst + offset, config.wDPrior.data(), config.wDPrior.size() * sizeof(double));
    offset += config.wDPrior.size();

    std::fill_n(dst + offset, config.nL0, config.tetherL0);
    offset += config.nL0;

    assert(offset == p.size());
}

void TwoUavPayloadNmheBackend::packBounds(
    const estimatorConfig& config,
    std::vector<double>& lbx,
    std::vector<double>& ubx) const
{
    // wind/d-part: [-wind_max(np); lb_dF_battp; lb_dF_battp] ./
    // [wind_scale; d_scale] -- mirrors build_nmhe_twoUavPayload.m's own
    // lb_wind_d/ub_wind_d exactly: the SAME 5-entry [dFx,dFy,dFz,broll,
    // bpitch] block repeated once per UAV (nd=10 = 2*5), not two
    // independently-tuned blocks.
    const double invWindScale = config.windScale.empty() ? 1.0 : 1.0 / config.windScale[0];

    std::vector<double> lbWindD(config.np + config.nd);
    std::vector<double> ubWindD(config.np + config.nd);

    for (int i = 0; i < config.np; ++i) {
        lbWindD[i] = -config.windMax * invWindScale;
        ubWindD[i] =  config.windMax * invWindScale;
    }

    const double dFBlock[5] = {config.dFMax, config.dFMax, config.dFMax, config.bAttMax, config.bAttMax};
    for (int i = 0; i < config.nd; ++i) {
        const double invScale = config.dScale.empty() ? 1.0 : 1.0 / config.dScale[i % config.dScale.size()];
        lbWindD[config.np + i] = -dFBlock[i % 5] * invScale;
        ubWindD[config.np + i] =  dFBlock[i % 5] * invScale;
    }

    size_t offset = 0;
    for (int k = 0; k <= config.M; ++k) {
        for (int i = 0; i < config.nx; ++i) {
            lbx[offset + i] = -std::numeric_limits<double>::infinity();
            ubx[offset + i] =  std::numeric_limits<double>::infinity();
        }
        offset += config.nx;

        std::ranges::copy(lbWindD, lbx.begin() + static_cast<std::ptrdiff_t>(offset));
        std::ranges::copy(ubWindD, ubx.begin() + static_cast<std::ptrdiff_t>(offset));
        offset += lbWindD.size();
    }

    assert(offset == lbx.size());
}

const char* TwoUavPayloadNmheBackend::name() const {
    return "nmhe_twoUavPayload";
}
