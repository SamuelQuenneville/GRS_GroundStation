/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneUavNmheBackend.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>

// Only translation unit that ever includes this header -- see the note in
// OneUavNmheBackend.h on why that isolation matters.
#include "../CasadiSolver/nmhe_oneGround.h"

OneUavNmheBackend::OneUavNmheBackend() {
    m_mem = nmhe_oneGround_checkout();
    nmhe_oneGround_init_mem(m_mem);
}

OneUavNmheBackend::~OneUavNmheBackend() {
    if (m_mem >= 0) {
        nmhe_oneGround_release(m_mem);
        m_mem = -1;
    }
}

long long OneUavNmheBackend::inputSize(const int i) const {
    return nmhe_oneGround_sparsity_in(i)[0];
}

long long OneUavNmheBackend::outputSize(const int i) const {
    return nmhe_oneGround_sparsity_out(i)[0];
}

size_t OneUavNmheBackend::workIntSize() const {
    return nmhe_oneGround_SZ_IW;
}

size_t OneUavNmheBackend::workRealSize() const {
    return nmhe_oneGround_SZ_W;
}

int OneUavNmheBackend::solve(const double** arg, double** res, long long* iw, double* w) {
    return nmhe_oneGround(
        reinterpret_cast<const casadi_real**>(arg),
        reinterpret_cast<casadi_real**>(res),
        reinterpret_cast<casadi_int*>(iw),
        reinterpret_cast<casadi_real*>(w),
        m_mem);
}

void OneUavNmheBackend::packParameters(
    const estimatorConfig& config,
    const std::vector<double>& measurementWindow,
    const std::vector<double>& appliedControlWindow,
    const std::vector<double>& windPrior,
    const std::vector<double>& dPrior,
    std::vector<double>& p) const
{
    // Layout must match build_nmhe_oneGround.m's P_optim order exactly --
    // see the header comment: per-stage [Xmeas_k, Uapp_k] interleaved for
    // k=1..M (Xmeas_{M+1} alone closes the window, no U_applied at the
    // last stage -- there's nothing left to apply it to), then the
    // arrival-cost prior and weights, then L0.

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
    // Stage M+1 (index M): measurement only, no applied control.
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

void OneUavNmheBackend::packBounds(
    const estimatorConfig& config,
    std::vector<double>& lbx,
    std::vector<double>& ubx) const
{
    // x-part unbounded, matching build_nmhe_oneGround.m's own
    // "lbx = [lbx; -inf(nx,1); lb_wind_d]" -- state estimates are
    // corrected by the measurement-fit cost, not a box bound.
    //
    // wind/d-part: [-wind_max(np); -dF_max; -dF_max; -dF_max; -b_att_max;
    // -b_att_max] ./ [wind_scale; d_scale] -- mirrors that file's
    // lb_wind_d/ub_wind_d exactly (nd=5: dFx, dFy, dFz, b_roll, b_pitch).
    const double invWindScale = config.windScale.empty() ? 1.0 : 1.0 / config.windScale[0];

    std::vector<double> lbWindD(config.np + config.nd);
    std::vector<double> ubWindD(config.np + config.nd);

    for (int i = 0; i < config.np; ++i) {
        lbWindD[i] = -config.windMax * invWindScale;
        ubWindD[i] =  config.windMax * invWindScale;
    }

    const double dFBound[5] = {config.dFMax, config.dFMax, config.dFMax, config.bAttMax, config.bAttMax};
    for (int i = 0; i < config.nd && i < 5; ++i) {
        const double invScale = config.dScale.empty() ? 1.0 : 1.0 / config.dScale[i % config.dScale.size()];
        lbWindD[config.np + i] = -dFBound[i] * invScale;
        ubWindD[config.np + i] =  dFBound[i] * invScale;
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

const char* OneUavNmheBackend::name() const {
    return "nmhe_oneGround";
}
