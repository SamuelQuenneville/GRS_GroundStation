/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneUavNmpcBackend.h"

#include <algorithm>
#include <cassert>
#include <cstring>

// Only translation unit that ever includes this header -- see the note in
// OneUavNmpcBackend.h on why that isolation matters.
#include "../CasadiSolver/solver_oneGround_nmpc.h"

OneUavNmpcBackend::OneUavNmpcBackend() {
    m_mem = solver_oneGround_nmpc_checkout();
    solver_oneGround_nmpc_init_mem(m_mem);
}

OneUavNmpcBackend::~OneUavNmpcBackend() {
    if (m_mem >= 0) {
        solver_oneGround_nmpc_release(m_mem);
        m_mem = -1;
    }
}

long long OneUavNmpcBackend::inputSize(const int i) const {
    return solver_oneGround_nmpc_sparsity_in(i)[0];
}

long long OneUavNmpcBackend::outputSize(const int i) const {
    return solver_oneGround_nmpc_sparsity_out(i)[0];
}

size_t OneUavNmpcBackend::workIntSize() const {
    return solver_oneGround_nmpc_SZ_IW;
}

size_t OneUavNmpcBackend::workRealSize() const {
    return solver_oneGround_nmpc_SZ_W;
}

int OneUavNmpcBackend::solve(const double** arg, double** res, long long* iw, double* w) {
    return solver_oneGround_nmpc(
        reinterpret_cast<const casadi_real**>(arg),
        reinterpret_cast<casadi_real**>(res),
        reinterpret_cast<casadi_int*>(iw),
        reinterpret_cast<casadi_real*>(w),
        m_mem);
}

void OneUavNmpcBackend::packParameters(
    const solverConfig& config,
    const std::vector<double>& initialStates,
    const std::vector<double>& referenceTrajectory,
    const size_t refOffset,
    const std::vector<double>& uPrev,
    std::vector<double>& p) const
{
    // Layout must match build_nlp_oneGround_nmpc.m's P_optim order exactly
    // -- see the header comment. This is a different, LONGER layout than
    // the retired solver_oneGround expected -- D_est, U_prev and L0 did not
    // exist as parameters before.

    size_t offset = 0;

    const size_t nx = config.nx;
    const size_t nu = config.nu;
    const size_t stride = nx + nu;
    const size_t N = config.N;

    double* dst = p.data();

    // x_initial
    std::memcpy(dst + offset, initialStates.data(), nx * sizeof(double));
    offset += nx;

    // copy [x0 u0 ... xN-1 uN-1 xN] starting at this solve's window
    const size_t count = N * stride + nx;
    std::memcpy(dst + offset, referenceTrajectory.data() + refOffset, count * sizeof(double));
    offset += count;

    // Wind_est -- TODO Wind could come from an estimator later on (no NMHE
    // wired into the GCS yet, see gcs-sitl-integration-plan.md §2).
    std::fill_n(dst + offset, config.np, 0.0);
    offset += config.np;

    // D_est -- same TODO as wind: zero until NMHE exists.
    std::fill_n(dst + offset, config.nd, 0.0);
    offset += config.nd;

    // Weight = [Q(nx); R(nu); Qf(nx); Rdu(nu); Rdu0(nu)]
    std::memcpy(dst + offset, config.weight.data(), config.weight.size() * sizeof(double));
    offset += config.weight.size();

    // U_prev, physical units [T, roll, pitch] -- see MpcController's
    // m_uPrev member comment for why this is tracked.
    std::memcpy(dst + offset, uPrev.data(), nu * sizeof(double));
    offset += nu;

    // L0 (tether rest length), physical units.
    std::fill_n(dst + offset, config.nL0, config.tetherL0);
    offset += config.nL0;

    assert(offset == p.size());
}

void OneUavNmpcBackend::packBounds(
    const solverConfig& config,
    std::vector<double>& lbx,
    std::vector<double>& ubx) const
{
    std::vector<double> lbxStates(config.nx);
    std::vector<double> ubxStates(config.nx);
    for (size_t i = 0; i < static_cast<size_t>(config.nx); ++i) {
        lbxStates[i] = config.lbxStates[i] * config.invScalesStates[i];
        ubxStates[i] = config.ubxStates[i] * config.invScalesStates[i];
    }

    std::vector<double> lbxControls(config.nu);
    std::vector<double> ubxControls(config.nu);
    for (size_t i = 0; i < static_cast<size_t>(config.nu); ++i) {
        lbxControls[i] = config.lbxControls[i] * config.invScalesControls[i];
        ubxControls[i] = config.ubxControls[i] * config.invScalesControls[i];
    }

    size_t offset = 0;

    for (size_t k = 0; k <= static_cast<size_t>(config.N); ++k) {
        std::ranges::copy(lbxStates, lbx.begin() + static_cast<std::ptrdiff_t>(offset));
        std::ranges::copy(ubxStates, ubx.begin() + static_cast<std::ptrdiff_t>(offset));

        offset += lbxStates.size();

        if (k < static_cast<size_t>(config.N)) {
            std::ranges::copy(lbxControls, lbx.begin() + static_cast<std::ptrdiff_t>(offset));
            std::ranges::copy(ubxControls, ubx.begin() + static_cast<std::ptrdiff_t>(offset));

            offset += lbxControls.size();
        }
    }

    assert(offset == lbx.size());
}

const char* OneUavNmpcBackend::name() const {
    return "solver_oneGround_nmpc";
}
