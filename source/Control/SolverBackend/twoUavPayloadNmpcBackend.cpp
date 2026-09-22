/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "twoUavPayloadNmpcBackend.h"

#include <algorithm>
#include <cassert>
#include <cstring>

// Only translation unit that ever includes this header -- see the note in
// TwoUavPayloadNmpcBackend.h on why that isolation matters.
#include "../CasadiSolver/solver_twoUavPayload_nmpc.h"

TwoUavPayloadNmpcBackend::TwoUavPayloadNmpcBackend() {
    m_mem = solver_twoUavPayload_nmpc_checkout();
    solver_twoUavPayload_nmpc_init_mem(m_mem);
}

TwoUavPayloadNmpcBackend::~TwoUavPayloadNmpcBackend() {
    if (m_mem >= 0) {
        solver_twoUavPayload_nmpc_release(m_mem);
        m_mem = -1;
    }
}

long long TwoUavPayloadNmpcBackend::inputSize(const int i) const {
    return solver_twoUavPayload_nmpc_sparsity_in(i)[0];
}

long long TwoUavPayloadNmpcBackend::outputSize(const int i) const {
    return solver_twoUavPayload_nmpc_sparsity_out(i)[0];
}

size_t TwoUavPayloadNmpcBackend::workIntSize() const {
    return solver_twoUavPayload_nmpc_SZ_IW;
}

size_t TwoUavPayloadNmpcBackend::workRealSize() const {
    return solver_twoUavPayload_nmpc_SZ_W;
}

int TwoUavPayloadNmpcBackend::solve(const double** arg, double** res, long long* iw, double* w) {
    return solver_twoUavPayload_nmpc(
        reinterpret_cast<const casadi_real**>(arg),
        reinterpret_cast<casadi_real**>(res),
        reinterpret_cast<casadi_int*>(iw),
        reinterpret_cast<casadi_real*>(w),
        m_mem);
}

void TwoUavPayloadNmpcBackend::packParameters(
    const solverConfig& config,
    const std::vector<double>& initialStates,
    const std::vector<double>& referenceTrajectory,
    const size_t refOffset,
    const std::vector<double>& uPrev,
    std::vector<double>& p) const
{
    // Layout must match build_nlp_twoUavPayload_nmpc.m's P_optim order
    // exactly -- see the header comment. Structurally identical to
    // OneUavNmpcBackend::packParameters(), just driven by this config's own
    // (larger) nx/nu/np/nd/nL0 -- confirmed against the two-UAV builder
    // directly, not assumed to match.

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

    // Wind_est -- shared by both UAVs, TODO from NMHE later (see
    // gcs-sitl-integration-plan.md §2/Phase 4).
    std::fill_n(dst + offset, config.np, 0.0);
    offset += config.np;

    // D_est -- [d_uav1(5); d_uav2(5)], same TODO as wind.
    std::fill_n(dst + offset, config.nd, 0.0);
    offset += config.nd;

    // Weight = [Q(nx); R(nu); Qf(nx); Rdu(nu); Rdu0(nu)]
    std::memcpy(dst + offset, config.weight.data(), config.weight.size() * sizeof(double));
    offset += config.weight.size();

    // U_prev, physical units, [T1,roll1,pitch1,T2,roll2,pitch2].
    std::memcpy(dst + offset, uPrev.data(), nu * sizeof(double));
    offset += nu;

    // L0 (tether rest length, shared by both tethers), physical units.
    std::fill_n(dst + offset, config.nL0, config.tetherL0);
    offset += config.nL0;

    assert(offset == p.size());
}

void TwoUavPayloadNmpcBackend::packBounds(
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

const char* TwoUavPayloadNmpcBackend::name() const {
    return "solver_twoUavPayload_nmpc";
}
