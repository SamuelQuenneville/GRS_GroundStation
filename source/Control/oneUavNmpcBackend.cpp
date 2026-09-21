/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneUavNmpcBackend.h"

// Only translation unit that ever includes this header -- see the note in
// OneUavNmpcBackend.h on why that isolation matters.
#include "solver_oneGround_nmpc.h"

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

const char* OneUavNmpcBackend::name() const {
    return "solver_oneGround_nmpc";
}
