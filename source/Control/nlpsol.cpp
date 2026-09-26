/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "nlpsol.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "CasadiSolver/nmhe_oneGround.h"
#include "CasadiSolver/nmhe_twoUavPayload.h"
#include "CasadiSolver/solver_oneGround_nmpc.h"
#include "CasadiSolver/solver_twoUavPayload_nmpc.h"

struct NlpsolApi {
    const char* name;
    int (*eval)(const casadi_real**, casadi_real**, casadi_int*, casadi_real*, int);
    int (*checkout)();
    void (*release)(int);
    int (*initMem)(int);
    const casadi_int* (*sparsityIn)(casadi_int);
    const casadi_int* (*sparsityOut)(casadi_int);
    int (*work)(casadi_int*, casadi_int*, casadi_int*, casadi_int*);
};

#define NLPSOL_API(f) NlpsolApi{#f, f, f##_checkout, f##_release, f##_init_mem, f##_sparsity_in, f##_sparsity_out, f##_work}

namespace {

const NlpsolApi& findApi(const Nlpsol::Problem problem, const int numUavs) {
    static constexpr NlpsolApi nmpc[] = {NLPSOL_API(solver_oneGround_nmpc), NLPSOL_API(solver_twoUavPayload_nmpc)};
    static constexpr NlpsolApi nmhe[] = {NLPSOL_API(nmhe_oneGround), NLPSOL_API(nmhe_twoUavPayload)};

    if (numUavs < 1 || numUavs > 2) {
        throw std::runtime_error("Nlpsol: no generated solver for numUavs=" + std::to_string(numUavs));
    }
    return (problem == Nlpsol::Problem::Nmpc ? nmpc : nmhe)[numUavs - 1];
}

} // namespace

Nlpsol::Nlpsol(const Problem problem, const int numUavs)
    : m_api(findApi(problem, numUavs))
    , m_mem(m_api.checkout())
{
    m_api.initMem(m_mem);

    std::vector<double>* in[] = {&x0, &p, &lbx, &ubx, &lbg, &ubg, &lamX0, &lamG0};
    for (int i = 0; i < 8; ++i) in[i]->assign(m_api.sparsityIn(i)[0], 0.0);
    std::vector<double>* out[] = {&x, &f, &g, &lamX, &lamG, &lamP};
    for (int i = 0; i < 6; ++i) out[i]->assign(m_api.sparsityOut(i)[0], 0.0);

    // arg/res are also scratch for the generated code, hence longer than 8/6.
    casadi_int szArg, szRes, szIw, szW;
    m_api.work(&szArg, &szRes, &szIw, &szW);
    m_arg.resize(szArg);
    m_res.resize(szRes);
    m_iw.resize(szIw);
    m_w.resize(szW);
}

Nlpsol::~Nlpsol() {
    m_api.release(m_mem);
}

int Nlpsol::solve() {
    std::ranges::copy(std::initializer_list<const double*>{
        x0.data(), p.data(), lbx.data(), ubx.data(), lbg.data(), ubg.data(), lamX0.data(), lamG0.data()}, m_arg.begin());
    std::ranges::copy(std::initializer_list<double*>{
        x.data(), f.data(), g.data(), lamX.data(), lamG.data(), lamP.data()}, m_res.begin());
    return m_api.eval(m_arg.data(), m_res.data(), m_iw.data(), m_w.data(), m_mem);
}

Nlpsol::Check Nlpsol::check(const int flag, const double feasTol) const {
    Check c;
    if (flag != 0) {
        return c;
    }

    double worst = 0.0;
    for (size_t i = 0; i < g.size(); ++i) {
        worst = std::max({worst, lbg[i] - g[i], g[i] - ubg[i]});
    }
    c.maxConstraintViolation = worst;

    // std::max() ignores NaN, so non-finite values are tested separately.
    const auto finite = [](const std::vector<double>& v) {
        return std::ranges::all_of(v, [](const double e) { return std::isfinite(e); });
    };
    c.valid = worst <= feasTol && finite(g) && finite(x) && !f.empty() && std::isfinite(f[0]);
    return c;
}

const char* Nlpsol::name() const {
    return m_api.name;
}
