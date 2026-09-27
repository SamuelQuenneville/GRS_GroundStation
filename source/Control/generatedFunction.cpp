/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "generatedFunction.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "CasadiSolver/linearize_oneGround_lmpc.h"
#include "CasadiSolver/linearize_twoUavPayload_lmpc.h"
#include "CasadiSolver/nmhe_oneGround.h"
#include "CasadiSolver/nmhe_twoUavPayload.h"
#include "CasadiSolver/solver_oneGround_lmpc.h"
#include "CasadiSolver/solver_oneGround_nmpc.h"
#include "CasadiSolver/solver_twoUavPayload_lmpc.h"
#include "CasadiSolver/solver_twoUavPayload_nmpc.h"

struct GeneratedApi {
    const char* name;
    int (*eval)(const casadi_real**, casadi_real**, casadi_int*, casadi_real*, int);
    int (*checkout)();
    void (*release)(int);
    int (*initMem)(int);
    const casadi_int* (*sparsityIn)(casadi_int);
    const casadi_int* (*sparsityOut)(casadi_int);
    int (*work)(casadi_int*, casadi_int*, casadi_int*, casadi_int*);
};

#define GENERATED_API(f) GeneratedApi{#f, f, f##_checkout, f##_release, f##_init_mem, f##_sparsity_in, f##_sparsity_out, f##_work}

namespace {

// Row index: GeneratedFunction::Id. Column: numUavs - 1.
constexpr GeneratedApi kApis[][2] = {
    {GENERATED_API(solver_oneGround_nmpc), GENERATED_API(solver_twoUavPayload_nmpc)},
    {GENERATED_API(solver_oneGround_lmpc), GENERATED_API(solver_twoUavPayload_lmpc)},
    {GENERATED_API(linearize_oneGround_lmpc), GENERATED_API(linearize_twoUavPayload_lmpc)},
    {GENERATED_API(nmhe_oneGround), GENERATED_API(nmhe_twoUavPayload)},
};

const GeneratedApi& findApi(const GeneratedFunction::Id id, const int numUavs) {
    if (numUavs < 1 || numUavs > 2) {
        throw std::runtime_error("GeneratedFunction: nothing generated for numUavs=" + std::to_string(numUavs));
    }
    return kApis[static_cast<int>(id)][numUavs - 1];
}

// Compressed CasADi sparsity: {nrow, ncol, 1} when dense.
size_t denseSize(const casadi_int* sparsity, const char* name) {
    if (sparsity[2] != 1) {
        throw std::runtime_error(std::string("GeneratedFunction: ") + name + " has a sparse input or output");
    }
    return static_cast<size_t>(sparsity[0] * sparsity[1]);
}

} // namespace

GeneratedFunction::GeneratedFunction(const Id id, const int numUavs)
    : m_api(findApi(id, numUavs))
    , m_mem(m_api.checkout())
{
    m_api.initMem(m_mem);

    // arg/res are also scratch for the generated code: sized by work(), not
    // by the number of inputs/outputs.
    casadi_int szArg, szRes, szIw, szW;
    m_api.work(&szArg, &szRes, &szIw, &szW);
    m_arg.resize(szArg);
    m_res.resize(szRes);
    m_iw.resize(szIw);
    m_w.resize(szW);
}

GeneratedFunction::~GeneratedFunction() {
    m_api.release(m_mem);
}

size_t GeneratedFunction::inputSize(const int i) const {
    return denseSize(m_api.sparsityIn(i), m_api.name);
}

size_t GeneratedFunction::outputSize(const int i) const {
    return denseSize(m_api.sparsityOut(i), m_api.name);
}

int GeneratedFunction::eval(const std::initializer_list<const double*> arg, const std::initializer_list<double*> res) {
    std::ranges::copy(arg, m_arg.begin());
    std::ranges::copy(res, m_res.begin());
    return m_api.eval(m_arg.data(), m_res.data(), m_iw.data(), m_w.data(), m_mem);
}

const char* GeneratedFunction::name() const {
    return m_api.name;
}
