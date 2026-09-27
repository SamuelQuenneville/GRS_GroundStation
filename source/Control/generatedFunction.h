/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GENERATEDFUNCTION_H
#define GENERATEDFUNCTION_H

#pragma once

#include <cstddef>
#include <initializer_list>
#include <vector>

struct GeneratedApi;

// One CasADi-generated function from CasadiSolver/, with its memory slot and
// workspaces. Inputs and outputs must be dense.
class GeneratedFunction {
public:
    enum class Id { Nmpc, Lmpc, LmpcLinearization, Nmhe };

    // Throws if nothing was generated for this id and numUavs.
    GeneratedFunction(Id id, int numUavs);
    ~GeneratedFunction();

    GeneratedFunction(const GeneratedFunction&) = delete;
    GeneratedFunction& operator=(const GeneratedFunction&) = delete;

    // Number of values of input or output i.
    [[nodiscard]] size_t inputSize(int i) const;
    [[nodiscard]] size_t outputSize(int i) const;

    // One pointer per input and per output, in order. Returns the generated
    // function's flag (non-zero on an evaluation error).
    int eval(std::initializer_list<const double*> arg, std::initializer_list<double*> res);

    [[nodiscard]] const char* name() const;

private:
    const GeneratedApi& m_api;
    int m_mem;
    std::vector<const double*> m_arg;
    std::vector<double*> m_res;
    std::vector<long long> m_iw;
    std::vector<double> m_w;
};

#endif //GENERATEDFUNCTION_H
