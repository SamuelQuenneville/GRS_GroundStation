/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NLPSOL_H
#define NLPSOL_H

#pragma once

#include <vector>

struct NlpsolApi;

// One CasADi-generated nlpsol from CasadiSolver/, with its buffers: dense
// inputs and outputs in nlpsol order, and workspaces sized by the solver.
class Nlpsol {
public:
    enum class Problem { Nmpc, Nmhe };

    // Throws if no solver was generated for this problem and numUavs.
    Nlpsol(Problem problem, int numUavs);
    ~Nlpsol();

    Nlpsol(const Nlpsol&) = delete;
    Nlpsol& operator=(const Nlpsol&) = delete;

    // Inputs, zero-filled at construction
    std::vector<double> x0, p, lbx, ubx, lbg, ubg, lamX0, lamG0;
    // Outputs
    std::vector<double> x, f, g, lamX, lamG, lamP;

    // Returns the generated function's flag: non-zero on an evaluation
    // error only, not when Fatrop stops without converging.
    int solve();

    struct Check {
        bool valid = false;
        // Worst violation of lbg <= g <= ubg; -1 when not evaluated (flag != 0).
        double maxConstraintViolation = -1.0;
    };

    // Usable if flag is 0, every constraint holds within feasTol, and g, x and f are finite.
    [[nodiscard]] Check check(int flag, double feasTol = kFeasibilityTolerance) const;

    static constexpr double kFeasibilityTolerance = 5e-4;

    [[nodiscard]] const char* name() const;

private:
    const NlpsolApi& m_api;
    int m_mem;
    std::vector<const double*> m_arg;
    std::vector<double*> m_res;
    std::vector<long long> m_iw;
    std::vector<double> m_w;
};

#endif //NLPSOL_H
