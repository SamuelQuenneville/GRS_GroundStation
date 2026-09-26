/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NLPSOLIO_H
#define NLPSOLIO_H

#pragma once

#include <array>
#include <vector>

// Buffers for one CasADi-generated nlpsol (8 dense inputs, 6 dense outputs,
// integer and real workspaces), shared by MpcController and NmheEstimator.
// Works with any backend exposing inputSize()/outputSize()/workIntSize()/
// workRealSize()/solve() (SolverBackend and EstimatorBackend both do).
struct NlpsolIo {
    // Inputs, in nlpsol order
    std::vector<double> x0;     // 0: initial guess
    std::vector<double> p;      // 1: parameters
    std::vector<double> lbx;    // 2: decision-variable bounds
    std::vector<double> ubx;    // 3
    std::vector<double> lbg;    // 4: constraint bounds
    std::vector<double> ubg;    // 5
    std::vector<double> lamX0;  // 6: dual warm start
    std::vector<double> lamG0;  // 7

    // Outputs, in nlpsol order
    std::vector<double> x;      // 0: solution
    std::vector<double> f;      // 1: objective
    std::vector<double> g;      // 2: constraint values
    std::vector<double> lamX;   // 3
    std::vector<double> lamG;   // 4
    std::vector<double> lamP;   // 5

    // Everything sized from the backend, zero-filled.
    template <class Backend>
    explicit NlpsolIo(const Backend& backend) {
        std::vector<double>* in[8] = {&x0, &p, &lbx, &ubx, &lbg, &ubg, &lamX0, &lamG0};
        for (int i = 0; i < 8; ++i) in[i]->assign(static_cast<size_t>(backend.inputSize(i)), 0.0);
        std::vector<double>* out[6] = {&x, &f, &g, &lamX, &lamG, &lamP};
        for (int i = 0; i < 6; ++i) out[i]->assign(static_cast<size_t>(backend.outputSize(i)), 0.0);
        m_iw.assign(backend.workIntSize(), 0);
        m_w.assign(backend.workRealSize(), 0.0);
    }

    // Runs one solve on the current inputs; returns the backend's flag.
    template <class Backend>
    int solve(Backend& backend) {
        m_bind();
        return backend.solve(m_arg.data(), m_res.data(), m_iw.data(), m_w.data());
    }

    struct Check {
        bool valid = false;
        // Worst violation of lbg <= g <= ubg; -1 when not evaluated (flag != 0).
        double maxConstraintViolation = -1.0;
    };

    // A solution is usable if the backend flag is 0, every constraint holds
    // within feasTol, and the constraints, solution and objective are finite.
    [[nodiscard]] Check check(int flag, double feasTol = kFeasibilityTolerance) const;

    static constexpr double kFeasibilityTolerance = 5e-4;

private:
    std::vector<long long> m_iw;
    std::vector<double> m_w;
    std::array<const double*, 8> m_arg{};
    std::array<double*, 6> m_res{};

    void m_bind();
};

#endif //NLPSOLIO_H
