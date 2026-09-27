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

Nlpsol::Nlpsol(const GeneratedFunction::Id id, const int numUavs)
    : m_function(id, numUavs)
{
    std::vector<double>* in[] = {&x0, &p, &lbx, &ubx, &lbg, &ubg, &lamX0, &lamG0};
    for (int i = 0; i < 8; ++i) {
        in[i]->assign(m_function.inputSize(i), 0.0);
    }

    std::vector<double>* out[] = {&x, &f, &g, &lamX, &lamG, &lamP};
    for (int i = 0; i < 6; ++i) {
        out[i]->assign(m_function.outputSize(i), 0.0);
    }
}

int Nlpsol::solve() {
    return m_function.eval({x0.data(), p.data(), lbx.data(), ubx.data(), lbg.data(), ubg.data(), lamX0.data(), lamG0.data()},
                           {x.data(), f.data(), g.data(), lamX.data(), lamG.data(), lamP.data()});
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
    return m_function.name();
}
