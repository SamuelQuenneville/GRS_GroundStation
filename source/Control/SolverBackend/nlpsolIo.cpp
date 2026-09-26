/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "nlpsolIo.h"

#include <algorithm>
#include <cmath>

void NlpsolIo::m_bind() {
    m_arg = {x0.data(), p.data(), lbx.data(), ubx.data(), lbg.data(), ubg.data(), lamX0.data(), lamG0.data()};
    m_res = {x.data(), f.data(), g.data(), lamX.data(), lamG.data(), lamP.data()};
}

NlpsolIo::Check NlpsolIo::check(const int flag, const double feasTol) const {
    Check c;
    if (flag != 0) {
        return c; // solver call itself failed: nothing to evaluate
    }

    double worst = 0.0;
    for (size_t i = 0; i < g.size(); ++i) {
        worst = std::max({worst, lbg[i] - g[i], g[i] - ubg[i]});
    }
    c.maxConstraintViolation = worst;

    // std::max() ignores NaN, so non-finite values are tested separately.
    const auto finite = [](const std::vector<double>& v) {
        return std::all_of(v.begin(), v.end(), [](const double e) { return std::isfinite(e); });
    };
    c.valid = worst <= feasTol && finite(g) && finite(x) && !f.empty() && std::isfinite(f[0]);
    return c;
}
