/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "truthSample.h"

#include <set>
#include <sstream>
#include <stdexcept>
#include <cmath>

namespace grs::sim {

namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\"");
    const auto e = s.find_last_not_of(" \t\r\"");
    return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(trim(cell));
    return out;
}

// Multiplicative ('rel') parameter names for the given vehicle count.
std::set<std::string> relNames(const int numUavs) {
    std::set<std::string> names{"pay_mass", "tether_k", "tether_b"};
    for (int i = 1; i <= numUavs; ++i) {
        const std::string a = "a" + std::to_string(i) + "_";
        for (const char* c : {"mass", "CL0", "CL_alpha", "CD0", "e", "T_roll", "T_pitch"}) names.insert(a + c);
    }
    if (numUavs == 1) names.erase("pay_mass");
    return names;
}

std::set<std::string> absNames(const int numUavs) {
    std::set<std::string> names{"wind_speed", "wind_heading", "wind_down", "tether_L0_err"};
    for (int i = 1; i <= numUavs; ++i) {
        const std::string d = "d" + std::to_string(i) + "_";
        for (const char* c : {"broll", "bpitch"}) names.insert(d + c);
        if (numUavs == 1) {
            for (const char* c : {"Fx", "Fy", "Fz"}) names.insert(d + c);
        } else {
            for (const char* c : {"Fa", "CL"}) names.insert(d + c);
        }
    }
    if (numUavs > 1) names.insert("pay_Fz");
    return names;
}

} // namespace

std::vector<Sample> readSamples(const std::string& path) {
    if (path == "nominal") return {Sample{}};

    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open samples file " + path);

    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("empty samples file " + path);
    const auto header = splitCsv(line);
    if (header.empty() || header[0] != "sample_id") {
        throw std::runtime_error(path + ": first column must be sample_id");
    }

    std::vector<Sample> samples;
    while (std::getline(f, line)) {
        if (trim(line).empty()) continue;
        const auto cells = splitCsv(line);
        if (cells.size() != header.size()) {
            throw std::runtime_error(path + ": row has " + std::to_string(cells.size()) + " cells, header has " +
                                     std::to_string(header.size()));
        }
        Sample s;
        s.id = static_cast<int>(std::lround(std::stod(cells[0])));
        for (size_t i = 1; i < header.size(); ++i) s.values[header[i]] = std::stod(cells[i]);
        samples.push_back(std::move(s));
    }
    return samples;
}

TruthSpec applySample(const Sample& sample, const int numUavs, const double L0Nominal) {
    const auto rel = relNames(numUavs);
    const auto abs = absNames(numUavs);
    for (const auto& [name, value] : sample.values) {
        if (!rel.contains(name) && !abs.contains(name)) {
            throw std::runtime_error("sample " + std::to_string(sample.id) + ": unknown parameter '" + name +
                                     "' for a " + std::to_string(numUavs) + "-UAV plant (names follow mc_param_space_twoUav.m)");
        }
        if (rel.contains(name) && !(std::isfinite(value) && value > 0.0)) {
            throw std::runtime_error("sample " + std::to_string(sample.id) + ": " + name + " = " + std::to_string(value) +
                                     ", but it is a multiplicative factor on nominal (1 = nominal) and must be > 0");
        }
    }
    auto v = [&](const std::string& name) {
        const auto it = sample.values.find(name);
        if (it != sample.values.end()) return it->second;
        return rel.contains(name) ? 1.0 : 0.0; // every 'abs' nominal is 0 in mc_param_space_twoUav.m
    };

    TruthSpec t;
    for (int i = 0; i < numUavs; ++i) {
        const std::string a = "a" + std::to_string(i + 1) + "_";
        AirframeParams& p = t.uav[i];
        p.mass *= v(a + "mass");
        p.CL0 *= v(a + "CL0");
        p.CL_alpha *= v(a + "CL_alpha");
        p.CD0 *= v(a + "CD0");
        p.e *= v(a + "e");
        p.T_roll *= v(a + "T_roll");
        p.T_pitch *= v(a + "T_pitch");
    }

    if (numUavs == 2) t.rig.m_pay *= v("pay_mass");
    t.rig.k_t *= v("tether_k");
    t.rig.b_t *= v("tether_b");

    t.L0Plant = L0Nominal + v("tether_L0_err");

    const double spd = v("wind_speed");
    const double hdg = v("wind_heading") * M_PI / 180.0;
    t.windTrue = {spd * std::cos(hdg), spd * std::sin(hdg), v("wind_down")};

    // Same layout as the model's d (plantModel.cpp).
    constexpr double deg = M_PI / 180.0;
    if (numUavs == 1) {
        t.dTrue = {v("d1_Fx"), v("d1_Fy"), v("d1_Fz"), v("d1_broll") * deg, v("d1_bpitch") * deg};
    } else {
        t.dTrue.clear();
        for (int i = 0; i < numUavs; ++i) {
            const std::string n = "d" + std::to_string(i + 1) + "_";
            for (const double value : {v(n + "Fa"), v(n + "CL"), v(n + "broll") * deg, v(n + "bpitch") * deg}) {
                t.dTrue.push_back(value);
            }
        }
        t.dTrue.push_back(v("pay_Fz"));
    }
    return t;
}

std::unique_ptr<PlantModel> makePlant(const TruthSpec& spec, const int numUavs) {
    if (numUavs == 2) return std::make_unique<TwoUavPayloadPlant>(spec.uav[0], spec.uav[1], spec.rig);
    if (numUavs == 1) return std::make_unique<OneGroundPlant>(spec.uav[0], spec.rig);
    throw std::runtime_error("no truth plant for NUM_UAVS=" + std::to_string(numUavs));
}

} // namespace grs::sim
