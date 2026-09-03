/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

// Standalone regression harness for the Phase 0 trajectory-generator port
// (ADR-001). Compares TrajectoryGenerator::generate() against Octave-run
// output of the original MATLAB trajectory_generation project, saved as CSVs
// under tests/golden/ (see trajectory_generation/run_gen.m for how those
// were produced). Not wired into the main GCS CMake build -- this exercises
// only source/Trajectory + source/Mathematics, no MAVSDK/CasADi required.
//
// Usage: validate_trajectory <path-to-golden-dir>

#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "../trajectoryGenerator.h"

namespace {

std::vector<std::vector<double>> readCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<double> row;
        std::stringstream ss(line);
        std::string field;
        while (std::getline(ss, field, ',')) row.push_back(std::stod(field));
        rows.push_back(row);
    }
    return rows;
}

struct Stats { double maxAbs = 0.0; size_t worstRow = 0; size_t worstCol = 0; };

Stats compare(const std::string& name, const std::vector<std::vector<double>>& golden,
              const std::function<double(size_t, size_t)>& actual, size_t nCols) {
    Stats stats;
    bool sizeMismatch = false;
    if (golden.empty()) { std::printf("[%s] golden file empty!\n", name.c_str()); return stats; }

    for (size_t r = 0; r < golden.size(); ++r) {
        for (size_t c = 0; c < nCols; ++c) {
            const double g = golden[r][c];
            const double a = actual(r, c);
            const double diff = std::abs(g - a);
            if (diff > stats.maxAbs) { stats.maxAbs = diff; stats.worstRow = r; stats.worstCol = c; }
        }
    }
    std::printf("[%-28s] rows=%-5zu maxAbsErr=%.6e (row %zu, col %zu)%s\n",
                name.c_str(), golden.size(), stats.maxAbs, stats.worstRow, stats.worstCol,
                sizeMismatch ? "  ** SIZE MISMATCH **" : "");
    return stats;
}

}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <golden-dir>\n", argv[0]);
        return 2;
    }
    const std::string dir = argv[1];

    grs::trajgen::TrajectoryConfig config; // defaults mirror trajectory_generation/config/config.m
    grs::trajgen::TrajectoryGenerator gen(config);
    const auto mission = gen.generate();

    const size_t nTakeoff = static_cast<size_t>(std::floor(config.aircraftPath.takeoffTime / config.simDt + 1e-9)) + 1;

    double worst = 0.0;

    for (int k = 1; k <= 2; ++k) {
        const size_t idx = static_cast<size_t>(k - 1);

        // Takeoff phase pos/vel/acc
        {
            const auto pos = readCsv(dir + "/oct_takeoff" + std::to_string(k) + "_pos.csv");
            auto s = compare("takeoff" + std::to_string(k) + "_pos", pos,
                [&](size_t r, size_t c){ return mission.aircraft[idx].inertial[r].pos[c]; }, 3);
            worst = std::max(worst, s.maxAbs);
        }
        {
            const auto vel = readCsv(dir + "/oct_takeoff" + std::to_string(k) + "_vel.csv");
            auto s = compare("takeoff" + std::to_string(k) + "_vel", vel,
                [&](size_t r, size_t c){ return mission.aircraft[idx].inertial[r].vel[c]; }, 3);
            worst = std::max(worst, s.maxAbs);
        }

        // Mission phase pos/vel (offset by nTakeoff into the appended mission array)
        {
            const auto pos = readCsv(dir + "/oct_ac" + std::to_string(k) + "_pos.csv");
            auto s = compare("ac" + std::to_string(k) + "_pos(mission)", pos,
                [&](size_t r, size_t c){ return mission.aircraft[idx].inertial[nTakeoff + r].pos[c]; }, 3);
            worst = std::max(worst, s.maxAbs);
        }
        {
            const auto vel = readCsv(dir + "/oct_ac" + std::to_string(k) + "_vel.csv");
            auto s = compare("ac" + std::to_string(k) + "_vel(mission)", vel,
                [&](size_t r, size_t c){ return mission.aircraft[idx].inertial[nTakeoff + r].vel[c]; }, 3);
            worst = std::max(worst, s.maxAbs);
        }

        // Controls: MATLAB columns = [roll, pitch, yaw, thrust, aoa, liftX, liftY, liftZ]
        {
            const auto ctrl = readCsv(dir + "/oct_controls" + std::to_string(k) + ".csv");
            auto s = compare("controls" + std::to_string(k), ctrl,
                [&](size_t r, size_t c) -> double {
                    const auto& cs = mission.controls[idx][r];
                    switch (c) {
                        case 0: return cs.rollRad;
                        case 1: return cs.pitchRad;
                        case 2: return cs.yawRad;
                        case 3: return cs.thrustNewton;
                        case 4: return cs.angleOfAttackRad;
                        case 5: return cs.liftDirection[0];
                        case 6: return cs.liftDirection[1];
                        case 7: return cs.liftDirection[2];
                        default: return 0.0;
                    }
                }, 8);
            worst = std::max(worst, s.maxAbs);
        }
    }

    // Payload: mission-only golden (2797 rows) vs the appended array offset by nTakeoff.
    {
        const auto pos = readCsv(dir + "/oct_payload_pos.csv");
        auto s = compare("payload_pos(mission)", pos,
            [&](size_t r, size_t c){ return mission.payload[nTakeoff + r].pos[c]; }, 3);
        worst = std::max(worst, s.maxAbs);
    }
    {
        const auto vel = readCsv(dir + "/oct_payload_vel.csv");
        auto s = compare("payload_vel(mission)", vel,
            [&](size_t r, size_t c){ return mission.payload[nTakeoff + r].vel[c]; }, 3);
        worst = std::max(worst, s.maxAbs);
    }

    std::printf("\nWorst absolute error across all comparisons: %.6e\n", worst);

    constexpr double kTolerance = 1e-6;
    if (worst > kTolerance) {
        std::printf("FAIL: exceeds tolerance %.1e\n", kTolerance);
        return 1;
    }
    std::printf("PASS: within tolerance %.1e\n", kTolerance);
    return 0;
}
