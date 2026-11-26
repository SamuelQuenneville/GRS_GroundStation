/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef YAMLCUSTOMSTRUCTURE_H
#define YAMLCUSTOMSTRUCTURE_H

#pragma once

#include <algorithm>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

#include "yaml-cpp/node/node.h"
#include "controllerStructures.h"

/*
 *   YAML template for custom struct
 */

namespace YAML {

    template<>
        struct convert<catapultEndpointConfig> {
            static bool decode(const Node& node, catapultEndpointConfig& rhs) {
                if (!node.IsMap()) return false;
                rhs.id         = node["id"].as<uint8_t>();
                rhs.expectedIp = node["ip"].as<std::string>();
                rhs.port       = node["port"] ? node["port"].as<uint16_t>() : CATAPULT_PORT;
                return true;
            }
        };

    template<>
        struct convert<pixhawkEndpointConfig> {
            static bool decode(const Node& node, pixhawkEndpointConfig& rhs) {
                if (!node.IsMap()) return false;
                rhs.id   = node["id"].as<uint8_t>();
                if (node["uri"]) {
                    rhs.uri = node["uri"].as<std::string>();
                    return true;
                }
                rhs.ip   = node["ip"].as<std::string>();
                rhs.port = node["port"].as<uint16_t>();
                return true;
            }
        };

    template<>
        struct convert<pixhawkConfig> {
            static bool decode(const Node& node, pixhawkConfig& rhs) {
                if (!node.IsMap()) return false;
                if (node["sitl"])             rhs.sitl             = node["sitl"].as<bool>();
                if (node["remoteIP"])         rhs.remoteIP         = node["remoteIP"].as<std::string>();
                if (node["tcpPort"])          rhs.tcpPort          = node["tcpPort"].as<int>();
                if (node["tcpPortIncrement"]) rhs.tcpPortIncrement = node["tcpPortIncrement"].as<int>();
                if (node["paramCheck"])       rhs.paramCheck       = node["paramCheck"].as<bool>();
                if (node["paramCheckPrefixes"]) rhs.paramCheckPrefixes = node["paramCheckPrefixes"].as<std::vector<std::string>>();
                return true;
            }
        };

    template<>
        struct convert<linkDelayConfig> {
            static bool decode(const Node& node, linkDelayConfig& rhs) {
                if (!node.IsMap()) return false;
                const auto mode = node["mode"] ? node["mode"].as<std::string>() : "none";
                if (mode == "none")            rhs.mode = linkDelayConfig::Mode::NONE;
                else if (mode == "constant")   rhs.mode = linkDelayConfig::Mode::CONSTANT;
                else if (mode == "parametric") rhs.mode = linkDelayConfig::Mode::PARAMETRIC;
                else if (mode == "trace")      rhs.mode = linkDelayConfig::Mode::TRACE;
                else throw std::runtime_error("LinkEmulation mode must be none, constant, parametric or trace, got '" + mode + "'");

                if (node["delayMs"])       rhs.delayMs       = node["delayMs"].as<double>();
                if (node["jitterMs"])      rhs.jitterMs      = node["jitterMs"].as<double>();
                if (node["episodeRateHz"]) rhs.episodeRateHz = node["episodeRateHz"].as<double>();
                if (node["peakMinMs"])     rhs.peakMinMs     = node["peakMinMs"].as<double>();
                if (node["peakMedianMs"])  rhs.peakMedianMs  = node["peakMedianMs"].as<double>();
                if (node["peakSigma"])     rhs.peakSigma     = node["peakSigma"].as<double>();
                if (node["peakMaxMs"])     rhs.peakMaxMs     = node["peakMaxMs"].as<double>();
                if (node["growth"])        rhs.growth        = node["growth"].as<double>();
                if (node["file"])          rhs.traceFile     = node["file"].as<std::string>();

                if (rhs.delayMs < 0.0 || rhs.jitterMs < 0.0 || rhs.episodeRateHz < 0.0 || rhs.peakSigma < 0.0 || !(rhs.growth > 0.0))
                    throw std::runtime_error("LinkEmulation: delayMs, jitterMs, episodeRateHz, peakSigma must be >= 0 and growth > 0");

                if (rhs.mode == linkDelayConfig::Mode::TRACE) {
                    std::ifstream file(rhs.traceFile);
                    if (!file) throw std::runtime_error("LinkEmulation: cannot read trace file '" + rhs.traceFile + "'");
                    std::string line;
                    std::getline(file, line);
                    while (std::getline(file, line)) {
                        double t, delayMs;
                        char comma;
                        std::istringstream fields(line);
                        if (fields >> t >> comma >> delayMs) rhs.trace.emplace_back(t, delayMs);
                    }
                    if (rhs.trace.empty()) throw std::runtime_error("LinkEmulation: trace file '" + rhs.traceFile + "' has no samples");
                }
                return true;
            }
        };

    template<>
        struct convert<linkEmulationConfig> {
            static bool decode(const Node& node, linkEmulationConfig& rhs) {
                if (!node.IsMap()) return false;
                if (node["enabled"]) rhs.enabled = node["enabled"].as<bool>();
                if (node["seed"]) rhs.seed = node["seed"].as<uint32_t>();
                if (const auto vehicles = node["vehicles"]) {
                    for (const auto& vehicle : vehicles) {
                        const auto sysId = static_cast<uint8_t>(vehicle.first.as<int>());
                        if (vehicle.second["downlink"]) rhs.downlink[sysId] = vehicle.second["downlink"].as<linkDelayConfig>();
                        if (vehicle.second["uplink"])   rhs.uplink[sysId]   = vehicle.second["uplink"].as<linkDelayConfig>();
                    }
                }
                return true;
            }
        };

    template<>
        struct convert<solverConfig> {
            static bool decode(const Node& node, solverConfig& rhs) {
                if(!node.IsMap()) {
                    return false;
                }

                rhs.nx             = node["NX"].as<int>();
                rhs.nu             = node["NU"].as<int>();
                rhs.np             = node["NP"].as<int>();
                rhs.nd             = node["ND"].as<int>();
                rhs.nL0            = node["NL0"].as<int>();
                if (rhs.nL0 != 1 && rhs.nL0 != 2) throw std::runtime_error("SolverConfiguration.NL0 must be 1 (L0) or 2 (L0, payload mass)");
                rhs.N              = node["N"].as<int>();
                rhs.numUavs        = node["NUM_UAVS"].as<int>();
                rhs.dt             = node["DT"].as<double>();
                rhs.tetherL0       = node["L0"].as<double>();
                rhs.alphaMax       = node["ALPHA_MAX"].as<double>();
                rhs.weight         = node["WEIGHT"].as<std::vector<double>>();
                rhs.lbxStates      = node["LBX_STATES"].as<std::vector<double>>();
                rhs.ubxStates      = node["UBX_STATES"].as<std::vector<double>>();
                rhs.lbxControls    = node["LBX_CONTROLS"].as<std::vector<double>>();
                rhs.ubxControls    = node["UBX_CONTROLS"].as<std::vector<double>>();
                rhs.scalesStates   = node["SCALES_STATES"].as<std::vector<double>>();
                rhs.scalesControls = node["SCALES_CONTROLS"].as<std::vector<double>>();

                if (node["AOA_FF_STAGE"]) rhs.aoaFeedforwardStage = node["AOA_FF_STAGE"].as<int>();
                if (rhs.aoaFeedforwardStage < 0 || rhs.aoaFeedforwardStage >= rhs.N) {
                    throw std::runtime_error("SolverConfiguration.AOA_FF_STAGE must be in [0, N-1]");
                }

                if (node["DELAY_STAGES"])    rhs.delayStages = node["DELAY_STAGES"].as<int>();
                if (node["DELAY_TAU_ROLL"])  rhs.delayTauRoll = node["DELAY_TAU_ROLL"].as<double>();
                if (node["DELAY_TAU_PITCH"]) rhs.delayTauPitch = node["DELAY_TAU_PITCH"].as<double>();
                if (node["PAYLOAD_MASS"])    rhs.payloadMass = node["PAYLOAD_MASS"].as<double>();
                if (node["MAX_CONTROL_STEP"]) {
                    // [thrust N, roll deg, pitch deg] per tick, stored in rad
                    const auto s = node["MAX_CONTROL_STEP"].as<std::vector<double>>();
                    if (s.size() != 3 || !(s[0] > 0.0 && s[1] > 0.0 && s[2] > 0.0)) {
                        throw std::runtime_error("SolverConfiguration.MAX_CONTROL_STEP must be [thrust N, roll deg, pitch deg], all > 0");
                    }
                    constexpr double rad = std::numbers::pi / 180.0;
                    rhs.maxControlStep = {s[0], s[1] * rad, s[2] * rad};
                }
                if (rhs.delayStages < 0 || rhs.delayStages >= rhs.N) {
                    throw std::runtime_error("SolverConfiguration.DELAY_STAGES must be in [0, N-1]");
                }
                if (!(rhs.delayTauRoll > 0.0) || !(rhs.delayTauPitch > 0.0) || !(rhs.payloadMass > 0.0)) {
                    throw std::runtime_error("SolverConfiguration.DELAY_TAU_ROLL, DELAY_TAU_PITCH and PAYLOAD_MASS must be > 0");
                }

                if (node["LAUNCH_POS_TOL"])   rhs.launchPositionTolerance = node["LAUNCH_POS_TOL"].as<double>();
                if (node["LAUNCH_YAW_TOL"])   rhs.launchYawTolerance = node["LAUNCH_YAW_TOL"].as<double>();
                if (node["IN_FLIGHT_SPEED"])  rhs.inFlightSpeed = node["IN_FLIGHT_SPEED"].as<double>();
                if (node["LAUNCH_TIMEOUT"])   rhs.launchTimeout = node["LAUNCH_TIMEOUT"].as<double>();

                // Optional: "nmpc" (default) or "lmpc".
                if (const auto controller = node["CONTROLLER"]) {
                    const auto name = controller.as<std::string>();
                    if (name == "lmpc") {
                        rhs.controller = solverConfig::Controller::Lmpc;
                    } else if (name != "nmpc") {
                        throw std::runtime_error("SolverConfiguration.CONTROLLER must be 'nmpc' or 'lmpc', got '" + name + "'");
                    }
                }

                const auto checkSize = [](const std::vector<double>& v, const int n, const char* key) {
                    if (v.size() != static_cast<size_t>(n)) {
                        throw std::runtime_error(std::string("SolverConfiguration.") + key + " needs " + std::to_string(n) + " values, got " + std::to_string(v.size()));
                    }
                };
                checkSize(rhs.lbxStates, rhs.nx, "LBX_STATES");
                checkSize(rhs.ubxStates, rhs.nx, "UBX_STATES");
                checkSize(rhs.scalesStates, rhs.nx, "SCALES_STATES");
                checkSize(rhs.lbxControls, rhs.nu, "LBX_CONTROLS");
                checkSize(rhs.ubxControls, rhs.nu, "UBX_CONTROLS");
                checkSize(rhs.scalesControls, rhs.nu, "SCALES_CONTROLS");
                for (const auto& scales : {rhs.scalesStates, rhs.scalesControls}) {
                    if (std::ranges::any_of(scales, [](const double s) { return !(s > 0.0); })) {
                        throw std::runtime_error("SolverConfiguration scales must be > 0");
                    }
                }

                for (const auto scale: rhs.scalesStates) {
                    rhs.invScalesStates.push_back(1.0 / scale);
                }

                for (const auto scale: rhs.scalesControls) {
                    rhs.invScalesControls.push_back(1.0 / scale);
                }

                return true;
            }
        };

    template<>
        struct convert<estimatorConfig> {
            static bool decode(const Node& node, estimatorConfig& rhs) {
                if(!node.IsMap()) {
                    return false;
                }

                rhs.nx      = node["NX"].as<int>();
                rhs.nu      = node["NU"].as<int>();
                rhs.np      = node["NP"].as<int>();
                rhs.nd      = node["ND"].as<int>();
                rhs.nL0     = node["NL0"].as<int>();
                if (rhs.nL0 != 1 && rhs.nL0 != 2) throw std::runtime_error("EstimatorConfiguration.NL0 must be 1 (L0) or 2 (L0, payload mass)");
                rhs.nxi     = rhs.nx + rhs.np + rhs.nd;
                rhs.M       = node["M"].as<int>();
                rhs.numUavs = node["NUM_UAVS"].as<int>();
                rhs.dt      = node["DT"].as<double>();
                rhs.tetherL0 = node["L0"].as<double>();

                rhs.wMeas      = node["W_MEAS"].as<std::vector<double>>();
                rhs.wWindPrior = node["W_WINDP"].as<std::vector<double>>();
                rhs.wDPrior    = node["W_DP"].as<std::vector<double>>();

                rhs.windBound = node["WIND_BOUND"].as<std::vector<double>>();
                rhs.dBound    = node["D_BOUND"].as<std::vector<double>>();
                if (rhs.windBound.size() != static_cast<size_t>(rhs.np) || rhs.dBound.size() != static_cast<size_t>(rhs.nd)) {
                    throw std::runtime_error("EstimatorConfiguration: WIND_BOUND needs NP entries and D_BOUND ND entries");
                }

                rhs.xScale    = node["X_SCALE"].as<std::vector<double>>();
                rhs.windScale = node["WIND_SCALE"].as<std::vector<double>>();
                rhs.dScale    = node["D_SCALE"].as<std::vector<double>>();

                for (const auto scale: rhs.xScale) {
                    rhs.invXScale.push_back(1.0 / scale);
                }

                return true;
            }
        };
}

#endif //YAMLCUSTOMSTRUCTURE_H
