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

                if (node["LAUNCH_POS_TOL"])   rhs.launchPositionTolerance = node["LAUNCH_POS_TOL"].as<double>();
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
