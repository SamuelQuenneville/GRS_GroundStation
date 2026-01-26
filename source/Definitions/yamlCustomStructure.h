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

#include "yaml-cpp/node/node.h"

#include "controllerStructures.h"

/*
 *   YAML template for custom struct
 */

namespace YAML {

    template<>
        struct convert<solverConfig> {
            static bool decode(const Node& node, solverConfig& rhs) {
                if(!node.IsMap()) {
                    return false;
                }

                rhs.nx          = node["NX"].as<int>();
                rhs.nu          = node["NU"].as<int>();
                rhs.np          = node["NP"].as<int>();
                rhs.N           = node["N"].as<int>();
                rhs.numUavs     = node["NUM_UAVS"].as<int>();
                rhs.lbxStates   = node["LBX_STATES"].as<std::vector<double>>();
                rhs.ubxStates   = node["UBX_STATES"].as<std::vector<double>>();
                rhs.lbxControls = node["LBX_CONTROLS"].as<std::vector<double>>();
                rhs.ubxControls = node["UBX_CONTROLS"].as<std::vector<double>>();
                return true;
            }
        };
}

#endif //YAMLCUSTOMSTRUCTURE_H
