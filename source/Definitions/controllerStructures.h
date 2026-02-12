/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLLERSTRUCTURES_H
#define CONTROLLERSTRUCTURES_H

#pragma once

#include <vector>

struct solverConfig {
    int nx;         // state dimension per UAV
    int nu;         // control dimension per UAV
    int np;         // parameter vector length used by the solver
    int N;          // prediction horizon
    int numUavs;    // number of UAVs in the solver dynamic model
    std::vector<double> weight;
    std::vector<double> lbxStates;
    std::vector<double> ubxStates;
    std::vector<double> lbxControls;
    std::vector<double> ubxControls;
};

#endif //CONTROLLERSTRUCTURES_H
