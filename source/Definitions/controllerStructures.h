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
    int np;         // wind parameter count (3, shared -- not per UAV)
    int nd;         // disturbance-bias parameter count (from NMHE/EKF; zero
                     // vector until an estimator is wired in -- see the
                     // wind/disturbance gap in gcs-sitl-integration-plan.md)
    int nL0;         // tether rest-length parameter count (1 today)
    int N;          // prediction horizon
    int numUavs;    // number of UAVs in the solver dynamic model
    double dt;      // shooting interval [s] the solver was built with --
                     // bookkeeping only (baked into the codegen'd dynamics
                     // already), but tracked so a config/solver mismatch is
                     // at least visible instead of silently wrong (ADR-001
                     // action item #1).
    double tetherL0; // tether rest length [m], packed into the solver's L0
                     // parameter every solve. Fixed from config for now --
                     // see tether-length-parameterization.md for the
                     // GCS-settable version this will become.
    std::vector<double> weight;
    std::vector<double> lbxStates;
    std::vector<double> ubxStates;
    std::vector<double> lbxControls;
    std::vector<double> ubxControls;
    std::vector<double> scalesStates;
    std::vector<double> scalesControls;
    std::vector<double> invScalesStates;
    std::vector<double> invScalesControls;
};

#endif //CONTROLLERSTRUCTURES_H
