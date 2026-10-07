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

// NMPC or LMPC configuration. Must match the solver as it was generated
// (build_nlp_*_nmpc.m, build_nlp_*_lmpc.m): the solver bakes in N, dt and
// the constraint structure.
struct solverConfig {
    // Lmpc: the NMPC problem with the dynamics and angle-of-attack
    // constraints linearized about the reference window at every solve.
    enum class Controller { Nmpc, Lmpc };
    Controller controller = Controller::Nmpc;

    int nx;          // joint state size, all vehicles (stateVector.h)
    int nu;          // joint control size, all UAVs
    int np;          // wind parameters (3, shared)
    int nd;          // disturbance parameters
    int nL0;         // tether rest-length parameters
    int N;           // prediction horizon
    int numUavs;
    double dt;       // shooting interval [s]
    double tetherL0; // tether rest length [m]
    double alphaMax; // angle-of-attack bound [rad], symmetric
    // Plan stage, counted from the applied control, whose predicted angle of
    // attack is sent as feedforward (1: the next sample, + link latency).
    int aoaFeedforwardStage = 1;

    // Launch (optional keys). Before launch the solve starts from the
    // measured position with the reference's first velocity, and the launch
    // is refused while that position is more than launchPositionTolerance [m]
    // from the reference's first sample. In flight once a UAV exceeds
    // inFlightSpeed [m/s]; back to standby if that has not happened
    // launchTimeout [s] after the launch.
    double launchPositionTolerance = 3.0;
    double inFlightSpeed = 10.0;
    double launchTimeout = 1.0;

    std::vector<double> weight; // [Q(nx) R(nu) Qf(nx) Rdu(nu) Rdu0(nu)]
    std::vector<double> lbxStates;
    std::vector<double> ubxStates;
    std::vector<double> lbxControls;
    std::vector<double> ubxControls;
    std::vector<double> scalesStates;
    std::vector<double> scalesControls;
    std::vector<double> invScalesStates;
    std::vector<double> invScalesControls;
};

// NMHE configuration, same conventions as solverConfig. The decision
// variables are the per-stage augmented states [x; wind; d] over a window
// of M+1 samples, with no control decision: nu only sizes the applied
// controls passed as parameters.
struct estimatorConfig {
    int nx;
    int nu;
    int np;
    int nd;
    int nL0;
    int nxi;         // nx + np + nd
    int M;           // window length in steps (M+1 samples)
    int numUavs;
    double dt;       // sample interval [s]
    double tetherL0;

    // Measurement-fit and arrival-cost weights: W_meas(nx), W_windp(np), W_dp(nd).
    std::vector<double> wMeas;
    std::vector<double> wWindPrior;
    std::vector<double> wDPrior;

    // Bounds |wind_i| <= windBound[i] (np) and |d_i| <= dBound[i] (nd),
    // physical units, on the first stage (the identity dynamics carry them
    // to every stage), as build_nmhe_*.m.
    std::vector<double> windBound;
    std::vector<double> dBound;

    std::vector<double> xScale;
    std::vector<double> windScale;
    std::vector<double> dScale;
    std::vector<double> invXScale;
};

#endif //CONTROLLERSTRUCTURES_H
