/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NMPCCONTROLLER_H
#define NMPCCONTROLLER_H

#pragma once

#include <map>
#include <cstring>
#include <casadi/casadi.hpp>

#include "Definitions/communicationStructures.h"

// CasADi-generated solver
#include "solver.h"

class NMPCController {
public:

    struct solverConfig {
        int nx;         // state dimension per UAV
        int nu;         // control dimension per UAV
        int np;         // parameter vector length used by the solver
        int N;          // prediction horizon
        int numUavs;    // number of UAVs in the solver dynamic model
        std::vector<double> lbxStates;
        std::vector<double> ubxStates;
        std::vector<double> lbxControls;
        std::vector<double> ubxControls;
    };

    struct referencePoint {
        std::vector<double> statesRef;    // size of m_config.nx
        std::vector<double> controlsRef;  // size of m_config.nu
    };

    explicit NMPCController(const solverConfig& config);
    ~NMPCController();

    void initLaunch();

    // Main entry point: convert states → run solver → return commands
    std::map<uint8_t, uavCommandsFlags> solve(const std::map<uint8_t, uavStates>& latestStates);
    double lastSolveMs() const;

private:
    solverConfig m_config;
    std::vector<referencePoint> m_referenceTrajectory;

    bool m_launched = false;
    bool m_endedTraj = false;
    int m_idxTraj = 0;

    // Solver memory handle
    int m_mem = -1;

    // Solver C API pointers
    std::vector<const casadi_real*> m_arg;  // Input pointers
    std::vector<casadi_real*>       m_res;  // Output pointers

    // Solver Inputs
    std::vector<casadi_real> m_x0;
    std::vector<casadi_real> m_p;
    std::vector<casadi_real> m_lbx;
    std::vector<casadi_real> m_ubx;
    std::vector<casadi_real> m_lbg;
    std::vector<casadi_real> m_ubg;
    std::vector<casadi_real> m_lam_x0;
    std::vector<casadi_real> m_lam_g0;

    // Solver Outputs
    std::vector<casadi_real> m_x;
    std::vector<casadi_real> m_f;
    std::vector<casadi_real> m_g;
    std::vector<casadi_real> m_lam_x;
    std::vector<casadi_real> m_lam_g;
    std::vector<casadi_real> m_lam_p;

    // CasADi workspace arrays
    std::vector<casadi_int>  m_iw;
    std::vector<casadi_real> m_w;

    // Synchronization
    mutable std::mutex m_solveMutex;

    // Timing
    double m_lastSolveMs = -1.0;

    void m_loadTrajectory(const std::string& path);

    void m_initalizeSolverIO();
    void m_bindSolverIO();

    void m_shiftSolution();
    void m_packBounds();
    void m_packInitialGuess();
    void m_packParameters(const std::map<uint8_t, uavStates>& latestStates);
    std::map<uint8_t, uavCommandsFlags> m_extractControls() const;

    static std::vector<double> m_unpackLatestStates(const std::map<uint8_t, uavStates>& latestStates);
};

#endif //NMPCCONTROLLER_H
