/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef PLANTMODEL_H
#define PLANTMODEL_H

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Truth-plant models for grs_batchsim: C++ ports of GRS_Controller's
//   01_models/02_twoUav/grsTwoUavPayloadDynamicAugmented.m
//   01_models/01_oneUav/grsOneGroundDynamicAugmented.m
// (point mass + first-order attitude lag, same ode and alpha outputs), with
// every physical constant a RUNTIME parameter so a Monte Carlo sample can
// perturb the truth plant without regenerating anything. The controller and
// estimator never see these structs: they keep the nominal model baked into
// their generated solvers.
//
// Hand-ported rather than CasADi-generated on purpose: a generated Function
// bakes params1/params2/rig in as constants, which is exactly what the Monte
// Carlo campaign needs to vary. Equivalence with the MATLAB model is checked
// by plant_model_tests against golden data written by
// GRS_Controller/02_matlab/06_comparison/cpp_batchsim/export_plant_golden.m.
namespace grs::sim {

// uavParams_Sig.m
struct AirframeParams {
    double mass = 4.475;       // [kg]
    double T_roll = 0.2215;    // [s]
    double T_pitch = 0.1832;   // [s]
    double S = 0.53625;        // [m^2]
    double CL0 = 0.21385;      // [-]
    double CL_alpha = 4.32893; // [1/rad]
    double CD0 = 0.05;         // [-]
    double AR = 4.9242;        // [-]
    double e = 0.92;           // [-]
};

// rigParams_twoUavPayload.m (payload/ground fields unused by the one-UAV
// model, whose tether/environment constants are the same literals).
struct RigParams {
    double m_pay = 20.0;            // [kg]
    double k_t = 1500.0;            // [N/m]
    double b_t = 30.0;              // [N.s/m]
    double tether_gate_gain = 20.0; // [1/m]
    double k_ground = 9810.0;       // [N/m]
    double b_ground = 620.0;        // [N.s/m]
    double kappa_ground = 40.0;     // [1/m]
    double g = 9.81;                // [m/s^2]
    double rho = 1.225;             // [kg/m^3]
};

class PlantModel {
public:
    virtual ~PlantModel() = default;

    [[nodiscard]] virtual int nx() const = 0;
    [[nodiscard]] virtual int nu() const = 0;
    [[nodiscard]] virtual int np() const = 0;
    [[nodiscard]] virtual int nd() const = 0;
    [[nodiscard]] virtual int numUavs() const = 0;
    [[nodiscard]] virtual bool hasPayload() const = 0;
    [[nodiscard]] virtual const RigParams& rig() const = 0;

    // F(x,u,wind,d,L0) -> {ode, alpha_1..alpha_numUavs}. xdot has nx()
    // entries, alpha numUavs() entries (may be null).
    virtual void evaluate(const double* x, const double* u, const double* wind, const double* d,
                          double L0, double* xdot, double* alpha) const = 0;
};

class TwoUavPayloadPlant final : public PlantModel {
public:
    TwoUavPayloadPlant(const AirframeParams& uav1, const AirframeParams& uav2, const RigParams& rig);

    [[nodiscard]] int nx() const override { return 22; }
    [[nodiscard]] int nu() const override { return 6; }
    [[nodiscard]] int np() const override { return 3; }
    [[nodiscard]] int nd() const override { return 10; }
    [[nodiscard]] int numUavs() const override { return 2; }
    [[nodiscard]] bool hasPayload() const override { return true; }
    [[nodiscard]] const RigParams& rig() const override { return m_rig; }

    void evaluate(const double* x, const double* u, const double* wind, const double* d,
                  double L0, double* xdot, double* alpha) const override;

private:
    AirframeParams m_uav[2];
    RigParams m_rig;
};

class OneGroundPlant final : public PlantModel {
public:
    OneGroundPlant(const AirframeParams& uav, const RigParams& rig);

    [[nodiscard]] int nx() const override { return 8; }
    [[nodiscard]] int nu() const override { return 3; }
    [[nodiscard]] int np() const override { return 3; }
    [[nodiscard]] int nd() const override { return 5; }
    [[nodiscard]] int numUavs() const override { return 1; }
    [[nodiscard]] bool hasPayload() const override { return false; }
    [[nodiscard]] const RigParams& rig() const override { return m_rig; }

    void evaluate(const double* x, const double* u, const double* wind, const double* d,
                  double L0, double* xdot, double* alpha) const override;

private:
    AirframeParams m_uav;
    RigParams m_rig;
};

enum class IntegrationMethod { RK4, RK2, Euler };
IntegrationMethod parseIntegrationMethod(const std::string& name);

// doStepFineAug.m / integrate_dynamics.m: advances x by dt with nSub equal
// sub-steps of the chosen explicit method, u/wind/d/L0 held constant.
void integrate(const PlantModel& plant, std::vector<double>& x, const std::vector<double>& u,
               const std::vector<double>& wind, const std::vector<double>& d, double L0,
               double dt, int nSub = 8, IntegrationMethod method = IntegrationMethod::RK4);

} // namespace grs::sim

#endif //PLANTMODEL_H
