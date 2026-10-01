/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef TRUTHSAMPLE_H
#define TRUTHSAMPLE_H

#pragma once

#include <map>
#include <memory>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "plantModel.h"

namespace grs::sim {

// One Monte Carlo sample: sample_id + named parameter values, one row of the
// samples.csv written by GRS_Controller's mc_export_samples_csv.m (same
// column names as mc_param_space_twoUav.m). Parameters absent from the row
// are nominal.
struct Sample {
    int id = 0;
    std::map<std::string, double> values;
};

// Reads samples.csv (header: sample_id,<name>,<name>,...). The special
// path "nominal" returns a single all-nominal sample with id 0.
std::vector<Sample> readSamples(const std::string& path);

// Everything the truth plant needs for one run -- C++ counterpart of
// mc_apply_sample_twoUav.m's output struct.
struct TruthSpec {
    AirframeParams uav[2];
    RigParams rig;
    std::vector<double> windTrue;  // np, NED [m/s]
    std::vector<double> dTrue;     // nd, same layout as the model's d
    double L0Plant = 30.0;         // true tether rest length [m]
    double L0Ctrl = 30.0;          // value the controller/NMHE use (YAML L0)
};

// Same mapping as mc_apply_sample_twoUav.m:
//   'rel' parameters (a<i>_<coef>, pay_mass, tether_k, tether_b) multiply
//   the nominal value (1 = nominal, must be > 0), 'abs' parameters are
//   values in their unit (wind_speed m/s, wind_heading deg toward, wind_down
//   m/s, d<i>_broll/bpitch deg, tether_L0_err m; two UAVs: d<i>_Fa N,
//   d<i>_CL, pay_Fz N; one UAV: d1_F<xyz> N).
// numUavs selects the parameter set that exists (1: a1_*, d1_*; 2: both).
// Unknown names throw, so a typo cannot silently leave a parameter nominal.
TruthSpec applySample(const Sample& sample, int numUavs, double L0Nominal);

// Builds the truth plant for a spec (TwoUavPayloadPlant for numUavs=2,
// OneGroundPlant for numUavs=1).
std::unique_ptr<PlantModel> makePlant(const TruthSpec& spec, int numUavs);

} // namespace grs::sim

#endif //TRUTHSAMPLE_H
