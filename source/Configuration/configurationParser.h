/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONFIGURATIONPARSER_H
#define CONFIGURATIONPARSER_H

#include "gcsConfig.h"
#include "yaml-cpp/yaml.h"
#include "Definitions/yamlCustomStructure.h"

class ConfigurationParser {

public:
    ConfigurationParser() = default;
    ~ConfigurationParser() = default;

    static gcsConfig parseGcsConfig(YAML::Node& node, const gcsConfig& defaults = {});
    static solverConfig parseSolverConfig(YAML::Node& node);

    // Returns std::nullopt when the YAML has no "EstimatorConfiguration"
    // section -- that's how ControlInterface decides whether an
    // Estimator/NmheEstimator gets constructed at all (no separate enable
    // flag; presence of the section IS the enable, same as
    // SolverConfiguration implicitly gates MPC mode).
    static std::optional<estimatorConfig> parseEstimatorConfig(YAML::Node& node);

};



#endif //CONFIGURATIONPARSER_H
