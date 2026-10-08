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

    // std::nullopt without an "EstimatorConfiguration" section: the section itself enables the estimator.
    static std::optional<estimatorConfig> parseEstimatorConfig(YAML::Node& node);

};

#endif //CONFIGURATIONPARSER_H