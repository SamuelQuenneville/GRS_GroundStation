/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONFIGURATIONPARSER_H
#define CONFIGURATIONPARSER_H

#include "yaml-cpp/yaml.h"
#include "Definitions/yamlCustomStructure.h"

class ConfigurationParser {

public:
    ConfigurationParser() = default;
    ~ConfigurationParser() = default;

    static solverConfig parseSolverConfig(YAML::Node& node);

};



#endif //CONFIGURATIONPARSER_H
