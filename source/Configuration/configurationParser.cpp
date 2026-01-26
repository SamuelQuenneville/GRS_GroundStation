/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "configurationParser.h"

solverConfig ConfigurationParser::parseSolverConfig(YAML::Node &node) {
    return node["SolverConfiguration"].as<solverConfig>();
}
