/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "configurationParser.h"

// configurationParser.cpp
gcsConfig ConfigurationParser::parseGcsConfig(YAML::Node& node, const gcsConfig& defaults) {
    gcsConfig config = defaults;

    if (const auto gcsNode = node["GcsConfiguration"]) {
        if (gcsNode["numUavs"])            config.numUavs = gcsNode["numUavs"].as<int>();
        if (gcsNode["verbose"])            config.verbose = gcsNode["verbose"].as<bool>();
        if (gcsNode["hlcFrequency"])        config.hlcFrequency = gcsNode["hlcFrequency"].as<double>();
        if (gcsNode["telemetryPublishHz"])  config.telemetry_publish_hz = gcsNode["telemetryPublishHz"].as<double>();

        if (gcsNode["controlMode"]) {
            const auto mode = gcsNode["controlMode"].as<std::string>();
            if (mode == "MATLAB")             config.controlMode = ControlMode::MATLAB;
            else if (mode == "ATTITUDE_FILE") config.controlMode = ControlMode::ATTITUDE_FILE;
            else                              config.controlMode = ControlMode::MPC;
        }
    }

    if (const auto pixhawkNode = node["Pixhawk"]) {
        config.pixhawk = pixhawkNode.as<pixhawkConfig>();
        if (pixhawkNode["endpoints"])
            config.pixhawkEndpoints = pixhawkNode["endpoints"].as<std::vector<pixhawkEndpointConfig>>();
    }

    if (const auto catapultsNode = node["Catapults"])
        config.catapults = catapultsNode.as<std::vector<catapultEndpointConfig>>();

    return config;
}

solverConfig ConfigurationParser::parseSolverConfig(YAML::Node &node) {
    return node["SolverConfiguration"].as<solverConfig>();
}
