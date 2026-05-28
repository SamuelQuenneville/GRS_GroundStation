/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GCSCONFIG_H
#define GCSCONFIG_H

#include <cstdint>
#include <string>
#include <optional>
#include <map>

enum class ControlMode {
    MPC,
    MATLAB,
    ATTITUDE_FILE
};

// As define in GrsPlane/mode.h
enum FlightMode {
    INIT,
    MANUAL,
    GUIDED,
    XNAV
};

struct flightModeMap : public std::map<std::string, FlightMode> {
    flightModeMap() {
        this->operator[]("INIT") = INIT;
        this->operator[]("MANUAL") = MANUAL;
        this->operator[]("GUIDED") = GUIDED;
        this->operator[]("XNAV") = XNAV;
    };
    ~flightModeMap() = default;
};

struct pixhawkConfig {
    std::string remoteIP = "127.0.0.1";
    int tcpPort = 5760;                 // Ardupilot specific
    int tcpPortIncrement = 10;          // Ardupilot specific
    bool sitl = false;
};

struct gcsConfig {
    bool verbose = false;
    int numUavs = 1;
    double telemetry_publish_hz = -1.0;
    double hlcFrequency = 20.0;
    pixhawkConfig pixhawk;
    std::optional<std::pair<std::string, uint16_t>> matlab;
    std::optional<std::string> attitudeFile;
    std::optional<std::string> rcFile;
    ControlMode controlMode = ControlMode::MPC;
};

#endif //GCSCONFIG_H
