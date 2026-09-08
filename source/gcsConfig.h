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
#include <vector>

#include "Definitions/catapultProtocol.h"

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
    XNAV,
    ACMD,
    FBWA
};

struct flightModeMap : public std::map<std::string, FlightMode> {
    flightModeMap() {
        this->operator[]("INIT") = INIT;
        this->operator[]("MANUAL") = MANUAL;
        this->operator[]("GUIDED") = GUIDED;
        this->operator[]("XNAV") = XNAV;
        this->operator[]("ACMD") = ACMD;
        this->operator[]("FBWA") = FBWA;
    };
    ~flightModeMap() = default;
};

// Reverse of flightModeMap: turns a raw custom_mode value -- read straight
// off a HEARTBEAT's custom_mode field (see CommunicationManager's raw
// heartbeat subscription, uavHealth::customMode) -- back into the same
// mode name setMode()/setModeAll() accept. This is GrsPlane's own mode
// numbering (see GrsPlane/mode.h), NOT the standard ArduPilot/PX4 mode set
// mavsdk::Telemetry::subscribe_flight_mode() translates against -- that
// translation table doesn't know GUIDED=2/XNAV=3 mean something custom
// here, so it either misclassifies them as a same-numbered stock mode or
// falls back to Unknown. Reading the raw HEARTBEAT and mapping it through
// this instead sidesteps MAVSDK's translation entirely, with no changes
// to MAVSDK itself.
inline std::string flightModeToString(const uint32_t customMode) {
    static const flightModeMap modes;
    for (const auto& [name, value] : modes) {
        if (static_cast<uint32_t>(value) == customMode) return name;
    }
    return "UNKNOWN(" + std::to_string(customMode) + ")";
}

struct pixhawkConfig {
    std::string remoteIP = "127.0.0.1";
    int tcpPort = 5760;                 // Ardupilot specific
    int tcpPortIncrement = 10;          // Ardupilot specific
    bool sitl = false;
};

struct pixhawkEndpointConfig {
    uint8_t id;
    std::string ip;
    uint16_t port;
};

struct catapultEndpointConfig {
    uint8_t id;
    uint16_t port = CATAPULT_PORT;
    std::string expectedIp;
};

struct gcsConfig {
    bool verbose = false;
    // Gates the heavy per-tick CSV dumps (solver args/output, raw states,
    // raw controls -- see Logger::start()). Default true to keep existing
    // behavior. The NMPC controller's sparse event log (launch, in-flight,
    // trajectory ended/loaded, solver violation entered/cleared) is
    // unaffected by this -- it's always on, see LogType::NMPC_EVENT.
    bool verboseLogging = false;
    int numUavs = 1;
    double telemetry_publish_hz = -1.0;
    double hlcFrequency = 20.0;
    pixhawkConfig pixhawk;
    std::vector<pixhawkEndpointConfig> pixhawkEndpoints; // used when pixhawk.sitl == false
    std::optional<std::pair<std::string, uint16_t>> matlab;
    std::optional<std::string> attitudeFile;
    std::optional<std::string> rcFile;
    ControlMode controlMode = ControlMode::MPC;
    std::vector<catapultEndpointConfig> catapults;
    std::string configPath = "inputFilesExamples/configuration.yaml";
};

#endif //GCSCONFIG_H
