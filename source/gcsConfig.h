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
#include <utility>
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
    // Launch refused until the params matching these prefixes are identical on every UAV.
    bool paramCheck = true;
    std::vector<std::string> paramCheckPrefixes{"APTCH_", "ARLL_", "RPM_", "RPM1_SCALING", "SCALING_SPEED"};
};

struct pixhawkEndpointConfig {
    uint8_t id;
    std::string ip;
    uint16_t port = 0;
    std::string uri; // any MAVSDK connection (e.g. serial:///dev/ttyACM0:115200), replaces ip/port
};

struct catapultEndpointConfig {
    uint8_t id;
    uint16_t port = CATAPULT_PORT;
    std::string expectedIp;
};

// Emulated delay of one link direction of one vehicle (see LinkEmulator).
// Every mode adds delayMs and an exponential jitter of mean jitterMs.
struct linkDelayConfig {
    enum class Mode { NONE, CONSTANT, PARAMETRIC, TRACE };
    Mode mode = Mode::NONE;
    double delayMs = 0.0;
    double jitterMs = 0.0;
    // PARAMETRIC: congestion episodes (Poisson, episodeRateHz). The delay grows by
    // `growth` [s/s] up to a peak of peakMinMs + lognormal(peakMedianMs, peakSigma),
    // at most peakMaxMs, then drops back.
    double episodeRateHz = 0.0;
    double peakMinMs = 30.0;
    double peakMedianMs = 100.0;
    double peakSigma = 0.6;
    double peakMaxMs = 900.0;
    double growth = 0.8;
    // TRACE: excess delay [ms] against time since launch [s], read from traceFile
    // (CSV, one header line). 0 before launch and after the end of the trace.
    std::string traceFile;
    std::vector<std::pair<double, double>> trace;
};

// For simulation only: adds delay to whatever link the GCS uses.
struct linkEmulationConfig {
    bool enabled = false;
    uint32_t seed = 1;
    std::map<uint8_t, linkDelayConfig> downlink; // CONTROL_SYSTEM_STATE, by sysId
    std::map<uint8_t, linkDelayConfig> uplink;   // SET_ATTITUDE_TARGET, by sysId
};

struct gcsConfig {
    bool verbose = false;
    // Enables the per-tick CSV logs (solver inputs/outputs, states,controls). NMPC_EVENT is always logged.
    bool verboseLogging = false;
    int numUavs = 1;
    double telemetry_publish_hz = -1.0;
    double hlcFrequency = 20.0;
    // NMHE rate [Hz], independent of hlcFrequency. Used only when the YAML has an EstimatorConfiguration section.
    double nmheFrequency = 5.0;
    // MPC mode: no command while a vehicle's last CONTROL_SYSTEM_STATE is older than this [s].
    double telemetryTimeout = 0.3;
    // CONTROL_SYSTEM_STATE rate requested from each vehicle [Hz]. ArduPilot
    // refuses more than 80 % of its main loop rate (SCHED_LOOP_RATE, 50 Hz on Plane).
    double stateRateHz = 40.0;
    pixhawkConfig pixhawk;
    std::vector<pixhawkEndpointConfig> pixhawkEndpoints; // used when pixhawk.sitl == false
    std::optional<std::pair<std::string, uint16_t>> matlab;
    std::optional<std::string> attitudeFile;
    ControlMode controlMode = ControlMode::MPC;
    std::vector<catapultEndpointConfig> catapults;
    linkEmulationConfig linkEmulation;
    std::string configPath = "inputFilesExamples/configuration.yaml";
    // --run-dir: logs go to <runDirectory>/gcs instead of logGcs/<date>.
    std::string runDirectory;
    // Arguments as typed, recorded in the log manifest.
    std::string commandLine;
};

#endif //GCSCONFIG_H
