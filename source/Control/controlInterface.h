/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLINTERFACE_H
#define CONTROLINTERFACE_H

#include <thread>
#include <atomic>
#include <chrono>
#include <map>
#include <optional>
#include <arpa/inet.h>

#include "gcsConfig.h"
#include "Definitions/communicationStructures.h"
#include "Configuration/configurationParser.h"
#include "Log/programLogger.h"
#include "Powertrain/powertrain.h"
#include "navigationFrameManager.h"
#include "NMPCController.h"
#include "Trajectory/trajectoryGenerator.h"

class ControlInterface {

public:
    ControlInterface();
    ~ControlInterface();

    void initialize(const gcsConfig& config);
    void start();
    void stop();

    void setCommandCallback(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> cb);
    void updateStates(const std::map<uint8_t, uavStates>& states);

    // Fired once per control-loop iteration while running in MPC mode, right
    // after NMPCController::solve() returns. No-op in MATLAB/ATTITUDE_FILE
    // mode since there's no NMPC controller to report on.
    void setNmpcDebugCallback(std::function<void(const NMPCController::DebugInfo&)> cb);

    void setOriginCallback(std::function<void(double latitudeDegrees, double longitudeDegrees, double altitude)> cb);
    void setTrajectoryLoadedCallback(std::function<void()> cb);

    // Passthrough accessors for setup/orientation tooling -- null-safe, since
    // m_nmpc only exists in MPC control mode (see initialize()).
    std::vector<NMPCController::TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const;
    int numUavs() const;
    bool trajectoryHasPayload() const;
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    void initMatlabConnection(const char* ip, uint16_t port);
    void setCommandsList(const std::map<uint8_t, std::vector<uavCommandsFlags>>& commandsList);

    // Raw WGS84 GPS fix -- lat/lon/AMSL altitude straight from the latest
    // telemetry, deliberately NOT run through NavigationFrameManager (there's
    // no origin yet; this is what *establishes* one -- see
    // GroundControlStation::setOriginFromPayload()).
    struct GpsFix {
        double latitudeDegrees = 0.0;
        double longitudeDegrees = 0.0;
        double altitudeMeters = 0.0;
    };

    // The payload's current raw GPS fix, for setting the navigation origin
    // directly from where the payload actually is instead of typing lat/lon
    // by hand. Same "payload = highest sysId" convention as
    // NMPCController::m_unpackLatestStates / getLiveNavigationStates() below
    // -- any sysId beyond m_config.numUavs is the payload (highest wins if
    // more than one, matching that convention's tie-break). Returns nullopt
    // if no such telemetry has arrived yet.
    [[nodiscard]] std::optional<GpsFix> getPayloadGpsFix() const;

    void initLaunch() const;

    void loadTrajectory(const std::string& file) const;

    // ADR-001 Phase 1/2: builds a trajectory in-process with
    // TrajectoryGenerator (no MATLAB, no CSV round-trip), applies field
    // calibration (config.fieldHeadingDeg / originOffsetNed -- no-ops at
    // their defaults), and loads it directly into the NMPC controller.
    // Fires the same setTrajectoryLoadedCallback() as loadTrajectory(file),
    // so the existing setup3d.html / GET /api/trajectory path picks it up
    // with no dashboard changes.
    //
    // ADR-001 Phase 4: `selection` narrows the generated mission before it's
    // sent to the controller -- e.g. one UAV, no payload, only through the
    // first loiter, for exercising a reduced-order NMPC build. The default
    // (no selection) is a strict no-op: full mission, and `hasPayload`
    // deferred to the loaded NMPCController's own hasPayload(), exactly like
    // before this existed.
    void generateTrajectory(const grs::trajgen::TrajectoryConfig& config,
        const grs::trajgen::SubsetSelection& selection = {}) const;

    // Pure computation, does not touch the NMPC controller -- for the
    // dashboard's generate/preview step (POST /api/trajectory/generate)
    // before the operator commits with generateTrajectory()/"Apply". Safe to
    // call even before initialize() (unlike generateTrajectory(), it doesn't
    // need m_nmpc). See generateTrajectory() above for what `selection` does.
    [[nodiscard]] grs::trajgen::GeneratedMission previewTrajectory(const grs::trajgen::TrajectoryConfig& config,
        const grs::trajgen::SubsetSelection& selection = {}) const;
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

    // ADR-001 Phase 3: real launch-position capture for the trajectory
    // generator sidebar. Mirrors exactly what m_controlLoop() feeds
    // NMPCController every tick -- the latest telemetry, corrected into the
    // NavigationFrameManager's NED frame -- so a captured "live" position is
    // the same NED the rest of the system already trusts. Returns an empty
    // map if the nav frame hasn't been initialized yet (no origin / no GPS
    // lock), so callers can tell "no fix yet" from "fix at the origin".
    // Payload convention, matching NMPCController::m_unpackLatestStates:
    // when present, the payload is whichever entry has the highest sysId.
    [[nodiscard]] std::map<uint8_t, uavStates> getLiveNavigationStates() const;

private:
    NavigationFrameManager m_navFrameManager;

    void m_controlLoop();

    void m_initMatlabConnection(const char* ip, uint16_t port);
    void m_sendDataToMatlab(const std::map<uint8_t, uavStates>& states);
    std::map<uint8_t, uavCommands> m_receiveDataFromMatlab();

    std::atomic<bool> m_running;
    std::thread m_controllerThread;
    double m_fileFrequency = 10.0;

    gcsConfig m_config;

    std::unique_ptr<NMPCController> m_nmpc;

    std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> m_sendCommand;
    std::function<void(const NMPCController::DebugInfo&)> m_nmpcDebugCallback;
    std::function<void(double, double, double)> m_originCallback;
    std::function<void()> m_trajectoryLoadedCallback;
    std::map<uint8_t, uavStates> m_latestStates;
    mutable std::mutex m_stateMutex; // locked from const getLiveNavigationStates() too

    std::map<uint8_t, std::vector<uavCommandsFlags>> m_commandsList{};

    int m_udpSocketMatlab = 0;
    sockaddr_in m_matlabAddress{};
};


#endif //CONTROLINTERFACE_H
