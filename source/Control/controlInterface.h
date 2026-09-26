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
#include <ranges>

#include "gcsConfig.h"
#include "Definitions/communicationStructures.h"
#include "Powertrain/powertrain.h"
#include "navigationFrameManager.h"
#include "Mathematics/math.h"
#include "SolverBackend/solverBackendFactory.h"
#include "controlStep.h"
#include "Trajectory/trajectoryGenerator.h"

// Controller/Estimator-agnostic on purpose -- ControlInterface never needs to know whether the active controller is
// MpcController or something else, only that it implements Controller. mpcController.h is only ever included by the
// .cpp, which is the one place that actually constructs one.
#include "controller.h"
#include "estimator.h"

class ControlInterface {

public:
    ControlInterface();
    ~ControlInterface();

    void initialize(const gcsConfig& config);
    void start();
    void stop();

    void setCommandCallback(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> cb);
    void updateStates(const std::map<uint8_t, uavStates>& states);

    void setNmpcDebugCallback(std::function<void(const Controller::DebugInfo&)> cb);

    void setOriginCallback(std::function<void(double latitudeDegrees, double longitudeDegrees, double altitude)> cb);
    void setTrajectoryLoadedCallback(std::function<void()> cb);

    // Passthrough accessors for setup/orientation tooling
    std::vector<Controller::TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const;
    int numUavs() const;
    bool trajectoryHasPayload() const;
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    void initMatlabConnection(const char* ip, uint16_t port);
    void setCommandsList(const std::map<uint8_t, std::vector<uavCommandsFlags>>& commandsList);

    // Raw WGS84 GPS fix -- lat/lon/AMSL altitude straight from the latest telemetry, deliberately NOT run through
    // NavigationFrameManager (there's no origin yet; this is what set one --> see GroundControlStation::setOriginFromPayload()).
    struct GpsFix {
        double latitudeDegrees = 0.0;
        double longitudeDegrees = 0.0;
        double altitudeMeters = 0.0;
    };

    // The payload's current raw GPS fix, for setting the navigation origin directly from where the payload actually is
    // instead of typing lat/lon by hand. Same "payload = highest sysId" convention as MpcController::m_unpackLatestStates
    // / getLiveNavigationStates() below.
    [[nodiscard]] std::optional<GpsFix> getPayloadGpsFix() const;

    void initLaunch() const;

    void loadTrajectory(const std::string& file) const;
    void saveTrajectory(const std::string& file) const;

    void generateTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection = {}, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed = {}) const;

    // Pure computation, does not touch the NMPC controller. For the dashboard's generate/preview step
    // (POST /api/trajectory/generate) before the operator commits with generateTrajectory()/"Apply". Safe to
    // call even before initialize() (unlike generateTrajectory(), it doesn't need m_controller). See generateTrajectory()
    // above for what `selection` and`liveLaunchPositionsNed` do.
    [[nodiscard]] static grs::trajgen::GeneratedMission previewTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection = {}, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed = {});
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

    // Real launch-position capture for the trajectory generator sidebar. The latest telemetry, corrected into the
    // NavigationFrameManager's NED frame, so a captured "live" position is the same NED the rest of the system already
    // trusts. Returns an empty map if the nav frame hasn't been initialized yet (no origin / no GPS lock), so callers
    // can tell "no fix yet" from "fix at the origin". Payload convention, matching MpcController::m_unpackLatestStates
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

    std::unique_ptr<Controller> m_controller;
    std::unique_ptr<Estimator> m_estimator;

    // Per-tick controller logic (MPC mode only), shared with grs_batchsim --> see controlStep.h.
    // References m_controller/m_estimator and owns the NMHE thread (ThreadedEstimatorRunner). Declared after both
    // so it is destroyed first: the NMHE thread is joined before m_estimator goes away.
    std::unique_ptr<ControlStep> m_controlStep;

    std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> m_sendCommand;
    std::function<void(const Controller::DebugInfo&)> m_nmpcDebugCallback;
    std::function<void(double, double, double)> m_originCallback;
    std::function<void()> m_trajectoryLoadedCallback;
    std::map<uint8_t, uavStates> m_latestStates;
    mutable std::mutex m_stateMutex; // locked from const getLiveNavigationStates() too

    std::map<uint8_t, std::vector<uavCommandsFlags>> m_commandsList{};

    int m_udpSocketMatlab = 0;
    sockaddr_in m_matlabAddress{};
};


#endif //CONTROLINTERFACE_H
