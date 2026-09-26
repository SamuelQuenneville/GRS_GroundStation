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
#include "controlStep.h"
#include "Trajectory/trajectoryGenerator.h"

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

    std::vector<Controller::TrajectoryPointView> getTrajectoryForVehicle(int vehicleIndex) const;
    int numUavs() const;
    bool trajectoryHasPayload() const;
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    void initMatlabConnection(const char* ip, uint16_t port);
    void setCommandsList(const std::map<uint8_t, std::vector<uavCommandsFlags>>& commandsList);

    // Raw WGS84 fix from telemetry (AMSL altitude), not converted to NED: used to set the origin.
    struct GpsFix {
        double latitudeDegrees = 0.0;
        double longitudeDegrees = 0.0;
        double altitudeMeters = 0.0;
    };

    // The payload is the highest sysId above numUavs (stateVector.h).
    [[nodiscard]] std::optional<GpsFix> getPayloadGpsFix() const;

    void initLaunch() const;

    void loadTrajectory(const std::string& file) const;
    void saveTrajectory(const std::string& file) const;

    void generateTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection = {}, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed = {}) const;

    // Generates without applying it to the controller (dashboard preview); usable before initialize().
    [[nodiscard]] static grs::trajgen::GeneratedMission previewTrajectory(const grs::trajgen::TrajectoryConfig& config, const grs::trajgen::SubsetSelection& selection = {}, const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed = {});
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

    // Latest telemetry in the NED frame, by sysId (payload = highest sysId). Empty until the frame is initialized
    // (no origin or no GPS lock).
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

    // MPC-mode tick. Declared after m_controller/m_estimator so it is destroyed first: its NMHE thread must stop
    // before the estimator goes away.
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
