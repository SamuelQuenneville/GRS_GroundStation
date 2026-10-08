/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GCS_H
#define GCS_H

#include "Dashboard/dashboardServer.h"
#include "Communication/communicationManager.h"
#include "Communication/rtkBaseStation.h"
#include "Communication/catapultLauncher.h"
#include "Control/controlInterface.h"
#include "Control/controlDispatcher.h"
#include "Log/logger.h"
#include "gcsConfig.h"

class GroundControlStation {

public:
    GroundControlStation();
    ~GroundControlStation();

    void initialize(const gcsConfig& config);

    void setDashboard(DashboardServer* dashboard);

    void start();
    void stop();

    void connectAll();
    void armAll(bool force = false) const;
    // HOME of every vehicle at its current position (RTL and failsafe target).
    void setHomeAll() const;
    void setModeAll(const std::string& mode) const;
    void startController() const;
    void initLaunch() const;
    void fetchParam(int sysId) const;
    void loadTrajectory(const std::string& file) const;
    void generateTrajectory(const grs::trajgen::TrajectoryConfig& config,
        const grs::trajgen::SubsetSelection& selection = {},
        const std::vector<std::optional<grs::Vec3d>>& liveLaunchPositionsNed = {}) const;

    // Writes the reference applied in the controller, in loadTrajectory()'s
    // CSV format. Also called after each Apply, so a restart does not lose it.
    // `file`: path, or empty for ./trajectories/ with a timestamp. Returns
    // the path written. Throws if not in MPC mode, nothing is applied, or
    // the file can't be written.
    std::string saveTrajectory(const std::string& file = "") const;
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude) const;

    // Captures the payload's current raw GPS fix and uses it directly as the
    // NavigationFrameManager origin, instead of the operator typing lat/lon
    // into `setOrigin` by hand. Returns false (and logs why) if no payload
    // GPS fix has arrived yet -- e.g. its Pixhawk isn't connected/streaming.
    bool setOriginFromPayload() const;

    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

    // Starts reading the RTK base GPS (e.g. u-blox F9P) on `device` and
    // forwards corrections to every connected Pixhawk. baudrate = 0 to auto-detect.
    void startRtkBase(const std::string& device, unsigned baudrate);
    void stopRtkBase() const;

    void catapultConnect() const;
    void catapultArm() const;
    void catapultFire(uint32_t countdownMs = 500) const;
    void catapultAbort() const;
    void catapultDisarm() const;
    void catapultStatus() const;

    // Reports exactly which sysIds are fully registered right now (past
    // MAVSDK's has_autopilot()/is_connected() handshake, actually
    // subscribed to telemetry) -- useful when a vehicle "seems to connect"
    // but its dashboard card never appears, to tell a genuinely-stuck
    // registration apart from a telemetry/display issue further downstream.
    void listLinks() const;

private:
    gcsConfig m_gcsConfig;

    // NMPC dashboard panel context -- see setNmpcDebugCallback/
    // setTrajectoryLoadedCallback in the constructor. m_loopPeriodMs is set
    // once in initialize(); m_trajectoryLoadedAtMs is stamped every time a
    // trajectory is loaded/generated/applied.
    double m_loopPeriodMs = 0.0;
    uint64_t m_trajectoryLoadedAtMs = 0;

    DashboardServer* m_dashboardServer = nullptr;

    // Cached per-UAV state used to build dashboard snapshots: numeric
    // telemetry (fast, from CommunicationManager::setTelemetryCallback) and
    // status (slow/event-driven, from setStatusCallback) arrive on separate
    // callbacks, but DashboardServer::updateTelemetry() replaces the whole
    // per-UAV snapshot each time -- so every push needs to merge both.
    std::mutex m_dashboardMutex;
    std::map<uint8_t, uavStates> m_latestUavStates;
    std::map<uint8_t, uavHealth> m_latestUavHealth;
    void m_pushDashboardSnapshot(uint8_t sysId);
    static std::string m_gpsFixToString(mavsdk::Telemetry::FixType fix);

    // Dashboard snapshots of a reference trajectory:
    // - from the controller: what is actually applied;
    // - from a generated, not yet applied mission (preview). sourceUavIndices
    //   labels each vehicle by its index in the full mission (empty: 0..N-1).
    // m_paramsToTrajectoryConfig() starts from TrajectoryConfig's defaults
    // for the fields the dashboard does not expose.
    [[nodiscard]] TrajectorySnapshot m_buildTrajectorySnapshotFromController() const;
    static TrajectorySnapshot m_missionToTrajectorySnapshot(const grs::trajgen::GeneratedMission& mission,
        const std::vector<size_t>& sourceUavIndices, bool includePayload);
    static grs::trajgen::TrajectoryConfig m_paramsToTrajectoryConfig(const TrajectoryGenerationParams& params);

    // Reduced-order test fields to a SubsetSelection (no-op if
    // params.testEnabled is false). simDt converts the max duration to samples.
    static grs::trajgen::SubsetSelection m_paramsToSubsetSelection(const TrajectoryGenerationParams& params, double simDt);

    // "./trajectories/trajectory_<YYYY-MM-DD_HH-MM-SS>.csv", directory created if needed.
    static std::string m_defaultTrajectorySavePath();

    // GET /api/trajectory/live-positions: live NED positions of the UAVs and
    // the payload (highest sysId above numUavs).
    [[nodiscard]] LivePositionsSnapshot m_buildLivePositionsSnapshot() const;

    // Launch positions captured by the operator, indexed like
    // config.aircraftPath.phaseRad (2 UAVs). Does not read live telemetry,
    // so repeated Generate/Apply reuse the same capture. nullopt where
    // nothing was captured, and everywhere if snapToLiveLaunchPosition is off.
    static std::vector<std::optional<grs::Vec3d>> m_paramsToLiveLaunchPositions(const TrajectoryGenerationParams& params);

    std::unique_ptr<CommunicationManager> m_communicationManager;
    std::unique_ptr<ControlDispatcher>    m_controlDispatcher;
    std::unique_ptr<ControlInterface>     m_controlInterface;
    std::unique_ptr<RtkBaseStation>       m_rtkBaseStation;
    std::unique_ptr<CatapultLauncher> m_catapultLauncher;

    void m_parseCommandFile(const std::string& file) const;
    static bool m_parseUavCommandsLine(const std::string& line, uavCommandsFlags& commands);

    void m_supervisorLoop() const;
    std::thread m_supervisorThread;
    std::atomic<bool> m_running;
};

#endif //GCS_H
