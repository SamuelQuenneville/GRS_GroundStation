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
    void armAll() const;
    void setModeAll(const std::string& mode) const;
    void startController() const;
    void initLaunch() const;
    void fetchParam(int sysId) const;
    void loadTrajectory(const std::string& file) const;
    void generateTrajectory(const grs::trajgen::TrajectoryConfig& config,
        const grs::trajgen::SubsetSelection& selection = {}) const;
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

private:
    gcsConfig m_gcsConfig;

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
    static std::string m_flightModeToString(mavsdk::Telemetry::FlightMode mode);
    static std::string m_gpsFixToString(mavsdk::Telemetry::FixType fix);
    static std::string m_catapultStateToString(CatapultState state);

    // ADR-001 Phase 2: shared conversion helpers between the trajectory
    // generator/controller and the dashboard's JSON snapshot types.
    // m_buildTrajectorySnapshotFromController() reads back whatever's
    // currently loaded in the NMPC controller (used by both the existing
    // setTrajectoryLoadedCallback and the /api/trajectory/apply response, so
    // "Apply" always reflects what's actually loaded rather than the preview
    // that was requested). m_missionToTrajectorySnapshot() instead converts a
    // freshly-generated, not-yet-applied GeneratedMission -- used for
    // /api/trajectory/generate's pure preview. m_paramsToTrajectoryConfig()
    // maps the dashboard's flat TrajectoryGenerationParams onto a full
    // grs::trajgen::TrajectoryConfig (starting from its defaults, so any
    // field the sidebar doesn't expose keeps its config.m-mirrored value).
    // ADR-001 Phase 4: `sourceUavIndices` labels each output vehicle by its
    // ORIGINAL index in the full mission (e.g. a subset keeping only UAV 2
    // still shows as "UAV 2", not relabeled "UAV 1") -- pass the same
    // indices used to build the (possibly narrowed) mission, or leave empty
    // to label 0..N-1 as UAV 1..N (the full-mission case). `includePayload`
    // controls whether the payload vehicle is included in the snapshot at
    // all, independent of whether `mission.payload` happens to have data --
    // for preview fidelity with whatever will actually reach the controller.
    TrajectorySnapshot m_buildTrajectorySnapshotFromController() const;
    static TrajectorySnapshot m_missionToTrajectorySnapshot(const grs::trajgen::GeneratedMission& mission,
        const std::vector<size_t>& sourceUavIndices, bool includePayload);
    static grs::trajgen::TrajectoryConfig m_paramsToTrajectoryConfig(const TrajectoryGenerationParams& params);

    // ADR-001 Phase 4: maps the sidebar's reduced-order-testing fields onto a
    // SubsetSelection. Returns a default (no-op) selection whenever
    // params.testEnabled is false. `simDt` (from the already-built
    // TrajectoryConfig) converts testMaxDurationSeconds into a sample count.
    static grs::trajgen::SubsetSelection m_paramsToSubsetSelection(const TrajectoryGenerationParams& params, double simDt);

    // ADR-001 Phase 3: answers GET /api/trajectory/live-positions. Reads
    // ControlInterface::getLiveNavigationStates() (already NED-corrected) and
    // splits it into UAVs (sysId <= numUavs()) vs. payload (the highest
    // remaining sysId, if any) -- same convention NMPCController's own
    // state-unpacking already relies on.
    LivePositionsSnapshot m_buildLivePositionsSnapshot() const;

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
