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
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude) const;
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
