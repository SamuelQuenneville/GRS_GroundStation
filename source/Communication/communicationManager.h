/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef COMMUNICATIONMANAGER_H
#define COMMUNICATIONMANAGER_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/rtk/rtk.h>

#include <thread>

#include "gcsConfig.h"
#include "statesAggregator.h"
#include "Definitions/communicationStructures.h"


#define GROUND_STATION mavsdk::Mavsdk::Configuration(255, MAV_COMP_ID_MISSIONPLANNER, true)

// MAVSDK links to every vehicle: registration, telemetry in, commands out.
//
// Threads: MAVSDK callbacks, registration threads (one per new vehicle), the
// publish thread, and the callers of the public methods (console, dashboard,
// dispatcher). m_linkMutex guards m_vehicles and m_watchers, m_statesMutex
// the aggregators and healths, m_requestMutex the request bookkeeping. A
// plugin is copied out (shared_ptr) under the lock and used outside it.
class CommunicationManager {

public:
    CommunicationManager();
    ~CommunicationManager();

    void initialize(const gcsConfig& config);
    void start();
    // Unsubscribes and forgets every vehicle and closes every link; connectAll() starts again.
    void stop();

    void setTelemetryCallback(std::function<void(const std::map<uint8_t, uavStates>&)> cb);
    // Non-numeric status (health, battery, GPS fix, RC, armed, mode,
    // connection), event-driven, for the dashboard.
    void setStatusCallback(std::function<void(const std::map<uint8_t, uavHealth>&)> cb);
    // Each vehicle's EKF origin (GPS_GLOBAL_ORIGIN: lat, lon [deg], altitude
    // AMSL [m]), requested until it arrives, then every 10 s to see a change.
    void setEkfOriginCallback(std::function<void(uint8_t, double, double, double)> cb);

    // Age [s] of a vehicle's last CONTROL_SYSTEM_STATE; nullopt if none yet.
    [[nodiscard]] std::optional<double> telemetryAge(uint8_t sysId);

    // Opens the links; vehicles register whenever they connect (no time
    // limit). discoveryTimeoutMs only bounds how long this waits to print a
    // summary.
    void connectAll(const std::string& baseIp, uint16_t basePort, int numUavs, int increment, int discoveryTimeoutMs = 5000);
    void connectAll(const std::vector<pixhawkEndpointConfig>& endpoints, int discoveryTimeoutMs = 5000);

    // force: ArduPilot force arm, skips the pre-arm checks.
    void armAll(bool force = false);
    void setMode(uint8_t sysId, const std::string& mode);
    void setModeAll(const std::string& mode);
    void fetchParam(int sysId);

    // Opens one link; the vehicle on it registers asynchronously (m_watchSystem).
    bool addLink(const std::string& connection);
    void listLinks();

    void setHomeToCurrentPosition();
    void setUavCommands(const std::map<uint8_t, uavCommandsFlags>& uavCommands);

    void sendRtcmData(const std::vector<uint8_t> &data);

private:
    // Declared before every plugin so the plugins are destroyed first.
    mavsdk::Mavsdk m_mavsdk;
    mavsdk::Mavsdk::NewSystemHandle m_newSystemHandle;

    struct Vehicle {
        std::shared_ptr<mavsdk::System> system;
        std::shared_ptr<mavsdk::Telemetry> telemetry;
        std::shared_ptr<mavsdk::Param> param;
        std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
        std::shared_ptr<mavsdk::Rtk> rtk;
        subscriptionHandles handles;
    };
    std::map<uint8_t, Vehicle> m_vehicles;  // registered vehicles, m_linkMutex

    // is_connected watcher of every system MAVSDK reported, m_linkMutex.
    struct Watcher {
        std::shared_ptr<mavsdk::System> system;
        std::optional<mavsdk::System::IsConnectedHandle> handle;
    };
    std::map<uint8_t, Watcher> m_watchers;

    std::vector<std::thread> m_registrationThreads; // m_linkMutex, joined by stop()

    // sysIds of Pixhawk.endpoints, to flag a SYSID collision. Empty in SITL.
    std::vector<uint8_t> m_expectedSysIds;
    // Every link ever opened, so stop() closes them all.
    std::vector<mavsdk::Mavsdk::ConnectionHandle> m_connectionHandles;

    std::mutex m_linkMutex;
    std::mutex m_statesMutex;

    std::shared_ptr<mavsdk::MavlinkPassthrough> m_passthroughOf(uint8_t sysId);
    std::vector<std::pair<uint8_t, std::shared_ptr<mavsdk::MavlinkPassthrough>>> m_passthroughs();
    std::shared_ptr<StatesAggregator> m_aggregatorOf(uint8_t sysId);

    // Attaches an is_connected watcher once per sysId; m_onConnectionChanged
    // registers the vehicle on its first connection and restores its message
    // rates on a reconnection (vehicle reboot).
    void m_watchSystem(const std::shared_ptr<mavsdk::System>& system);
    void m_onConnectionChanged(const std::shared_ptr<mavsdk::System>& system, bool connected);
    // Creates the plugins and subscribes. Idempotent per sysId.
    void m_registerSystem(const std::shared_ptr<mavsdk::System>& system);
    void m_subscribeMavlink(uint8_t sysId);

    static void m_unsubscribeMavlink(Vehicle& vehicle); // without m_linkMutex (callbacks take it)

    // Waits up to timeoutMs only to print how many vehicles registered.
    void m_waitAndSummarize(int expectedCount, int timeoutMs);

    gcsConfig m_config;
    std::function<void(const std::map<uint8_t, uavStates>&)> m_telemetryCallback;
    std::function<void(const std::map<uint8_t, uavHealth>&)> m_statusCallback;
    std::function<void(uint8_t, double, double, double)> m_ekfOriginCallback;

    std::atomic<bool> m_running{false};
    std::thread m_publishThread;
    std::atomic<bool> m_snapshotDirty{false};
    std::map<uint8_t, std::shared_ptr<StatesAggregator>> m_aggregators; // m_statesMutex
    std::map<uint8_t, uavHealth> m_uavHealths;                          // m_statesMutex
    void m_onTelemetryUpdate();
    // Publishes now, or marks the snapshot dirty for the publish thread.
    void m_telemetryChanged();
    void m_onStatusUpdate();
    // Applies `update` to a vehicle's health under the lock, then publishes.
    void m_updateHealth(uint8_t sysId, const std::function<void(uavHealth&)>& update);

    // HEARTBEAT custom_mode read raw: GrsPlane's mode numbering is not
    // MAVSDK's ArduPilot table (uavHealth::customMode).
    void m_onHeartbeat(uint8_t sysId, const mavlink_message_t& message);
    // CONTROL_SYSTEM_STATE: the controller state (grsMavlinkConventions.h),
    // requested at stateRateHz. Its arrival rate is logged after the first 5 s
    // window and whenever below 70 % of the request.
    void m_onControlState(uint8_t sysId, const mavlink_message_t& message);
    void m_onGpsGlobalOrigin(uint8_t sysId, const mavlink_message_t& message);

    // Requested again on any global position while the state is missing or
    // older than 1 s (startup, vehicle reboot), at most every 2 s.
    void m_requestControlState(uint8_t sysId);
    // Called on every global position: requests GPS_GLOBAL_ORIGIN when due.
    void m_requestGpsGlobalOrigin(uint8_t sysId);
    // Everything else at the low rates the dashboard needs.
    void m_requestStatusRates(uint8_t sysId);
    void m_sendCommandLong(uint8_t sysId, uint16_t command, float param1, float param2);

    std::mutex m_requestMutex; // the maps below
    std::map<uint8_t, bool> m_ekfOriginKnown;
    std::map<uint8_t, std::chrono::steady_clock::time_point> m_ekfOriginRequestedAt;
    std::map<uint8_t, std::chrono::steady_clock::time_point> m_stateRequestedAt;
    struct RateWindow {
        std::chrono::steady_clock::time_point start;
        int count = 0;
        bool reported = false;
    };
    std::map<uint8_t, RateWindow> m_stateRate;

    void m_sendAttitudeTarget(const std::map<uint8_t, uavCommandsFlags>& commands);
};


#endif //COMMUNICATIONMANAGER_H
