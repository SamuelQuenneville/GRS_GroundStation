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
#include <mavsdk/log_callback.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/rtk/rtk.h>
#include <mavsdk/base64.h>

#include <mavsdk/mavlink/common/mavlink_msg_param_set.h>

#include <map>
#include <mutex>
#include <thread>
#include <ranges>
#include <future>
#include <unordered_set>
#include <vector>

#include "gcsConfig.h"
#include "statesAggregator.h"
#include "Log/programLogger.h"
#include "Definitions/communicationStructures.h"

#include "mavlinkMessageBuilder.h"

#define GROUND_STATION mavsdk::Mavsdk::Configuration(255, MAV_COMP_ID_MISSIONPLANNER, true)

class CommunicationManager {

public:
    CommunicationManager();
    ~CommunicationManager();

    void initialize(const gcsConfig& config);
    void start();
    void stop();

    void setTelemetryCallback(std::function<void(const std::map<uint8_t, uavStates>&)> cb);

    // Fired whenever any UAV's non-numeric status changes (health, battery,
    // GPS fix, RC link, armed state, flight mode, connection). Low rate,
    // event-driven -- this is what the dashboard's Status/Health cards
    // should be built from, kept separate from the tight numeric
    // telemetryCallback used by the control loop.
    void setStatusCallback(std::function<void(const std::map<uint8_t, uavHealth>&)> cb);

    // Vehicle registration itself is fully event-driven (see m_watchSystem) and
    // is not gated by any timeout -- a vehicle that powers on late, or whose
    // MAVSDK handshake takes a moment, still gets registered whenever it
    // actually connects, with no need to re-run "connect". discoveryTimeoutMs
    // only bounds how long connectAll() blocks the caller (e.g. the console
    // thread handling "connect") before printing an initial status summary.
    void connectAll(const std::string& baseIp, uint16_t basePort, int numUavs, int increment, int discoveryTimeoutMs = 5000);
    void connectAll(const std::vector<pixhawkEndpointConfig>& endpoints, int discoveryTimeoutMs = 5000);

    void armAll();
    void setMode(uint8_t sysId, const std::string& mode);
    void setModeAll(const std::string& mode);
    void fetchParam(int sysId);

    // Opens one connection (link) and returns whether the socket-level add
    // succeeded. Whether a vehicle is ever actually discovered on it is
    // handled asynchronously afterward -- see m_watchSystem.
    bool addLink(const std::string& connection);
    void listLinks();

    std::shared_ptr<mavsdk::Telemetry> getTelemetry(uint8_t sysId);
    std::shared_ptr<mavsdk::Action> getAction(uint8_t sysId);

    void setHomeToCurrentPosition();
    void setUavCommands(const std::map<uint8_t, uavCommandsFlags>& uavCommands);

    void sendRtcmData(const std::vector<uint8_t> &data);

private:
    // Fires whenever MAVSDK sees a brand-new system on ANY open connection,
    // at any time -- not just during a bounded "connect" window. This is
    // what lets 2-3 Pixhawks connect in any order, at any pace, without
    // racing MAVSDK's post-heartbeat handshake (component discovery,
    // vehicle-type resolution, ...). See m_watchSystem for the rest of the
    // registration pipeline.
    mavsdk::Mavsdk::NewSystemHandle m_newSystemHandle;

    // sysIds we've already attached an is_connected watcher to, so a system
    // reported again by subscribe_on_new_system doesn't get double-watched.
    std::unordered_set<uint8_t> m_watchedSystems;

    // sysIds declared in the config's Pixhawk.endpoints list, used only to
    // flag an unexpected/duplicate sysId (e.g. two Pixhawks left on the same
    // default SYSID_THISMAV) with a clear warning. Empty in SITL mode, where
    // there's no such list -- the check is skipped in that case.
    std::unordered_set<uint8_t> m_expectedSysIds;

    // Every connection ever opened via addLink(), regardless of whether a
    // vehicle was ever discovered on it -- kept so stop() can always close
    // every socket, not just the ones that got as far as producing a system.
    std::vector<mavsdk::Mavsdk::ConnectionHandle> m_connectionHandles;

    mavsdk::Mavsdk m_mavsdk;
    std::map<uint8_t, std::shared_ptr<mavsdk::System>> m_links;
    std::map<uint8_t, std::shared_ptr<mavsdk::Telemetry>> m_telemetry;
    std::map<uint8_t, std::shared_ptr<mavsdk::Action>> m_action;
    std::map<uint8_t, std::shared_ptr<mavsdk::Param>> m_param;
    std::map<uint8_t, std::shared_ptr<mavsdk::MavlinkPassthrough>> m_passthrough;
    std::map<uint8_t, std::shared_ptr<mavsdk::Rtk>> m_rtk;

    std::map<uint8_t, mavsdk::Vehicle> m_vehicleType;

    std::atomic<uint32_t> m_currentMode;

    gcsConfig m_config;
    std::function<void(const std::map<uint8_t, uavStates>&)> m_telemetryCallback;
    std::function<void(const std::map<uint8_t, uavHealth>&)> m_statusCallback;
    void m_onStatusUpdate();

    std::unordered_map<uint8_t, subscriptionHandles> m_messageHandles;
    void m_subscribeMavlink(uint8_t sysId);
    void m_unsubscribeMavlink(uint8_t sysId);

    // Registration pipeline for a system MAVSDK has told us about.
    // m_watchSystem is safe to call repeatedly with the same system (e.g.
    // once per subscribe_on_new_system firing) -- it no-ops if that sysId is
    // already registered or already being watched. If the system isn't
    // fully ready yet (heartbeat seen, but component discovery / vehicle
    // type still resolving), it attaches a per-system is_connected watcher
    // instead of registering immediately, so registration happens the
    // moment MAVSDK actually marks it connected, however long that takes.
    void m_watchSystem(const std::shared_ptr<mavsdk::System>& system);

    // Actually creates the Telemetry/Action/Param/MavlinkPassthrough/Rtk
    // plugin instances for a newly-ready system and wires it into the rest
    // of the manager. Idempotent: a second call for an already-registered
    // sysId is a no-op.
    void m_registerSystem(const std::shared_ptr<mavsdk::System>& system);

    // Blocks up to timeoutMs purely to give the caller (e.g. the console
    // thread handling "connect") an informative summary of how many of the
    // expectedCount vehicles are registered so far. This is UX only --
    // vehicles that connect after this returns still get registered
    // automatically via m_watchSystem, with no time limit.
    void m_waitAndSummarize(int expectedCount, int timeoutMs);

    std::atomic<bool> m_running{false};
    std::thread m_publishThread;
    std::atomic<bool> m_snapshotDirty{false};
    std::unordered_map<uint8_t, std::shared_ptr<StatesAggregator>> m_aggregators;
    void m_onTelemetryUpdate();

    void m_handleCommandAck(const mavlink_message_t& message);
    void m_subscribeCommandAck(uint8_t sysId);

    void m_handleHeartbeat(const mavlink_message_t& message);
    void m_subscribeToHeartbeat(uint8_t sysId);

    void m_requestAttitudeTarget(uint8_t sysId);
    void m_handleAttitudeTarget(const mavlink_message_t& message) const;
    void m_subscribeAttitudeTarget(uint8_t sysId);

    void m_subscribeHealth(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeHealthAllOk(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeArmed(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeBattery(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeGpsInfo(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeRcStatus(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);

    static void m_subscribeHome(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeFlightMode(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeAttitude(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribePositionVelocity(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribePosition(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeFixedwingMetrics(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);

    void m_sendAttitudeTarget();

    void m_setParameter(uint8_t sysId, MAV_PARAM_TYPE type, std::string name, float value);

    std::map<uint8_t, uavHealth> m_uavHealths;
    std::map<uint8_t, uavCommandsFlags> m_uavCommands;

    std::mutex m_statesMutex;
    std::mutex m_linkMutex;

    std::mutex m_commandAckMutex;
    std::condition_variable m_cvCommandAck;
    mavlink_command_ack_t m_lastAck{};
};


#endif //COMMUNICATIONMANAGER_H
