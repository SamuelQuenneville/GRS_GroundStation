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
    void connectAll(const std::string& baseIp, uint16_t basePort, int numUavs, int increment);
    void connectAll(const std::vector<pixhawkEndpointConfig>& endpoints);

    void armAll();
    void setMode(uint8_t sysId, const std::string& mode);
    void setModeAll(const std::string& mode);
    void fetchParam(int sysId);

    bool addLink(const std::string& connection);
    void listLinks();

    std::shared_ptr<mavsdk::Telemetry> getTelemetry(uint8_t sysId);
    std::shared_ptr<mavsdk::Action> getAction(uint8_t sysId);

    void setHomeToCurrentPosition();
    void setUavCommands(const std::map<uint8_t, uavCommandsFlags>& uavCommands);

    void sendRtcmData(const std::vector<uint8_t> &data);

private:
    int m_numberOfUavs = 0;

    mavsdk::Mavsdk m_mavsdk;
    std::map<uint8_t, std::shared_ptr<mavsdk::System>> m_links;
    std::map<uint8_t, mavsdk::Handle<>> m_connectionHandles;
    std::map<uint8_t, std::shared_ptr<mavsdk::Telemetry>> m_telemetry;
    std::map<uint8_t, std::shared_ptr<mavsdk::Action>> m_action;
    std::map<uint8_t, std::shared_ptr<mavsdk::Param>> m_param;
    std::map<uint8_t, std::shared_ptr<mavsdk::MavlinkPassthrough>> m_passthrough;
    std::map<uint8_t, std::shared_ptr<mavsdk::Rtk>> m_rtk;

    std::map<uint8_t, mavsdk::Vehicle> m_vehicleType;

    std::atomic<uint32_t> m_currentMode;

    gcsConfig m_config;
    std::function<void(const std::map<uint8_t, uavStates>&)> m_telemetryCallback;

    std::unordered_map<uint8_t, subscriptionHandles> m_messageHandles;
    void m_subscribeMavlink(uint8_t sysId);
    void m_unsubscribeMavlink(uint8_t sysId);

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
