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

#include <map>
#include <mutex>
#include <thread>
#include <ranges>

#include "Log/programLogger.h"
#include "Definitions/communicationStructures.h"

#define GROUND_STATION mavsdk::Mavsdk::Configuration(mavsdk::ComponentType::GroundStation)

class CommunicationManager {

public:
    CommunicationManager();
    ~CommunicationManager();

    bool addLink(const std::string& connection);
    void listLinks();

    std::shared_ptr<mavsdk::Telemetry> getTelemetry(uint8_t sysId);
    std::shared_ptr<mavsdk::Action> getAction(uint8_t sysId);

private:
    mavsdk::Mavsdk m_mavsdk;
    std::map<uint8_t, std::shared_ptr<mavsdk::System>> m_links;
    std::map<uint8_t, mavsdk::Handle<>> m_connectionHandles;
    std::map<uint8_t, std::shared_ptr<mavsdk::Telemetry>> m_telemetry;
    std::map<uint8_t, std::shared_ptr<mavsdk::Action>> m_action;

    int m_numberOfUavs = 0;

    std::unordered_map<uint8_t, subscriptionHandles> m_messageHandles;
    void m_subscribeMavlink(uint8_t sysId);
    void m_unsubscribeMavlink(uint8_t sysId);

    void m_subscribeHealth(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeHealthAllOk(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeArmed(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeFlightMode(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeAttitude(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeAngularVelocity(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribePositionVelocity(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribePosition(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeVelocityNed(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeHeading(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);
    void m_subscribeFixedwingMetrics(const std::shared_ptr<mavsdk::Telemetry>& telemetry, uint8_t sysId, subscriptionHandles& handles);

    std::map<uint8_t, attitude> m_uavStates;

    std::mutex m_linkMutex;
};


#endif //COMMUNICATIONMANAGER_H
