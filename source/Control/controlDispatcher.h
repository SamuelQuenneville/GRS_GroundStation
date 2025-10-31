/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CONTROLDISPATCHER_H
#define CONTROLDISPATCHER_H

#pragma once

#include <mutex>
#include <condition_variable>
#include <map>
#include <queue>
#include <thread>
#include <atomic>
#include <functional>

#include "Definitions/communicationStructures.h"

class ControlDispatcher {
public:
    ControlDispatcher();
    ~ControlDispatcher();

    void start();
    void stop();

    // From ControlInterface (controller → dispatcher)
    void pushCommand(const std::map<uint8_t, uavCommandsFlags>& cmds);

    // From CommunicationManager (telemetry → controller)
    void updateTelemetry(const std::map<uint8_t, uavStates>& states);

    // Setters for integration
    void attachCommunicationManager(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> sendFn);
    void attachControllerInput(std::function<void(const std::map<uint8_t, uavStates>&)> recvFn);

private:
    void m_dispatchLoop();

    std::thread m_thread;
    std::atomic<bool> m_running{false};

    std::mutex m_queueMutex;
    std::condition_variable m_cv;
    std::queue<std::map<uint8_t, uavCommandsFlags>> m_commandQueue;

    std::mutex m_stateMutex;
    std::map<uint8_t, uavStates> m_latestStates;

    std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> m_sendToComms;
    std::function<void(const std::map<uint8_t, uavStates>&)> m_sendToController;
};

#endif //CONTROLDISPATCHER_H
