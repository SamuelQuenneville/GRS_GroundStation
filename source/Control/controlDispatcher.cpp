/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlDispatcher.h"

ControlDispatcher::ControlDispatcher() = default;

ControlDispatcher::~ControlDispatcher() {
    stop();
}

void ControlDispatcher::start() {
    m_running = true;
    m_thread = std::thread(&ControlDispatcher::m_dispatchLoop, this);
}

void ControlDispatcher::stop() {
    m_running = false;
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

void ControlDispatcher::pushCommand(const std::map<uint8_t, uavCommandsFlags>& cmds) {
    {
        std::lock_guard lock(m_queueMutex);
        m_commandQueue.push(cmds);
    }
    m_cv.notify_one();
}

void ControlDispatcher::updateTelemetry(const std::map<uint8_t, uavStates>& states) {
    {
        std::lock_guard lock(m_stateMutex);
        m_latestStates = states;
    }

    if (m_sendToController) {
        m_sendToController(m_latestStates);
    }
}

void ControlDispatcher::attachCommunicationManager(std::function<void(const std::map<uint8_t, uavCommandsFlags>&)> sendFn) {
    m_sendToComms = std::move(sendFn);
}

void ControlDispatcher::attachControllerInput(std::function<void(const std::map<uint8_t, uavStates>&)> recvFn) {
    m_sendToController = std::move(recvFn);
}

void ControlDispatcher::m_dispatchLoop() {

    LOG_INFO("ControlDispatcher started");

    while (m_running) {
        std::unique_lock lock(m_queueMutex);
        m_cv.wait(lock, [this]() { return !m_commandQueue.empty() || !m_running; });

        if (!m_running) break;

        auto cmds = m_commandQueue.front();
        m_commandQueue.pop();
        lock.unlock();

        if (m_sendToComms) {
            m_sendToComms(cmds);
        }
    }
}