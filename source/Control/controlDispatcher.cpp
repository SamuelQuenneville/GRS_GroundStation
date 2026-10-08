/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlDispatcher.h"

#include <utility>

ControlDispatcher::ControlDispatcher() = default;

ControlDispatcher::~ControlDispatcher() {
    stop();
}

void ControlDispatcher::start() {
    m_running = true;
    m_thread = std::thread(&ControlDispatcher::m_dispatchLoop, this);
}

void ControlDispatcher::stop() {
    {
        std::lock_guard lock(m_queueMutex); // no lost wake-up between the predicate and the wait
        m_running = false;
    }
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

void ControlDispatcher::pushCommand(const std::map<uint8_t, uavCommandsFlags>& cmds) {
    {
        // Latest command only: one still waiting is outdated, sending it would
        // only delay this one.
        std::lock_guard lock(m_queueMutex);
        if (m_pending && m_droppedCommands++ % 20 == 0) {
            LOG_WARNING("Command link slower than the control loop: " + std::to_string(m_droppedCommands) + " outdated commands dropped");
        }
        m_pending = cmds;
    }
    m_cv.notify_one();
}

void ControlDispatcher::updateTelemetry(const std::map<uint8_t, uavStates>& states) const {
    if (m_sendToController) {
        m_sendToController(states);
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
        m_cv.wait(lock, [this]() { return m_pending.has_value() || !m_running; });

        if (!m_running) break;

        const auto cmds = std::exchange(m_pending, std::nullopt).value();
        lock.unlock();

        if (m_sendToComms) {
            m_sendToComms(cmds);
        }
    }
}