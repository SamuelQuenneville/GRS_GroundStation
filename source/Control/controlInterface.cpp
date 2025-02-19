/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlInterface.h"
#include "groundStationApp.h"

ControlInterface::ControlInterface(GroundStationApp &gcs, const double frequency)
    : m_gcs(gcs)
    , m_running(false)
    , m_frequency(frequency)
{

}

ControlInterface::~ControlInterface() {
    stop();
}

void ControlInterface::start() {
    m_running = true;
    m_controllerThread = std::thread(&ControlInterface::controlLoop, this);
}

void ControlInterface::stop() {
    m_running = false;
    if (m_controllerThread.joinable()) {
        m_controllerThread.join();
    }
}

void ControlInterface::controlLoop() {
    std::chrono::steady_clock::time_point nextTick = std::chrono::steady_clock::now();

    while (m_running) {
        nextTick += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / m_frequency));

        // TODO place holder control loop
        const int latestData = m_gcs.getControlInput();

        // Use new data to update mpc and send back the infos required in SET_ATTITUDE_TARGET

        const int controlOutput = latestData * 2;
        m_gcs.updateControlOutput(controlOutput);

        LOG_INFO("Controller -> Processed data:" + std::to_string(controlOutput));

        std::this_thread::sleep_until(nextTick);
    }
}