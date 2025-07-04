/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
*/

#ifndef GUIDED_H
#define GUIDED_H

#include <mutex>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <utility>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "Log/programLogger.h"

static constexpr uint32_t ARDUPILOT_PLANE_XNAV_MODE = 3;

constexpr double PI = 3.14159265358979323846;

template<typename T> constexpr T degToRad(T deg) {
    return static_cast<T>(PI) / static_cast<T>(180.0) * deg;
}

class Guided {

public:
    explicit Guided(std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough);
    ~Guided();

    struct Attitude {
        float rollDegree{};     // Roll angle in degrees, positive is right side down
        float pitchDegree{};    // Pitch angle in degrees, positive is nose up
        float yawDegree{};      // Yaw angle in degrees, positive is move nose to the right
        float thrustValue{};    // range 0 to 1
    };

    bool setGuidedMode();
    void startAttitudeControl(float fallBackFrequencyHz = 10.0f);
    void stopAttitudeControl();
    void setAttitude(const Attitude& attitude);
    void setShouldMove(bool shouldMove);
    void setEndSimulation(bool shouldEndSimulation);

private:
    std::shared_ptr<mavsdk::MavlinkPassthrough> m_mavlinkPassthrough;
    std::thread m_controlThread;
    std::atomic<bool> m_running;
    std::mutex m_attitudeMutex;
    std::atomic<uint32_t> m_currentMode;

    std::chrono::steady_clock::time_point m_lastSentTime = std::chrono::steady_clock::now();

    [[nodiscard]] bool m_isGuidedMode() const;
    bool m_setAttitudeTarget();

    static void m_toQuaternion(const Attitude& attitude, float q[4]);

    Attitude m_attitude{0.0,0.0,0.0,0.0};
    bool m_shouldMove = false;
    bool m_endSimulation = false;
};

#endif //GUIDED_H
