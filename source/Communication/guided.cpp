/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
*/

#include "guided.h"

Guided::Guided(std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough)
    : m_mavlinkPassthrough(std::move(passthrough))
    , m_running(false)
{

}

Guided::~Guided() {
    stopAttitudeControl();
}

bool Guided::setGuidedMode() {
    mavsdk::MavlinkPassthrough::CommandLong command{};
    command.command = MAV_CMD_DO_SET_MODE;
    command.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    command.param2 = ARDUPILOT_PLANE_XNAV_MODE;
    command.target_sysid = m_mavlinkPassthrough->get_target_sysid();
    command.target_compid = MAV_COMP_ID_AUTOPILOT1;

    const auto result = m_mavlinkPassthrough->send_command_long(command);

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << result << std::endl;
        return false;
    }

    return true;
}

void Guided::startAttitudeControl(float fallBackFrequencyHz) {
    if (m_running) return;  // Already running

    if (!m_isGuidedMode()) {
        LOG_WARNING("Not in guided mode");
        return;
    }

    m_running = true;
    m_controlThread = std::thread([this, fallBackFrequencyHz]() {
        const auto interval = std::chrono::milliseconds(static_cast<int>(1000 / fallBackFrequencyHz));

        while (m_running) {
            auto now = std::chrono::steady_clock::now();

            // Send periodic attitude updates if needed
            std::lock_guard<std::mutex> lock(m_attitudeMutex);
            if (now - m_lastSentTime >= interval) {
                m_setAttitudeTarget();
                m_lastSentTime = now;
            }

            std::this_thread::sleep_for(interval);
        }
    });
}

void Guided::stopAttitudeControl() {
    m_running = false;
    if (m_controlThread.joinable()) {
        m_controlThread.join();
    }
}

void Guided::setAttitude(const Attitude& attitude) {
    std::lock_guard<std::mutex> lock(m_attitudeMutex);
    m_attitude = attitude;

    // Send immediately if called more frequently
    m_setAttitudeTarget();
    m_lastSentTime = std::chrono::steady_clock::now();
}

void Guided::setRc(const RcRaw& rc) {
    std::lock_guard<std::mutex> lock(m_attitudeMutex);
    m_rcRaw = rc;

    // Send immediately if called more frequently
    m_setRcOverride();
    m_lastSentTime = std::chrono::steady_clock::now();
}

void Guided::setShouldMove(const bool shouldMove) {
    std::lock_guard<std::mutex> lock(m_attitudeMutex);
    m_shouldMove = shouldMove;
}

void Guided::setEndSimulation(const bool endSimulation) {
    std::lock_guard<std::mutex> lock(m_attitudeMutex);
    m_endSimulation = endSimulation;
}

bool Guided::m_isGuidedMode() const {
    return m_currentMode == ARDUPILOT_PLANE_XNAV_MODE;
}

bool Guided::m_setAttitudeTarget() {

    const auto attitude = m_attitude;

    const float thrust = attitude.thrustValue;
    float quaternion[4];

    m_toQuaternion(attitude, quaternion);

    constexpr float thrustBody[3] = {0.0f, 0.0f, 0.0f};

    auto const result = m_mavlinkPassthrough->queue_message([&](const MavlinkAddress mavlink_address, const uint8_t channel) {

        mavlink_message_t message;
        mavlink_msg_set_attitude_target_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()),
            m_mavlinkPassthrough->get_target_sysid(),
            m_mavlinkPassthrough->get_target_compid(),
            ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
            quaternion,
            m_shouldMove,
            m_endSimulation,
            0,
            thrust,
            thrustBody
        );
        return message;
    });

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << result << std::endl;
        return false;
    }

    return true;
}

bool Guided::m_setRcOverride() {

    const uint16_t chan[8] = {
        m_rcRaw.aileron,
        m_rcRaw.elevator,
        m_rcRaw.throttle,
        m_rcRaw.rudder,
        UINT16_MAX,
        UINT16_MAX,
        m_shouldMove,
        m_endSimulation
    };

    auto const result = m_mavlinkPassthrough->queue_message([&](const MavlinkAddress mavlink_address, const uint8_t channel) {

        mavlink_message_t message;
        mavlink_msg_rc_channels_override_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            m_mavlinkPassthrough->get_target_sysid(),
            m_mavlinkPassthrough->get_target_compid(),
            chan[0],
            chan[1],
            chan[2],
            chan[3],
            chan[4],
            chan[5],
            chan[6],
            chan[7],
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX,
            UINT16_MAX
        );
        return message;
    });

    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << result << std::endl;
        return false;
    }

    return true;
}

void Guided::m_toQuaternion(const Attitude& attitude, float q[4]) {

    const float roll   = degToRad(attitude.rollDegree);
    const float pitch  = degToRad(attitude.pitchDegree);
    const float yaw    = degToRad(attitude.yawDegree);

    const double cos_phi_2   = cos(static_cast<double>(roll) * 0.5);
    const double sin_phi_2   = sin(static_cast<double>(roll) * 0.5);
    const double cos_theta_2 = cos(static_cast<double>(pitch) * 0.5);
    const double sin_theta_2 = sin(static_cast<double>(pitch) * 0.5);
    const double cos_psi_2   = cos(static_cast<double>(yaw) * 0.5);
    const double sin_psi_2   = sin(static_cast<double>(yaw) * 0.5);

    q[0] = static_cast<float>(cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2);
    q[1] = static_cast<float>(sin_phi_2 * cos_theta_2 * cos_psi_2 - cos_phi_2 * sin_theta_2 * sin_psi_2);
    q[2] = static_cast<float>(cos_phi_2 * sin_theta_2 * cos_psi_2 + sin_phi_2 * cos_theta_2 * sin_psi_2);
    q[3] = static_cast<float>(cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * sin_theta_2 * cos_psi_2);
}
