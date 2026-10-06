/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "mavlinkMessageBuilder.h"

mavlink_message_t MavlinkMessageBuilder::buildSetAttitudeTarget(const MavlinkAddress& address, const uint8_t channel, const uint8_t targetSysid, const uint8_t targetCompid, const uavCommandsFlags& target) {
    mavlink_message_t msg{};
    float q[4];
    m_eulerToQuaternion(target.commands.rollDegree, target.commands.pitchDegree, target.commands.yawDegree, q);

    constexpr float thrustBody[3] = {0.0f, 0.0f, 0.0f};

    mavlink_msg_set_attitude_target_pack_chan(
        address.system_id,
        address.component_id,
        channel,
        &msg,
        static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()),
        targetSysid,
        targetCompid,
        0,                              // type_mask: the body-rate fields carry data
        q,
        target.estimates.aoaDegree,     // body_roll_rate field
        target.estimates.tension,       // body_pitch_rate field
        static_cast<float>(target.flags), // body_yaw_rate field, commandFlag bits
        target.commands.thrust,
        thrustBody
    );
    return msg;

}

void MavlinkMessageBuilder::m_eulerToQuaternion(const double rollDeg, const double pitchDeg, const double yawDeg, float q[4]) {
    constexpr double degToRad_2 = M_PI_2 / 180.0;

    const double rollRad_2   = rollDeg * degToRad_2;
    const double pitchRad_2  = pitchDeg * degToRad_2;
    const double yawRad_2    = yawDeg * degToRad_2;

    const double cos_phi_2   = std::cos(rollRad_2);
    const double sin_phi_2   = std::sin(rollRad_2);
    const double cos_theta_2 = std::cos(pitchRad_2);
    const double sin_theta_2 = std::sin(pitchRad_2);
    const double cos_psi_2   = std::cos(yawRad_2);
    const double sin_psi_2   = std::sin(yawRad_2);

    q[0] = static_cast<float>(cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2);
    q[1] = static_cast<float>(sin_phi_2 * cos_theta_2 * cos_psi_2 - cos_phi_2 * sin_theta_2 * sin_psi_2);
    q[2] = static_cast<float>(cos_phi_2 * sin_theta_2 * cos_psi_2 + sin_phi_2 * cos_theta_2 * sin_psi_2);
    q[3] = static_cast<float>(cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * sin_theta_2 * cos_psi_2);
}