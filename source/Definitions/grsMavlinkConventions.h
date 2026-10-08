/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRSMAVLINKCONVENTIONS_H
#define GRSMAVLINKCONVENTIONS_H

// MAVLink fields the GRS ground station and the GRS ArduPilot fork use
// differently from common.xml. Keep this file identical on both sides.
//
// CONTROL_SYSTEM_STATE (146), vehicle -> GCS, requested with
// SET_MESSAGE_INTERVAL (the fork maps it to MSG_CONTROL_SYSTEM_STATE and is
// in no SRx_* stream group):
//   time_usec            AP_HAL::micros64() at packing [us since boot]
//   x/y/z_pos            NED position from the EKF origin [m]
//                        (ahrs.get_relative_position_NED_origin, as LOCAL_POSITION_NED)
//   x/y/z_vel            NED velocity [m/s] (ahrs.get_velocity_NED), not body frame
//   airspeed             ahrs.airspeed_estimate() (EAS) [m/s], -1 if unknown
//   q                    attitude, body to NED, [w x y z] (ahrs.get_quaternion)
//   roll/pitch/yaw_rate  body rates [rad/s] (ahrs.get_gyro)
//   x/y/z_acc            unused (0)
//   vel/pos_variance     unused (-1)
// Not sent until the EKF has a position and velocity solution.
//
// SET_ATTITUDE_TARGET (82), GCS -> vehicle, type_mask 0:
//   q                    roll, pitch command, yaw 0
//   thrust               throttle [0, 1] (thrust2rpm)
//   body_roll_rate       angle-of-attack feedforward [deg], 0 if none
//   body_pitch_rate      tether tension [N], 0 if none
//   body_yaw_rate        flags as a float (commandFlag, vehicleStructures.h):
//                        bit 0 should_move, bit 1 end_sim, bit 2 launch

#endif //GRSMAVLINKCONVENTIONS_H
