/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Required by PX4-GPSDrivers: the library expects the consuming project to
 * supply "definitions.h" with sensor_gps_s, satellite_info_s and
 * sensor_gnss_relative_s, plus GPS_INFO/GPS_WARN/GPS_ERR logging macros and
 * gps_absolute_time(). These are normally provided by PX4-Autopilot or
 * QGroundControl; this is the minimal equivalent for this project.
 *
 * Struct layouts copied from:
 * https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/gps/definitions.h
 * (also used as-is by https://github.com/julianoes/rtk-sender-example)
 *
 * Included via the GPS_DEFINITIONS_HEADER compile definition set in
 * CMakeLists.txt, not included directly elsewhere in this project.
 */

#ifndef GPSDEFINITIONS_H
#define GPSDEFINITIONS_H

#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <sys/time.h>

// PX4-GPSDrivers logs via these printf-style macros. Kept as printf rather
// than routed through Log/programLogger.h's LOG_INFO/LOG_WARNING/LOG_ERROR,
// since those take a single std::string argument and don't support the
// variadic format-string calls PX4-GPSDrivers makes internally.
#define GPS_INFO(...) printf(__VA_ARGS__)
#define GPS_WARN(...) printf(__VA_ARGS__)
#define GPS_ERR(...)  printf(__VA_ARGS__)

// To make PX4-GPSDrivers happy
#define M_DEG_TO_RAD 		(M_PI / 180.0)
#define M_RAD_TO_DEG 		(180.0 / M_PI)
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f

using gps_abstime = uint64_t;

static inline gps_abstime gps_absolute_time() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<gps_abstime>(tv.tv_sec) * 1'000'000ULL + static_cast<gps_abstime>(tv.tv_usec);
}

struct sensor_gps_s {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    static constexpr uint8_t FIX_TYPE_NONE                   = 1;
    static constexpr uint8_t FIX_TYPE_2D                     = 2;
    static constexpr uint8_t FIX_TYPE_3D                     = 3;
    static constexpr uint8_t FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4;
    static constexpr uint8_t FIX_TYPE_RTK_FLOAT              = 5;
    static constexpr uint8_t FIX_TYPE_RTK_FIXED              = 6;
    static constexpr uint8_t FIX_TYPE_EXTRAPOLATED           = 8;

    static constexpr uint8_t JAMMING_STATE_UNKNOWN   = 0;
    static constexpr uint8_t JAMMING_STATE_OK        = 1;
    static constexpr uint8_t JAMMING_STATE_MITIGATED = 2;
    static constexpr uint8_t JAMMING_STATE_DETECTED  = 3;

    static constexpr uint8_t SPOOFING_STATE_UNKNOWN   = 0;
    static constexpr uint8_t SPOOFING_STATE_OK        = 1;
    static constexpr uint8_t SPOOFING_STATE_MITIGATED = 2;
    static constexpr uint8_t SPOOFING_STATE_DETECTED  = 3;

    static constexpr uint8_t AUTHENTICATION_STATE_UNKNOWN      = 0;
    static constexpr uint8_t AUTHENTICATION_STATE_INITIALIZING = 1;
    static constexpr uint8_t AUTHENTICATION_STATE_ERROR        = 2;
    static constexpr uint8_t AUTHENTICATION_STATE_OK           = 3;
    static constexpr uint8_t AUTHENTICATION_STATE_DISABLED     = 4;

    static constexpr uint32_t SYSTEM_ERROR_OK                   = 0;
    static constexpr uint32_t SYSTEM_ERROR_INCOMING_CORRECTIONS = 1;
    static constexpr uint32_t SYSTEM_ERROR_CONFIGURATION        = 2;
    static constexpr uint32_t SYSTEM_ERROR_SOFTWARE             = 4;
    static constexpr uint32_t SYSTEM_ERROR_ANTENNA              = 8;
    static constexpr uint32_t SYSTEM_ERROR_EVENT_CONGESTION     = 16;
    static constexpr uint32_t SYSTEM_ERROR_CPU_OVERLOAD         = 32;
    static constexpr uint32_t SYSTEM_ERROR_OUTPUT_CONGESTION    = 64;

    static constexpr uint8_t CORRECTIONS_PROTOCOL_UNKNOWN = 0;
    static constexpr uint8_t CORRECTIONS_PROTOCOL_RTCM3  = 1;
    static constexpr uint8_t CORRECTIONS_PROTOCOL_SPARTN = 2;
    static constexpr uint8_t CORRECTIONS_PROTOCOL_HAS    = 3;
    static constexpr uint8_t CORRECTIONS_PROTOCOL_PMP    = 4;
    static constexpr uint8_t CORRECTIONS_PROTOCOL_QZSS_L6 = 5;

    static constexpr uint8_t CORRECTIONS_MSG_USED_UNKNOWN  = 0;
    static constexpr uint8_t CORRECTIONS_MSG_USED_NOT_USED = 1;
    static constexpr uint8_t CORRECTIONS_MSG_USED_USED     = 2;

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    uint64_t timestamp{0};
    uint64_t timestamp_sample{0};
    uint32_t device_id{0};

    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_msl_m{0.0};
    double altitude_ellipsoid_m{0.0};

    float s_variance_m_s{0.0f};
    float c_variance_rad{0.0f};
    uint8_t fix_type{0};
    float eph{0.0f};
    float epv{0.0f};
    float hdop{0.0f};
    float vdop{0.0f};

    int32_t noise_per_ms{0};
    uint16_t automatic_gain_control{0};
    uint8_t jamming_state{JAMMING_STATE_UNKNOWN};
    int32_t jamming_indicator{0};
    uint8_t spoofing_state{SPOOFING_STATE_UNKNOWN};
    uint8_t authentication_state{AUTHENTICATION_STATE_UNKNOWN};

    float vel_m_s{0.0f};
    float vel_n_m_s{0.0f};
    float vel_e_m_s{0.0f};
    float vel_d_m_s{0.0f};
    float cog_rad{0.0f};
    bool vel_ned_valid{false};

    int32_t timestamp_time_relative{0};
    uint64_t time_utc_usec{0};

    uint8_t satellites_used{0};

    uint32_t system_error{SYSTEM_ERROR_OK};

    float heading{0.0f};
    float heading_offset{0.0f};
    float heading_accuracy{0.0f};

    float rtcm_injection_rate{0.0f};
    uint8_t selected_rtcm_instance{0};
    uint8_t corrections_protocol{CORRECTIONS_PROTOCOL_UNKNOWN};
    bool corrections_crc_failed{false};
    uint8_t corrections_msg_used{CORRECTIONS_MSG_USED_UNKNOWN};

    float antenna_offset_x{0.0f};
    float antenna_offset_y{0.0f};
    float antenna_offset_z{0.0f};
};

struct satellite_info_s {
    uint64_t timestamp;
    uint8_t count;
    uint8_t svid[20];
    uint8_t used[20];
    uint8_t elevation[20];
    uint8_t azimuth[20];
    uint8_t snr[20];
    uint8_t prn[20];
    uint8_t _padding0[7]; // required for logger

    static constexpr uint8_t SAT_INFO_MAX_SATELLITES = 20;
};

struct sensor_gnss_relative_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint64_t time_utc_usec;
    uint32_t device_id;
    float position[3];
    float position_accuracy[3];
    float heading;
    float heading_accuracy;
    float position_length;
    float accuracy_length;
    uint16_t reference_station_id;
    bool gnss_fix_ok;
    bool differential_solution;
    bool relative_position_valid;
    bool carrier_solution_floating;
    bool carrier_solution_fixed;
    bool moving_base_mode;
    bool reference_position_miss;
    bool reference_observations_miss;
    bool heading_valid;
    bool relative_position_normalized;
};

#endif //GPSDEFINITIONS_H
