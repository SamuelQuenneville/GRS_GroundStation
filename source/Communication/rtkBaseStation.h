/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef RTKBASESTATION_H
#define RTKBASESTATION_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>

#include "gps_helper.h"
#include "ubx.h"

#include "Log/programLogger.h"

// Minimal serial port wrapper for talking to the F9P. Kept in this class
// rather than a separate file since it's small and only used here.
class RtkSerialPort {
public:
    bool init(const std::string& device);
    bool setBaudrate(unsigned baudrate) const;
    ssize_t read(uint8_t* bytes, unsigned len) const;
    ssize_t write(const uint8_t* bytes, unsigned len) const;
    ~RtkSerialPort();

private:
    int m_fd = -1;
};

class RtkBaseStation {
public:
    // Called with a chunk of raw RTCM3 bytes every time the F9P produces one.
    using RtcmCallback = std::function<void(const std::vector<uint8_t>&)>;

    ~RtkBaseStation();

    // device: e.g. "/dev/ttyACM0". baudrate: 0 to auto-detect.
    // surveyInMinimumMeters / surveyInDurationSeconds control the F9P's
    // self-survey before it starts producing corrections.
    bool start(
        const std::string& device,
        unsigned baudrate,
        const RtcmCallback& callback,
        float surveyInMinimumMeters = 10.0F,
        unsigned surveyInDurationSeconds = 60);

    void stop();

    [[nodiscard]] bool isRunning() const { return m_running.load(); }

    // Scans /sys/class/tty for USB serial devices (ttyACM*/ttyUSB*) whose
    // USB vendor ID matches vendorIdHex (default: "1546", u-blox AG's
    // vendor ID). Returns device paths, e.g. "/dev/ttyACM0",
    // sorted for stable ordering. Empty if none found or not on Linux.
    static std::vector<std::string> scanAvailablePorts(const std::string& vendorIdHex = "1546");

    // Convenience wrapper: returns the first matching port, or an empty
    // string if none was found (e.g. F9P not plugged in yet).
    static std::string findFirstMatchingPort(const std::string& vendorIdHex = "1546");

private:
    static int m_callbackEntry(GPSCallbackType type, void* data1, int data2, void* user);
    int m_callback(GPSCallbackType type, void* data1, int data2) const;

    RtcmCallback m_rtcmCallback;
    RtkSerialPort m_serialPort;
    std::unique_ptr<GPSDriverUBX> m_driver;

    struct sensor_gps_s m_gpsPos {};
    struct satellite_info_s m_satInfo {};

    std::atomic<bool> m_running{false};
    std::thread m_receiveThread;
};

#endif //RTKBASESTATION_H
