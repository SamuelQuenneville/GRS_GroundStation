/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rtkBaseStation.h"


// --------------------- RtkSerialPort ---------------------

bool RtkSerialPort::init(const std::string& device) {
    m_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (m_fd < 0) {
        LOG_ERROR("RtkBaseStation: failed to open " + device + ": " + std::strerror(errno));
        return false;
    }

    termios tty{};
    if (tcgetattr(m_fd, &tty) != 0) {
        LOG_ERROR("RtkBaseStation: tcgetattr failed");
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1s read timeout, in deciseconds

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        LOG_ERROR("RtkBaseStation: tcsetattr failed");
        return false;
    }

    return true;
}

bool RtkSerialPort::setBaudrate(const unsigned baudrate) const {
    if (m_fd < 0) return false;

    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            LOG_ERROR("RtkBaseStation: unsupported baudrate " + std::to_string(baudrate));
            return false;
    }

    termios tty{};
    if (tcgetattr(m_fd, &tty) != 0) return false;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    return tcsetattr(m_fd, TCSANOW, &tty) == 0;
}

ssize_t RtkSerialPort::read(uint8_t* bytes, const unsigned len) const {
    if (m_fd < 0) return -1;
    return ::read(m_fd, bytes, len);
}

ssize_t RtkSerialPort::write(const uint8_t* bytes, const unsigned len) const {
    if (m_fd < 0) return -1;
    return ::write(m_fd, bytes, len);
}

RtkSerialPort::~RtkSerialPort() {
    if (m_fd >= 0) ::close(m_fd);
}

// --------------------- RtkBaseStation ---------------------

RtkBaseStation::~RtkBaseStation() {
    stop();
}

bool RtkBaseStation::start(const std::string& device, unsigned baudrate, const RtcmCallback& callback, const float surveyInMinimumMeters, const unsigned surveyInDurationSeconds) {
    if (m_running.load()) {
        LOG_WARNING("RtkBaseStation already running");
        return false;
    }

    if (!m_serialPort.init(device)) {
        return false;
    }

    m_rtcmCallback = callback;

    GPSDriverUBX::Settings settings{};
    settings.dynamic_model = 0;             // driver forces stationary (2) internally for base/RTCM output
    settings.dgnss_timeout = 0;             // 0 = don't touch (u-blox default)
    settings.min_cno = 0;                   // 0 = don't touch (u-blox default)
    settings.min_elev = 0;                  // 0 = don't touch (u-blox default)
    settings.output_rate = 0;               // 0 = don't touch (u-blox default); driver caps at 25Hz anyway
    settings.heading_offset = 0.0F;         // unused for a single, non-heading receiver
    settings.uart2_baudrate = 0;            // not using UART2 (moving-base/heading setup)
    settings.ppk_output = false;
    settings.jam_det_sensitivity_hi = false;
    settings.mode = GPSDriverUBX::UBXMode::Normal; // standalone base station, not a moving-base/rover pair

    m_driver = std::make_unique<GPSDriverUBX>(
        GPSDriverUBX::Interface::UART,
        &RtkBaseStation::m_callbackEntry, this,
        &m_gpsPos, &m_satInfo, settings);

    // Distance the F9P must be confident of its own position within before
    // it starts broadcasting corrections, and how long it surveys for.
    m_driver->setSurveyInSpecs(static_cast<int>(surveyInMinimumMeters * 10000),static_cast<int>(surveyInDurationSeconds)); // driver expects 0.1mm units

    GPSHelper::GPSConfig gpsConfig{};
    gpsConfig.output_mode = GPSHelper::OutputMode::RTCM;
    gpsConfig.gnss_systems = GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS;

    if (m_driver->configure(baudrate, gpsConfig) != 0) {
        LOG_ERROR("RtkBaseStation: F9P configuration failed");
        return false;
    }

    LOG_INFO("RtkBaseStation: F9P configured, starting survey-in and RTCM streaming");

    m_running.store(true);
    m_receiveThread = std::thread([this]() {
        while (m_running.load()) {
            constexpr unsigned timeoutMs = 1000;
            m_driver->receive(timeoutMs);
            // receive() can return on timeout even with a healthy link;
            // just loop and keep going until stop() is called.
        }
    });

    return true;
}

void RtkBaseStation::stop() {
    if (!m_running.load()) return;

    m_running.store(false);
    if (m_receiveThread.joinable()) {
        m_receiveThread.join();
    }
    m_driver.reset();
    LOG_INFO("RtkBaseStation stopped");
}

// Reads /sys/.../idVendor at the given directory, if present. Returns false
// if the file doesn't exist there (caller should try a parent directory).
bool readIdVendorAt(const std::filesystem::path& dir, std::string& outVendorId) {
    std::ifstream f(dir / "idVendor");
    if (!f) return false;

    std::getline(f, outVendorId);
    while (!outVendorId.empty() && std::isspace(static_cast<unsigned char>(outVendorId.back()))) {
        outVendorId.pop_back();
    }
    return !outVendorId.empty();
}

std::vector<std::string> RtkBaseStation::scanAvailablePorts(const std::string& vendorIdHex) {
    std::vector<std::string> matches;

    const std::filesystem::path ttyClassDir = "/sys/class/tty";
    if (!std::filesystem::exists(ttyClassDir)) {
        LOG_WARNING("RtkBaseStation: " + ttyClassDir.string() + " not found (not on Linux?)");
        return matches;
    }

    std::error_code dirEc;
    for (const auto& entry : std::filesystem::directory_iterator(ttyClassDir, dirEc)) {
        const std::string name = entry.path().filename().string();

        // Only consider USB-attached serial devices; native/PCI UARTs
        // (ttyS*) don't have a USB vendor ID to check.
        if (name.rfind("ttyACM", 0) != 0 && name.rfind("ttyUSB", 0) != 0) {
            continue;
        }

        std::error_code ec;
        const std::filesystem::path devicePath = std::filesystem::canonical(entry.path() / "device", ec);
        if (ec) continue;

        // ttyACM*/ttyUSB* "device" symlinks point at the USB *interface*
        // directory, not the USB device itself, and the exact depth to walk
        // up to reach idVendor varies (ttyACM vs. ttyUSB, hubs in between,
        // etc.), so just climb until we find it or give up.
        std::string vendorId;
        std::filesystem::path search = devicePath;
        for (int depth = 0; depth < 6 && search.has_parent_path(); ++depth) {
            if (readIdVendorAt(search, vendorId)) break;
            search = search.parent_path();
        }

        if (!vendorId.empty() && vendorId == vendorIdHex) {
            matches.push_back("/dev/" + name);
        }
    }

    std::ranges::sort(matches);
    return matches;
}

std::string RtkBaseStation::findFirstMatchingPort(const std::string& vendorIdHex) {
    const auto ports = scanAvailablePorts(vendorIdHex);
    return ports.empty() ? std::string{} : ports.front();
}

int RtkBaseStation::m_callbackEntry(const GPSCallbackType type, void* data1, const int data2, void* user) {
    return static_cast<RtkBaseStation*>(user)->m_callback(type, data1, data2);
}

int RtkBaseStation::m_callback(const GPSCallbackType type, void* data1, const int data2) const {
    switch (type) {
        case GPSCallbackType::readDeviceData:
            return static_cast<int>(m_serialPort.read(static_cast<uint8_t*>(data1), data2));

        case GPSCallbackType::writeDeviceData:
            return static_cast<int>(m_serialPort.write(static_cast<const uint8_t*>(data1), data2));

        case GPSCallbackType::setBaudrate:
            return m_serialPort.setBaudrate(data2) ? 0 : 1;

        case GPSCallbackType::gotRTCMMessage: {
            const auto* bytes = static_cast<const uint8_t*>(data1);
            if (m_rtcmCallback) {
                m_rtcmCallback(std::vector<uint8_t>(bytes, bytes + data2));
            }
            return 0;
        }

        case GPSCallbackType::surveyInStatus: {
            const auto* status = static_cast<SurveyInStatus*>(data1);
            LOG_INFO("RTK base survey-in: flags=" + std::to_string(status->flags)
                + " accuracy=" + std::to_string(1e-3 * static_cast<double>(status->mean_accuracy)) + "m"
                + " duration=" + std::to_string(status->duration) + "s");
            return 0;
        }

        default:
            return 0;
    }
}