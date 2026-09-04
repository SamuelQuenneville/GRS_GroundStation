/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */
#include "consoleInterface.h"

ConsoleInterface::ConsoleInterface(GroundControlStation& gcs, bool& exitFlag, std::mutex& exitMutex, std::condition_variable& cv)
    : m_running(false)
    , m_gcs(gcs)
    , m_exitFlag(exitFlag)
    , m_exitMutex(exitMutex)
    , m_cv(cv)
{

}

ConsoleInterface::~ConsoleInterface() {
    stop();
}

void ConsoleInterface::start() {
    m_running = true;
    m_inputThread = std::thread(&ConsoleInterface::m_listen, this);
}

void ConsoleInterface::stop() {
    m_running = false;
    if (m_inputThread.joinable()) {
        m_inputThread.join();
    }
}

void ConsoleInterface::printCommands() {
    std::cout << "Commands: [commands]\n"
                      << "  start                 --> Start the Ground Station\n"
                      << "  connect               --> Connect to all UAVs\n"
                      << "  arm                   --> Arm all connected system\n"
                      << "  mode [MODE]           --> Set mode for all connected system (MANUAL / GUIDED / XNAV)\n"
                      << "  startController       --> Start the controller\n"
                      << "  launch                --> Init launch sequence\n"
                      << "  fetchParams [ID]      --> Retrieve all parameter and create a .param file\n"
                      << "  loadTraj [FILE]       --> Load a reference trajectory via a .csv file\n"
                      << "  genTraj               --> Generate a reference trajectory in-process (default config, no field calibration -- ADR-001 Phase 1)\n"
                      << "  setOrigin [WP]        --> Set the origin for the controller frame\n"
                      << "  setOriginFromPayload  --> Set the origin from the payload's current live GPS fix\n"
                      << "  listRtkPorts          --> List detected u-blox USB serial devices\n"
                      << "  startRtk [DEV] [BAUD] --> Start RTK base GPS (DEV='auto' to auto-detect), forward corrections to all UAVs (BAUD=0 to auto-detect)\n"
                      << "  stopRtk               --> Stop the RTK base GPS\n"
                      << "  catapultConnect       --> Connect to both catapult ESP32s\n"
                      << "  catapultArm           --> Arm both catapults (requires connect first)\n"
                      << "  catapultFire [MS]     --> Fire both catapults after a synchronized countdown (default 500ms)\n"
                      << "  catapultAbort         --> Cancel countdown / disarm both catapults immediately\n"
                      << "  catapultDisarm        --> Disarm both catapults\n"
                      << "  catapultStatus        --> Show servo-lock / safety-pin / armed status for each launcher\n"
                      << "  stop                  --> Stop the Ground Station\n"
                      << "  exit                  --> Terminate the execution\n";
}

void ConsoleInterface::handleCommand(const std::string& command) const {
    if (command == "start") {
        LOG_INFO("Starting main process...");
        m_gcs.start();
    } else if (command == "connect") {
        m_gcs.connectAll();
    } else if (command == "arm") {
        m_gcs.armAll();
    } else if (command.starts_with("mode ")) {
        static const std::map<std::string, FlightMode> validModes = flightModeMap();
        const std::string mode = command.substr(5);

        if (!validModes.contains(mode)) {
            LOG_ERROR("Unknown mode '" + mode + "'. Valid modes: MANUAL, GUIDED, XNAV");
        } else {
            m_gcs.setModeAll(mode);
        }
    }else if (command == "startController") {
        LOG_INFO("Starting controller ...");
        m_gcs.startController();
    } else if (command == "launch") {
        LOG_INFO("Launching ...");
        m_gcs.initLaunch();
    } else if (command.starts_with("fetchParams ")) {
        const auto sysId = grs::parseInt<int>(command.substr(12));
        if (!sysId || *sysId < 0) {
            LOG_ERROR("Usage: fetchParams [ID]  (ID must be a non-negative integer)");
        } else {
            m_gcs.fetchParam(*sysId);
        }
    } else if (command.starts_with("loadTraj ")) {
        m_gcs.loadTrajectory(command.substr(9));
    } else if (command == "genTraj") {
        LOG_INFO("Generating trajectory in-process (default config)...");
        m_gcs.generateTrajectory(grs::trajgen::TrajectoryConfig{});
    } else if (command.starts_with("setOrigin ")) {
        const std::string args = command.substr(10);
        double lat, lon, alt;

        if (!parseOrigin(args, lat, lon, alt)) {
            LOG_ERROR("Usage: setOrigin lat, lon, alt  OR  setOrigin lat lon alt");
        } else {
            m_gcs.setOrigin(lat, lon, alt);
        }
    } else if (command == "setOriginFromPayload") {
        m_gcs.setOriginFromPayload();
    } else if (command.starts_with("convert ")) {
        const std::string args = command.substr(8);
        double lat, lon, alt;

        if (!parseOrigin(args, lat, lon, alt)) {
            LOG_ERROR("Usage: setOrigin lat, lon, alt  OR  setOrigin lat lon alt");
        } else {
            m_gcs.debugConvert(lat, lon, alt);
        }
    } else if (command == "listRtkPorts") {
        const auto ports = RtkBaseStation::scanAvailablePorts();
        if (ports.empty()) {
            LOG_ERROR("No u-blox USB serial devices found");
        } else {
            for (const auto& port : ports) {
                LOG_INFO("Found: " + port);
            }
        }
    } else if (command.starts_with("startRtk ")) {
        std::istringstream iss(command.substr(9));
        std::string device;
        unsigned baudrate = 0;
        iss >> device >> baudrate;

        if (device.empty()) {
            LOG_ERROR("Usage: startRtk [DEVICE] [BAUD]  e.g. startRtk /dev/ttyACM0 0");
        } else {
            LOG_INFO("Starting RTK base station on " + device + "...");
            m_gcs.startRtkBase(device, baudrate);
        }
    } else if (command == "stopRtk") {
        LOG_INFO("Stopping RTK base station...");
        m_gcs.stopRtkBase();
    } else if (command == "catapultConnect") {
        m_gcs.catapultConnect();
    } else if (command == "catapultArm") {
        m_gcs.catapultArm();
    } else if (command.starts_with("catapultFire")) {
        const std::string arg = command.size() > 12 ? command.substr(13) : "";

        if (arg.empty()) {
            m_gcs.catapultFire();
        } else if (const auto countdownMs = grs::parseInt<uint32_t>(arg); !countdownMs) {
            LOG_ERROR("Usage: catapultFire [MS]  (MS must be a non-negative integer, default 500)");
        } else {
            m_gcs.catapultFire(*countdownMs);
        }
    } else if (command == "catapultAbort") {
        m_gcs.catapultAbort();
    } else if (command == "catapultDisarm") {
        m_gcs.catapultDisarm();
    } else if (command == "catapultStatus") {
        m_gcs.catapultStatus();
    } else if (command == "stop") {
        LOG_INFO("Stopping main process...");
        m_gcs.stop();
    } else if (command == "exit") {
        LOG_INFO("Exiting program...");
        m_gcs.stop();
    } else {
        LOG_ERROR("Unknown command");
    }
}

bool ConsoleInterface::parseOrigin(const std::string& input, double& lat, double& lon, double& alt) {
    std::string s = input;

    // Remove parentheses if present
    std::erase(s, '(');
    std::erase(s, ')');

    // Replace commas with spaces
    std::ranges::replace(s, ',', ' ');

    std::istringstream iss(s);

    if (!(iss >> lat >> lon >> alt))
        return false;

    // Reject extra junk
    std::string extra;
    if (iss >> extra)
        return false;

    // Validate ranges
    if (lat < -90.0 || lat > 90.0) return false;
    if (lon < -180.0 || lon > 180.0) return false;
    if (!std::isfinite(alt)) return false;

    return true;
}

void ConsoleInterface::m_dispatch(const std::string& command) const {
    // Defense in depth: handleCommand() and the code it calls into validate
    // what they reasonably can up front, but plenty of GCS-internal calls
    // (file I/O, YAML parsing, MAVSDK) can still throw for reasons the
    // console layer can't predict. Whatever a command throws, it should log
    // and return control to the prompt.
    try {
        handleCommand(command);
    } catch (const std::exception& e) {
        LOG_ERROR("Command '" + command + "' failed: " + e.what());
    } catch (...) {
        LOG_ERROR("Command '" + command + "' failed with an unknown error");
    }
}

void ConsoleInterface::m_listen() {
    std::string command;
    while (m_running) {
        std::cout << "\nGCS->";

        if (!std::getline(std::cin, command)) {
            // stdin closed (e.g. piped input ran out, or Ctrl-D) treat
            // like "exit" instead of spinning on a stream that will never
            // produce another line.
            LOG_WARNING("Console input closed, shutting down...");
            command = "exit";
        }

        if (!m_running) {
            break;
        }

        if (command == "exit") {
            LOG_INFO("Exiting program...");
            m_gcs.stop();

            {
                std::lock_guard<std::mutex> lock(m_exitMutex);
                m_exitFlag = true;
            }
            m_cv.notify_one();
            m_running = false;
            return;
        }

        if (!command.empty()) {
            m_dispatch(command);
        }
    }
}
