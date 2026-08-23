/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */
#include "consoleInterface.h"

ConsoleInterface::ConsoleInterface(GroundControlStation& gcs, bool& exitFlag, std::condition_variable& cv)
    : m_running(false)
    , m_gcs(gcs)
    , m_exitFlag(exitFlag)
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
                      << "  setOrigin [WP]        --> Set the origin for the controller frame\n"
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
        LOG_INFO("Connecting to all UAVs...");
        m_gcs.connectAll();
    } else if (command == "arm") {
        LOG_INFO("Arming controller ...");
        m_gcs.armAll();
    } else if (command.starts_with("mode ")) {
        LOG_INFO("Setting mode ...");
        m_gcs.setModeAll(command.substr(5));
    }else if (command == "startController") {
        LOG_INFO("Starting controller ...");
        m_gcs.startController();
    } else if (command == "launch") {
        LOG_INFO("Launching ...");
        m_gcs.initLaunch();
    } else if (command.starts_with("fetchParams ")) {
        m_gcs.fetchParam(std::stoi(command.substr(12)));
    } else if (command.starts_with("loadTraj ")) {
        m_gcs.loadTrajectory(command.substr(9));
    } else if (command.starts_with("setOrigin ")) {
        const std::string args = command.substr(10);
        double lat, lon, alt;

        if (!parseOrigin(args, lat, lon, alt)) {
            LOG_INFO("Usage: setOrigin lat, lon, alt  OR  setOrigin lat lon alt");
        }
        m_gcs.setOrigin(lat, lon, alt);
    } else if (command.starts_with("convert ")) {
        const std::string args = command.substr(8);
        double lat, lon, alt;

        if (!parseOrigin(args, lat, lon, alt)) {
            LOG_INFO("Usage: setOrigin lat, lon, alt  OR  setOrigin lat lon alt");
        }

        m_gcs.debugConvert(lat, lon, alt);
    } else if (command == "listRtkPorts") {
        const auto ports = RtkBaseStation::scanAvailablePorts();
        if (ports.empty()) {
            LOG_INFO("No u-blox USB serial devices found");
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
            LOG_INFO("Usage: startRtk [DEVICE] [BAUD]  e.g. startRtk /dev/ttyACM0 0");
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
        m_gcs.catapultFire(arg.empty() ? 500 : static_cast<uint32_t>(std::stoul(arg)));
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
        LOG_INFO("Unknown command");
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

void ConsoleInterface::m_listen() {
    std::string command;
    while (m_running) {
        std::cout << "\nGCS->";
        std::getline(std::cin, command);

        if (!m_running) {
            break;
        }

        if (command == "exit") {
            {
                std::lock_guard<std::mutex> lock(*(new std::mutex())); // short-lived local lock
                m_exitFlag = true;
            }
            handleCommand(command);
            m_cv.notify_one();
            m_running = false;
            return;
        }

        if (!command.empty()) {
            handleCommand(command);
        }
    }
}
