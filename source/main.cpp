/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <condition_variable>

#include "Console/consoleInterface.h"
#include "gcsConfig.h"

// Global synchronization for clean shutdown
std::mutex g_exitMutex;
std::condition_variable g_exitCv;
bool g_exitFlag = false;

int main(const int argc, const char * argv[]) {

    PROGRAM_LOGGER.setLogFileName("log_gcs.txt");

    gcsConfig config;

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--verbose") {
            config.verbose = true;

        } else if (arg.find("--UAVs=") == 0) {
            config.numUavs = std::stoi(arg.substr(arg.find('=') + 1));

        } else if (arg.find("--hlc-freq=") == 0) {
            config.hlcFrequency = std::stod(arg.substr(arg.find('=') + 1));

        } else if (arg.find("--matlab=") == 0) {
            const size_t equalPos = arg.find('=');
            const size_t colonPos = arg.find(':');

            static std::string ipStr = arg.substr(equalPos + 1, colonPos - equalPos - 1);
            const auto ip = ipStr.c_str();
            const auto port = static_cast<uint16_t>(std::stoi(arg.substr(colonPos + 1)));
            config.matlab = std::make_pair(ip, port);
            config.controlMode = ControlMode::MATLAB;

        } else if (arg.find("--commandFile=") == 0) {
            config.attitudeFile = arg.substr(arg.find('=') + 1);
            config.controlMode = ControlMode::ATTITUDE_FILE;

        } else if (arg == "--sitl") {
            config.pixhawk.sitl = true;

        } else if (arg == "--listCommand") {
            ConsoleInterface::printCommands();

        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --help                Show this help message\n"
                      << "  --verbose             Enable verbose mode\n"
                      << "  --UAVs=[Number]       Number of UAVs\n"
                      << "  --hlc-freq=[freq]     Controller frequency in Hz\n"
                      << "  --matlab=[ip]:[port]  Enable matlab controller via UDP\n"
                      << "  --commandFile=[file]  Enable control input (RPYT) from file\n"
                      << "  --listCommand         Show all commands\n"
                      << "  --sitl                Enable SITL mode\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    // ---- Launcher ----
    const catapultEndpointConfig launcher1{1,"192.168.4.101"};
    const catapultEndpointConfig launcher2{2,"192.168.4.102"};
    const std::vector<catapultEndpointConfig> launcher{launcher1, launcher2};

    config.catapults = launcher;

    // ---- Initialize app ----
    GroundControlStation gcs;
    gcs.initialize(config);

    // ---- Launch console thread ----
    ConsoleInterface console(gcs, g_exitFlag, g_exitCv);
    console.start();

    // ---- Wait for exit ----
    std::unique_lock lock(g_exitMutex);
    g_exitCv.wait(lock, [] {return g_exitFlag;});

    LOG_INFO("Shutting down GCS...");
    gcs.stop();
    console.stop();

    LOG_INFO("Ground Station terminated cleanly.");

    return 0;
}