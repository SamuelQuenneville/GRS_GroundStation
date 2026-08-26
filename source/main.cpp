/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <condition_variable>

#include "Console/consoleInterface.h"
#include "Dashboard/dashboardServer.h"
#include "Dashboard/browserLauncher.h"
#include "Util/parseUtils.h"
#include "gcsConfig.h"

// Global synchronization for clean shutdown
std::mutex g_exitMutex;
std::condition_variable g_exitCv;
bool g_exitFlag = false;

int main(const int argc, const char * argv[]) {

    PROGRAM_LOGGER.setLogFileName("log_gcs.txt");

    gcsConfig config;

    try {
        YAML::Node node = YAML::LoadFile(config.configPath);
        config = ConfigurationParser::parseGcsConfig(node, config);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config '" << config.configPath << "': " << e.what() << "\n";
        return 1;
    }

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--verbose") {
            config.verbose = true;

        } else if (arg.find("--UAVs=") == 0) {
            const auto value = grs::parseInt<int>(arg.substr(arg.find('=') + 1));
            if (!value || *value <= 0) {
                std::cerr << "Invalid " << arg << " (expected --UAVs=N with N a positive integer)\n";
                return 1;
            }
            config.numUavs = *value;

        } else if (arg.find("--hlc-freq=") == 0) {
            const auto value = grs::parseDouble(arg.substr(arg.find('=') + 1));
            if (!value || *value <= 0.0) {
                std::cerr << "Invalid " << arg << " (expected --hlc-freq=F with F a positive number)\n";
                return 1;
            }
            config.hlcFrequency = *value;

        } else if (arg.find("--matlab=") == 0) {
            const size_t equalPos = arg.find('=');
            const size_t colonPos = arg.find(':', equalPos);

            if (colonPos == std::string::npos) {
                std::cerr << "Invalid " << arg << " (expected --matlab=ip:port)\n";
                return 1;
            }

            std::string ipStr = arg.substr(equalPos + 1, colonPos - equalPos - 1);
            const auto port = grs::parseInt<int>(arg.substr(colonPos + 1));

            if (ipStr.empty() || !port || *port <= 0 || *port > 65535) {
                std::cerr << "Invalid " << arg << " (expected --matlab=ip:port, with port in 1-65535)\n";
                return 1;
            }

            config.matlab = std::make_pair(std::move(ipStr), static_cast<uint16_t>(*port));
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

    // ---- Dashboard ----
    DashboardServer dashboard(8080, "./dashboard");
    dashboard.start();

    const auto result = BrowserLauncher::launch("http://localhost:8080");

    if (!result.success) {
        std::cerr << "Failed to launch browser: " << result.error.message() << std::endl;
    }

    // ---- Initialize app ----
    GroundControlStation gcs;
    gcs.setDashboard(&dashboard);

    try {
        gcs.initialize(config);
    } catch (const std::exception& e) {
        // Mirrors the try/catch around the GCS config load above: initialize()
        // reloads the same YAML file to parse SolverConfiguration, and a
        // malformed/missing section there should fail with a clear message
        // too, not an unhandled exception.
        std::cerr << "Failed to initialize GCS from '" << config.configPath << "': " << e.what() << "\n";
        return 1;
    }

    // ---- Launch console thread ----
    ConsoleInterface console(gcs, g_exitFlag, g_exitMutex, g_exitCv);
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