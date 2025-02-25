/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <condition_variable>

#include "UserInputs/commandHandler.h"
#include "UserInputs/userInputListener.h"

std::mutex mutex;
std::condition_variable cv;
bool exitFlag = false;

int main(const int argc, const char * argv[]) {

    PROGRAM_LOGGER.setLogFileName("log_gcs.txt");

    GroundStationApp gcs;
    const CommandHandler commandHandler(gcs);

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {

        std::string arg = argv[i];

        if (arg == "--verbose") {
            PROGRAM_LOGGER.enableVerbose(true);
        } else if (arg.find("--UAVs=") == 0) {
            gcs.setNumberOfUavs(std::stoi(arg.substr(7)));
        } else if (arg.find("--hlc-freq=") == 0) {
            gcs.setControllerFrequency(std::stod(arg.substr(11)));
        } else if (arg.find("--matlab=") == 0) {
            const size_t equalPos = arg.find('=');
            const size_t colonPos = arg.find(':');

            static std::string ipStr = arg.substr(equalPos + 1, colonPos - equalPos - 1);
            const auto ip = ipStr.c_str();
            const auto port = static_cast<uint16_t>(std::stoi(arg.substr(colonPos + 1)));

            gcs.initMatlabController(ip, port);
        } else if (arg == "--listCommand") {
            CommandHandler::printCommands();
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --help                Show this help message\n"
                      << "  --verbose             Enable verbose mode\n"
                      << "  --UAVs=[Number]       Number of UAVs\n"
                      << "  --hlc-freq=[freq]     Controller frequency in Hz\n"
                      << "  --matlab=[ip]:[port]  Enable matlab controller via UDP\n"
                      << "  --listCommand         Show all commands\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    UserInputListener userInput([&](const std::string& cmd) {
        if (cmd == "exit") {
            {
                std::lock_guard<std::mutex> lock(mutex);
                exitFlag = true;
            }
            cv.notify_one();  // Notify main thread to exit
        }
        commandHandler.handleCommand(cmd);
    });

    userInput.start();

    // Wait for exit signal instead of sleeping
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [] {return exitFlag;});

    return 0;
}