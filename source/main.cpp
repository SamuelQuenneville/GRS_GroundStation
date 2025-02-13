/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <condition_variable>
#include <mavsdk/mavsdk.h>
#include <mutex>

#include "commandHandler.h"
#include "userInputListener.h"

std::mutex mutex;
std::condition_variable cv;
bool exitFlag = false;

int main(const int argc, const char * argv[]) {

    PROGRAM_LOGGER.setLogFileName("log_gcs.txt");

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {

        std::string arg = argv[i];

        if (arg == "--verbose") {
            PROGRAM_LOGGER.enableVerbose(true);
        } else if (arg.find("--matlab=") == 0) {
            [[maybe_unused]]int port = std::stoi(arg.substr(9));
        }else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --help           Show this help message\n"
                      << "  --verbose        Enable verbose mode\n"
                      << "  --matlab=[port]  Enable matlab controller via UDP\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    GroundStationApp gcs;
    const CommandHandler commandHandler(gcs);

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