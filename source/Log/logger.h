/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <atomic>
#include <string>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <thread>
#include <unordered_map>

#include "Definitions/logDefinitions.h"
#include "threadSafeQueue.h"

class Logger {
public:
    static Logger& instance();
    ~Logger();

    void start(bool enabled, const std::string& logDirectory);
    void stop();

    void log(LogType type, const std::string& line);

    uint64_t nowMilliseconds() const;
    static std::string getDateString();

private:
    Logger() = default;

    std::chrono::steady_clock::time_point m_startTime;

    void m_writeHeaders();
    void m_writerLoop();

    std::unordered_map<LogType, std::ofstream> m_files;
    ThreadSafeQueue<LogItem> m_queue;
    std::thread m_writerThread;
    std::atomic<bool> m_running = false;
    std::atomic<bool> m_enabled = false;
};

#endif //LOGGER_H