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
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "Definitions/logDefinitions.h"
#include "threadSafeQueue.h"

struct LogColumn {
    std::string name;
    std::string unit;
};

// What the log folder records about the run besides the CSVs (manifest.yaml
// and the copied configuration file).
struct LogRunInfo {
    std::string commandLine;   // program arguments as typed
    std::string runDirectory;  // --run-dir, empty if not given
    std::string configFile;    // GCS YAML, copied as config.yaml
};

class Logger {
public:
    static Logger& instance();
    ~Logger();

    // Opens the CSVs in logDirectory, writes manifest.yaml and copies the
    // configuration file. Each CSV starts with its column names (line 1) and
    // units (line 2), registered with setColumns().
    void start(bool enabled, const std::string& logDirectory, const LogRunInfo& info = {});
    void stop();

    // Columns of a log type. Called once the layout is known (Log/logSchema.h),
    // before or after start().
    void setColumns(LogType type, std::vector<LogColumn> columns);

    void log(LogType type, const std::string& line);
    // True when the per-tick logs (verboseLogging) are written.
    [[nodiscard]] bool verbose() const { return m_running && m_enabled; }

    [[nodiscard]] const std::string& directory() const { return m_directory; }

    uint64_t nowMilliseconds() const;
    static uint64_t nowWallTimeMs();
    static std::string getDateString();
    // Double-quoted CSV field, for free text that may hold commas.
    static std::string csvQuoted(const std::string& text);

private:
    Logger() = default;

    std::chrono::steady_clock::time_point m_startTime;

    void m_writerLoop();
    void m_writeManifest(const LogRunInfo& info) const;
    static std::string m_headerText(const std::vector<LogColumn>& columns);

    std::unordered_map<LogType, std::ofstream> m_files;
    std::mutex m_columnsMutex;
    std::unordered_map<LogType, std::vector<LogColumn>> m_columns;
    ThreadSafeQueue<LogItem> m_queue;
    std::thread m_writerThread;
    std::atomic<bool> m_running = false;
    std::atomic<bool> m_enabled = false;
    std::string m_directory;
    uint64_t m_startWallTimeMs = 0;
};

#endif //LOGGER_H
