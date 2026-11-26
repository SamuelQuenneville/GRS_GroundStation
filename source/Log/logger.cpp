/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

#include <ctime>
#include <ranges>
#include <filesystem>
#include <fstream>

namespace {

// Single-quoted YAML scalar.
std::string yamlQuoted(const std::string& text) {
    std::string out = "'";
    for (const char c : text) {
        if (c == '\'') out += '\'';
        out += c;
    }
    return out + "'";
}

} // namespace

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    stop();
}

void Logger::start(const bool enabled, const std::string& logDirectory, const LogRunInfo& info) {
    m_enabled = enabled;
    if (m_running) return;

    std::filesystem::create_directories(logDirectory);
    m_directory = logDirectory;
    m_startWallTimeMs = nowWallTimeMs();

    // Heavy per-tick dumps -- one line every solve, gated by `enabled`
    // (verboseLogging) since these are only wanted for detailed
    // post-analysis, not routine test flights.
    if (m_enabled) {
        m_files[LogType::STATES].open(logDirectory + "/states.csv");
        m_files[LogType::CONTROLS].open(logDirectory + "/controls.csv");
        m_files[LogType::MEASURED_STATES].open(logDirectory + "/measured_states.csv");
        m_files[LogType::ESTIMATOR].open(logDirectory + "/estimator.csv");
        m_files[LogType::PLAN].open(logDirectory + "/plan.csv");
        m_files[LogType::TIMING].open(logDirectory + "/timing.csv");
    }

    // Sparse NMPC event log -- always on, regardless of `enabled`: this is
    // the "high value, not everything" log (see MpcController's launch/
    // in-flight/trajectory/violation transition logging), meant to stay
    // readable and useful even on a run where the heavy dumps are off.
    m_files[LogType::NMPC_EVENT].open(logDirectory + "/nmpc_events.csv");

    {
        std::lock_guard lock(m_columnsMutex);
        m_columns[LogType::NMPC_EVENT] = {{"tick", "-"}, {"steadyMs", "ms"}, {"wallTimeMs", "ms"}, {"event", "-"}};
        // Columns registered before start(); the others arrive through setColumns().
        for (auto& [type, file] : m_files) {
            if (const auto it = m_columns.find(type); it != m_columns.end()) {
                file << m_headerText(it->second) << '\n';
            }
        }
    }

    for (auto &file: m_files | std::views::values) {
        file.setf(std::ios::unitbuf);
    }

    m_writeManifest(info);
    if (!info.configFile.empty()) {
        std::error_code ec;
        std::filesystem::copy_file(info.configFile, logDirectory + "/config.yaml",
                                   std::filesystem::copy_options::overwrite_existing, ec);
    }

    m_running = true;
    m_startTime = std::chrono::steady_clock::now();
    m_writerThread = std::thread(&Logger::m_writerLoop, this);
}

void Logger::stop() {
    if (!m_running) return;

    m_running = false;
    m_queue.stop();

    if (m_writerThread.joinable()) {
        m_writerThread.join();
    }

    m_queue.clear();

    for (auto &file: m_files | std::views::values) {
        if (file.is_open()) {
            file.close();
        }
    }
}

void Logger::setColumns(const LogType type, std::vector<LogColumn> columns) {
    std::lock_guard lock(m_columnsMutex);
    // Already started: the header goes through the queue, ahead of the first row.
    if (m_running && m_files.contains(type)) {
        m_queue.push(LogItem{.type = type, .line = m_headerText(columns)});
    }
    m_columns[type] = std::move(columns);
}

void Logger::log(const LogType type, const std::string& line) {
    if (!m_running) return;
    if (type != LogType::NMPC_EVENT && !m_enabled) return;
    m_queue.push(LogItem{.type = type, .line = line});
}

uint64_t Logger::nowMilliseconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startTime).count();
}

uint64_t Logger::nowWallTimeMs() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

std::string Logger::getDateString() {
    time_t rawTime;
    char buffer[80];

    time(&rawTime);
    const tm *timeInfo = localtime(&rawTime);

    strftime(buffer,sizeof(buffer),"/%Y-%m-%d_%H-%M-%S",timeInfo);
    const std::string date(buffer);
    return date;
}

std::string Logger::csvQuoted(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"') out += '"';
        out += c;
    }
    return out + "\"";
}

// Line 1: names, line 2: units. MATLAB reads both with
// detectImportOptions(f, 'VariableNamesLine', 1, 'VariableUnitsLine', 2, 'DataLines', 3).
std::string Logger::m_headerText(const std::vector<LogColumn>& columns) {
    std::string names;
    std::string units;
    for (size_t i = 0; i < columns.size(); ++i) {
        names += (i ? "," : "") + columns[i].name;
        units += (i ? "," : "") + columns[i].unit;
    }
    return names + "\n" + units;
}

void Logger::m_writeManifest(const LogRunInfo& info) const {
    std::ofstream manifest(m_directory + "/manifest.yaml");

    const auto startSec = static_cast<std::time_t>(m_startWallTimeMs / 1000);
    char localTime[40];
    std::strftime(localTime, sizeof(localTime), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&startSec));

    manifest << "program: GRS_GroundStation\n"
             << "logSchema: " << kLogSchemaVersion << "\n"
             << "startWallTimeMs: " << m_startWallTimeMs << "\n"
             << "startLocalTime: " << yamlQuoted(localTime) << "\n"
             << "verboseLogging: " << (m_enabled ? "true" : "false") << "\n"
             << "runDirectory: " << yamlQuoted(info.runDirectory) << "\n"
             << "commandLine: " << yamlQuoted(info.commandLine) << "\n"
             << "configFile: " << yamlQuoted(info.configFile) << "\n"
             << "clocks:\n"
             << "  steadyMs: GCS steady clock, ms since the logger started\n"
             << "  wallTimeMs: Unix time (system clock) when the row was formed\n"
             << "  steadySec: steady clock seconds (arbitrary origin), the controller's sample time\n"
             << "  vehicleTimeUs: autopilot time since boot of the state used (CONTROL_SYSTEM_STATE.time_usec)\n"
             << "frames:\n"
             << "  states: NED navigation frame of the GCS origin (stateVector.h)\n";
}

void Logger::m_writerLoop() {
    while (m_running || !m_queue.empty()) {
        auto item = m_queue.pop();
        if (!item.has_value()) {
            break;
        }

        auto it = m_files.find(item->type);
        if (it != m_files.end() && it->second.is_open()) {
            it->second << item->line << '\n';
        }
    }
}
