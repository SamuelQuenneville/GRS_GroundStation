/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

#include "programLogger.h"

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    stop();
}

void Logger::start(const bool enabled, const std::string& logDirectory) {
    m_enabled = enabled;
    if (m_running || !m_enabled) return;

    std::filesystem::create_directories(logDirectory);

    m_files[LogType::MPC_ARG_X0].open(logDirectory + "/mpc_arg_x0.csv");
    m_files[LogType::MPC_ARG_P].open(logDirectory + "/mpc_arg_p.csv");
    m_files[LogType::MPC_ARG_LBX].open(logDirectory + "/mpc_arg_lbx.csv");
    m_files[LogType::MPC_ARG_UBX].open(logDirectory + "/mpc_arg_ubx.csv");
    m_files[LogType::MPC_RES_X].open(logDirectory + "/mpc_res_x.csv");

    m_files[LogType::STATES].open(logDirectory + "/states.csv");
    m_files[LogType::CONTROLS].open(logDirectory + "/controls.csv");

    for (auto& [_, file] : m_files) {
        file.setf(std::ios::unitbuf);
    }

    //m_writeHeaders();

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

    for (auto& [_, file] : m_files) {
        if (file.is_open()) {
            file.close();
        }
    }
}

void Logger::log(const LogType type, const std::string& line) {
    if (!m_running || !m_enabled) return;
    m_queue.push(LogItem{type, line});
}

uint64_t Logger::nowMilliseconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startTime).count();
}

std::string Logger::getDateString() {
    time_t rawTime;
    char buffer[80];

    time(&rawTime);
    const struct tm *timeInfo = localtime(&rawTime);

    strftime(buffer,sizeof(buffer),"/%Y-%m-%d_%H-%M-%S",timeInfo);
    const std::string date(buffer);
    return date;
}

void Logger::m_writeHeaders() {
    m_files[LogType::MPC_ARG_X0] << "time,\n";
    m_files[LogType::MPC_ARG_P]  << "time,\n";
    m_files[LogType::MPC_RES_X]  << "time,\n";
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