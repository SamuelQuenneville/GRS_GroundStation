/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "programLogger.h"

ProgramLogger& ProgramLogger::instance() {
    static ProgramLogger instance;
    return instance;
}

ProgramLogger::~ProgramLogger() {
    if (m_logFile.is_open()) {
        m_logFile.close();
    }
}

void ProgramLogger::setLogFileName(const std::string& fileName) {
    std::lock_guard<std::mutex> lock(m_logMutex);
    if (m_logFile.is_open()) {
        m_logFile.close();
    }
    m_logFile.open(fileName, std::ios::app);
}

void ProgramLogger::enableVerbose(const bool enable) {
    m_verbose = enable;
}

void ProgramLogger::log(const LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(m_logMutex);
    const std::string logMessage = m_getLevelString(level) + message;

    if (m_verbose) {
        std::cout << logMessage << std::endl;
    }

    if (m_logFile.is_open()) {
        m_logFile << logMessage << std::endl;
    }
}

std::string ProgramLogger::m_getLevelString(const LogLevel level) {
    switch (level) {
        case LogLevel::debug: return "[DEBUG] ";
        case LogLevel::info: return "[INFO] ";
        case LogLevel::warning: return "[WARNING] ";
        case LogLevel::error: return "[ERROR] ";
        default: return "[UNKNOWN] ";
    }
}