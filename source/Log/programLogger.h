/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef PROGRAMLOGGER_H
#define PROGRAMLOGGER_H

#pragma once

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>

enum class LogLevel {
    debug = 1,
    info,
    warning,
    error
};

#define PROGRAM_LOGGER ProgramLogger::instance()

#define LOG_DEBUG(msg) PROGRAM_LOGGER.log(LogLevel::debug, msg)
#define LOG_INFO(msg) PROGRAM_LOGGER.log(LogLevel::info, msg)
#define LOG_WARNING(msg) PROGRAM_LOGGER.log(LogLevel::warning, msg)
#define LOG_ERROR(msg) PROGRAM_LOGGER.log(LogLevel::error, msg)


class ProgramLogger {

public:
    static ProgramLogger& instance();

    void log(LogLevel level, const std::string& message);
    void setLogFileName(const std::string& fileName);
    void enableVerbose(bool enable);

private:
    ProgramLogger() = default;
    ~ProgramLogger();
    ProgramLogger(const ProgramLogger&) = delete;
    ProgramLogger& operator=(const ProgramLogger&) = delete;

    std::ofstream m_logFile;
    std::mutex m_logMutex;
    bool m_verbose = false;

    static std::string m_getLevelString(LogLevel level);
};

#endif //PROGRAMLOGGER_H
