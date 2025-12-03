/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef PROFILINGTIMER_H
#define PROFILINGTIMER_H

#pragma once

#include <chrono>
#include <string>

#include "Log/programLogger.h"

class ProfilingTimer {
public:
    explicit ProfilingTimer(std::string name, double* outMs = nullptr)
        : m_name(std::move(name))
        , m_start(std::chrono::steady_clock::now())
        , m_out(outMs)
    {}

    ~ProfilingTimer() {
        using namespace std::chrono;
        const auto end = steady_clock::now();
        const double ms = duration<double, std::milli>(end - m_start).count();

        // Send to output variable if provided
        if (m_out)
            *m_out = ms;

        LOG_INFO("[PROFILE] " + m_name + " took " + std::to_string(ms) + " ms");
    }

private:
    std::string m_name;
    std::chrono::steady_clock::time_point m_start;
    double* m_out;
};

#define PROFILE_SCOPE(name) ProfilingTimer timer##__LINE__(name)
#define PROFILE_SCOPE_OUT(name, outptr) ProfilingTimer timer##__LINE__(name, outptr)


#endif //PROFILINGTIMER_H
