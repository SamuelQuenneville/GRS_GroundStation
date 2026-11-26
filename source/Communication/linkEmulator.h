/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef LINKEMULATOR_H
#define LINKEMULATOR_H

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <thread>
#include <vector>

#include "gcsConfig.h"

// Delay of one link direction of one vehicle, for messages handed in time order.
class LinkDelay {
public:
    LinkDelay(const linkDelayConfig& config, uint32_t seed);

    // Delay [s] of a message sent at t [s]. tLaunch: launch time [s] for TRACE, none before launch.
    double next(double t, std::optional<double> tLaunch);

private:
    linkDelayConfig m_config;
    std::mt19937 m_rng;
    std::exponential_distribution<double> m_jitter;
    std::exponential_distribution<double> m_episodeGap;
    std::lognormal_distribution<double> m_peak;

    double m_nextEpisode = 0.0;
    std::optional<double> m_episodeStart;
    double m_episodeLength = 0.0;

    double m_episodeDelay(double t);
    [[nodiscard]] double m_traceDelay(double t) const;
};

// Holds CONTROL_SYSTEM_STATE (downlink) and SET_ATTITUDE_TARGET (uplink) for
// their emulated delay, then runs them on its own thread. Each direction of
// each vehicle stays in order: a message is never released before the one
// sent ahead of it, so a long delay builds a backlog behind it.
class LinkEmulator {
public:
    explicit LinkEmulator(const linkEmulationConfig& config);
    ~LinkEmulator();

    [[nodiscard]] bool delaysDownlink(const uint8_t sysId) const { return m_downlink.contains(sysId); }
    [[nodiscard]] bool delaysUplink(const uint8_t sysId) const { return m_uplink.contains(sysId); }

    void downlink(uint8_t sysId, uint64_t vehicleTimeUsec, std::function<void()> deliver);
    void uplink(uint8_t sysId, std::function<void()> send);

    // Time zero of the traces; the first call counts.
    void setLaunched();

private:
    using Clock = std::chrono::steady_clock;

    struct Channel {
        LinkDelay delay;
        Clock::time_point lastRelease{};
    };

    struct Pending {
        Clock::time_point release;
        uint64_t sequence;
        std::function<void()> action;
        bool operator>(const Pending& other) const {
            return release != other.release ? release > other.release : sequence > other.sequence;
        }
    };

    void m_schedule(Channel& channel, uint8_t sysId, char direction, uint64_t vehicleTimeUsec, std::function<void()> action);
    void m_run();

    const Clock::time_point m_start = Clock::now();
    std::map<uint8_t, Channel> m_downlink;
    std::map<uint8_t, Channel> m_uplink;

    std::mutex m_mutex; // everything below and the channels' state
    std::optional<double> m_launchTime;
    std::priority_queue<Pending, std::vector<Pending>, std::greater<>> m_queue;
    uint64_t m_sequence = 0;
    bool m_stop = false;
    std::condition_variable m_wake;
    std::thread m_thread;
};

#endif //LINKEMULATOR_H
