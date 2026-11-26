/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "linkEmulator.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "Log/logger.h"

namespace {
    // A zero mean turns the distribution off.
    double rate(const double mean) {
        return mean > 0.0 ? 1.0 / mean : 1.0;
    }
}

LinkDelay::LinkDelay(const linkDelayConfig& config, const uint32_t seed)
    : m_config(config)
    , m_rng(seed)
    , m_jitter(rate(config.jitterMs * 1e-3))
    , m_episodeGap(config.episodeRateHz > 0.0 ? config.episodeRateHz : 1.0)
    , m_peak(std::log(std::max(config.peakMedianMs, 1e-3) * 1e-3), config.peakSigma)
{
    if (m_config.mode == linkDelayConfig::Mode::PARAMETRIC && m_config.episodeRateHz > 0.0) {
        m_nextEpisode = m_episodeGap(m_rng);
    }
}

double LinkDelay::next(const double t, const std::optional<double> tLaunch) {
    double delay = m_config.delayMs * 1e-3;
    if (m_config.jitterMs > 0.0) delay += m_jitter(m_rng);

    switch (m_config.mode) {
        case linkDelayConfig::Mode::PARAMETRIC:
            delay += m_episodeDelay(t);
            break;
        case linkDelayConfig::Mode::TRACE:
            if (tLaunch) delay += m_traceDelay(t - *tLaunch);
            break;
        default:
            break;
    }
    return std::max(delay, 0.0);
}

// Congestion episode: messages keep leaving but slower than they are sent, so
// the delay grows by `growth` per second up to a random peak, then the backlog
// clears at once. Episodes start as a Poisson process of rate episodeRateHz.
double LinkDelay::m_episodeDelay(const double t) {
    if (!(m_config.episodeRateHz > 0.0)) return 0.0;

    if (!m_episodeStart && t >= m_nextEpisode) {
        const double peak = std::min(m_config.peakMinMs * 1e-3 + m_peak(m_rng), m_config.peakMaxMs * 1e-3);
        m_episodeStart = m_nextEpisode;
        m_episodeLength = peak / m_config.growth;
    }
    if (!m_episodeStart) return 0.0;

    const double elapsed = t - *m_episodeStart;
    if (elapsed < m_episodeLength) return m_config.growth * elapsed;

    m_nextEpisode = *m_episodeStart + m_episodeLength + m_episodeGap(m_rng);
    m_episodeStart.reset();
    return 0.0;
}

// Excess delay of the trace at time t since launch, linearly interpolated; 0 outside the trace.
double LinkDelay::m_traceDelay(const double t) const {
    const auto& trace = m_config.trace;
    if (trace.empty() || t < trace.front().first || t > trace.back().first) return 0.0;

    const auto upper = std::lower_bound(trace.begin(), trace.end(), t, [](const auto& sample, const double time) {
        return sample.first < time;
    });
    if (upper == trace.begin()) return upper->second * 1e-3;
    const auto lower = std::prev(upper);
    const double span = upper->first - lower->first;
    const double w = span > 0.0 ? (t - lower->first) / span : 0.0;
    return (lower->second + w * (upper->second - lower->second)) * 1e-3;
}

LinkEmulator::LinkEmulator(const linkEmulationConfig& config) {
    for (const auto& [sysId, delay] : config.downlink) {
        if (delay.mode != linkDelayConfig::Mode::NONE)
            m_downlink.emplace(sysId, Channel{.delay = LinkDelay(delay, config.seed + 2u * sysId)});
    }
    for (const auto& [sysId, delay] : config.uplink) {
        if (delay.mode != linkDelayConfig::Mode::NONE)
            m_uplink.emplace(sysId, Channel{.delay = LinkDelay(delay, config.seed + 2u * sysId + 1u)});
    }
    m_thread = std::thread(&LinkEmulator::m_run, this);
}

LinkEmulator::~LinkEmulator() {
    {
        std::lock_guard lock(m_mutex);
        m_stop = true;
    }
    m_wake.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

void LinkEmulator::downlink(const uint8_t sysId, const uint64_t vehicleTimeUsec, std::function<void()> deliver) {
    m_schedule(m_downlink.at(sysId), sysId, 'D', vehicleTimeUsec, std::move(deliver));
}

void LinkEmulator::uplink(const uint8_t sysId, std::function<void()> send) {
    m_schedule(m_uplink.at(sysId), sysId, 'U', 0, std::move(send));
}

void LinkEmulator::setLaunched() {
    std::lock_guard lock(m_mutex);
    if (!m_launchTime) m_launchTime = std::chrono::duration<double>(Clock::now() - m_start).count();
}

void LinkEmulator::m_schedule(Channel& channel, const uint8_t sysId, const char direction, const uint64_t vehicleTimeUsec, std::function<void()> action) {
    const auto now = Clock::now();
    Clock::time_point release;
    {
        std::lock_guard lock(m_mutex);
        const double t = std::chrono::duration<double>(now - m_start).count();
        const auto delay = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(channel.delay.next(t, m_launchTime)));
        release = std::max(now + delay, channel.lastRelease);
        channel.lastRelease = release;
        m_queue.push(Pending{.release = release, .sequence = m_sequence++, .action = std::move(action)});
    }
    m_wake.notify_one();

    // link.csv: wall ms, sysId, D (downlink) or U (uplink), vehicle time_usec (downlink), delay ms
    std::ostringstream line;
    line << Logger::nowWallTimeMs() << "," << static_cast<int>(sysId) << "," << direction << "," << vehicleTimeUsec << ","
         << std::chrono::duration<double, std::milli>(release - now).count();
    Logger::instance().log(LogType::LINK, line.str());
}

void LinkEmulator::m_run() {
    std::unique_lock lock(m_mutex);
    while (!m_stop) {
        if (m_queue.empty()) {
            m_wake.wait(lock);
            continue;
        }
        const auto release = m_queue.top().release;
        if (Clock::now() < release) {
            m_wake.wait_until(lock, release);
            continue;
        }
        auto action = std::move(const_cast<Pending&>(m_queue.top()).action);
        m_queue.pop();
        lock.unlock();
        action();
        lock.lock();
    }
}
