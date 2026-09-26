/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "estimatorRunner.h"

// ---------------------------------------------------------------------------
// ThreadedEstimatorRunner
// ---------------------------------------------------------------------------

ThreadedEstimatorRunner::ThreadedEstimatorRunner(Estimator& estimator, const double nmheFrequency, const size_t maxPending)
    : m_estimator(estimator)
    , m_periodMs(1000.0 / nmheFrequency)
    , m_maxPending(std::max<size_t>(maxPending, 2))
    , m_worker(&ThreadedEstimatorRunner::m_loop, this)
{
}

ThreadedEstimatorRunner::~ThreadedEstimatorRunner() {
    m_running = false;
    {
        std::lock_guard lock(m_wakeMutex);
    }
    m_wake.notify_all();
    if (m_worker.joinable()) m_worker.join();
}

std::optional<EstimatorRunner::Estimate> ThreadedEstimatorRunner::takeEstimate() {
    std::lock_guard lock(m_outMutex);
    if (!m_hasNew) return std::nullopt;
    m_hasNew = false;
    return m_latest;
}

void ThreadedEstimatorRunner::pushSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) {
    std::lock_guard lock(m_inMutex);
    m_pending.emplace_back(measuredState, appliedControl);
    while (m_pending.size() > m_maxPending) m_pending.pop_front();
}

EstimatorRunner::Stats ThreadedEstimatorRunner::stats() const {
    std::lock_guard lock(m_outMutex);
    return m_stats;
}

void ThreadedEstimatorRunner::m_loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double, std::milli>(m_periodMs));
    auto next = clock::now() + period;

    std::deque<std::pair<std::vector<double>, std::vector<double>>> batch;
    while (m_running) {
        {
            std::unique_lock lock(m_wakeMutex);
            m_wake.wait_until(lock, next, [this] { return !m_running.load(); });
        }
        if (!m_running) break;

        // Pull everything the control thread pushed since the last wake-up;
        // the Estimator is only ever touched from this thread.
        {
            std::lock_guard lock(m_inMutex);
            batch.swap(m_pending);
        }
        for (const auto& [x, u] : batch) m_estimator.addSample(x, u);
        batch.clear();

        const auto t0 = clock::now();
        const bool ok = m_estimator.estimate();
        const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
        const bool attempted = m_estimator.getDebugInfo().windowFull;

        {
            std::lock_guard lock(m_outMutex);
            if (attempted) {
                ++m_stats.solves;
                m_stats.lastSolveMs = ms;
                if (ms > m_periodMs) ++m_stats.overruns;
            }
            if (ok) {
                m_latest.wind = m_estimator.windEstimate();
                m_latest.d = m_estimator.dEstimate();
                m_hasNew = true;
                ++m_stats.published;
            }
        }

        // Fixed-rate schedule; after an overrun, restart from now instead of
        // firing a burst of catch-up solves.
        next += period;
        const auto now = clock::now();
        if (next < now) {
            LOG_WARNING("NMHE solve took " + std::to_string(ms) + " ms, longer than its " +
                        std::to_string(m_periodMs) + " ms period");
            next = now + period;
        }
    }
}

// ---------------------------------------------------------------------------
// DeferredEstimatorRunner
// ---------------------------------------------------------------------------

DeferredEstimatorRunner::DeferredEstimatorRunner(Estimator& estimator, const double hlcFrequency,
                                                 const double nmheFrequency, const double latencyMs)
    : m_estimator(estimator)
    , m_tickMs(1000.0 / hlcFrequency)
    , m_periodMs(1000.0 / nmheFrequency)
    , m_latencyMs(latencyMs)
{
}

std::optional<EstimatorRunner::Estimate> DeferredEstimatorRunner::takeEstimate() {
    // Deliver the newest estimate whose solve has "finished" by this tick.
    std::optional<Estimate> out;
    while (!m_inFlight.empty() && m_inFlight.front().first <= m_tick) {
        out = std::move(m_inFlight.front().second);
        m_inFlight.pop_front();
    }
    return out;
}

void DeferredEstimatorRunner::pushSample(const std::vector<double>& measuredState, const std::vector<double>& appliedControl) {
    m_estimator.addSample(measuredState, appliedControl);
}

void DeferredEstimatorRunner::endTick() {
    m_stats.solvedThisTick = false;
    m_accumulatorMs += m_tickMs;
    // Small tolerance: 4 x 50.0 ms must count as one 200 ms period.
    if (m_accumulatorMs >= m_periodMs - 1e-9) {
        m_accumulatorMs = 0.0;
        const bool ok = m_estimator.estimate();
        const auto dbg = m_estimator.getDebugInfo();
        if (dbg.windowFull) {
            ++m_stats.solves;
            m_stats.lastSolveMs = dbg.lastSolveMs;
            m_stats.solvedThisTick = true;
        }
        if (ok) {
            const double ms = m_latencyMs >= 0.0 ? m_latencyMs : dbg.lastSolveMs;
            const auto ticks = static_cast<uint64_t>(std::max(1.0, std::ceil(ms / m_tickMs - 1e-9)));
            m_inFlight.emplace_back(m_tick + ticks, Estimate{m_estimator.windEstimate(), m_estimator.dEstimate()});
            ++m_stats.published;
        }
    }
    ++m_tick;
}
