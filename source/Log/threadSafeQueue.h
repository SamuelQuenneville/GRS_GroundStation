/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class ThreadSafeQueue {
public:
    void push(T value) {
        {
            std::scoped_lock lock(m_mutex);
            m_queue.push(std::move(value));
        }
        m_cv.notify_one();
    }

    std::optional<T> pop() {
        std::unique_lock lock(m_mutex);
        m_cv.wait(lock, [this] {
            return !m_queue.empty() || m_stopped;
        });

        if (m_queue.empty()) {
            return std::nullopt;
        }

        auto val = std::move(m_queue.front());
        m_queue.pop();
        return val;
    }

    void stop() {
        {
            std::scoped_lock lock(m_mutex);
            m_stopped = true;
        }
        m_cv.notify_all();
    }

    [[nodiscard]] bool empty() {
        std::scoped_lock lock(m_mutex);
        return m_queue.empty();
    }

    // Clear all remaining items
    void clear() {
        std::scoped_lock lock(m_mutex);
        std::queue<T> emptyQueue;
        std::swap(m_queue, emptyQueue);
    }

private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_stopped = false;
};

#endif //THREADSAFEQUEUE_H
