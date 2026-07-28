/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "catapultLauncher.h"

namespace {
    std::string stateName(const CatapultState s) {
        switch (s) {
            case CatapultState::Disconnected: return "Disconnected";
            case CatapultState::Connecting:   return "Connecting";
            case CatapultState::Connected:    return "Connected";
            case CatapultState::Arming:       return "Arming";
            case CatapultState::Armed:        return "Armed";
            case CatapultState::Countdown:    return "Countdown";
            case CatapultState::Launched:     return "Launched";
            case CatapultState::Fault:        return "Fault";
        }
        return "Unknown";
    }
}

CatapultLauncher::CatapultLauncher() = default;

CatapultLauncher::~CatapultLauncher() {
    m_watchdogRunning = false;
    if (m_watchdogThread.joinable()) {
        m_watchdogThread.join();
    }
    disconnectAll();
}

void CatapultLauncher::configure(const std::vector<CatapultEndpoint>& endpoints) {
    disconnectAll();
    m_links.clear();

    for (const auto&[id, ip, port] : endpoints) {
        auto link = std::make_unique<Link>();
        link->id = id;
        link->ip = ip;
        link->port = port;
        m_links.push_back(std::move(link));
    }
}

bool CatapultLauncher::m_connectLink(Link& link, const int timeoutMs) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": socket() failed");
        return false;
    }

    // Non-blocking connect with a timeout, so a dead/unreachable catapult
    // doesn't stall connectAll() for the OS default TCP timeout (~2 min).
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(link.port);
    if (inet_pton(AF_INET, link.ip.c_str(), &addr.sin_addr) != 1) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": invalid IP " + link.ip);
        close(fd);
        return false;
    }

    const int result = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (result < 0 && errno != EINPROGRESS) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": connect() failed");
        close(fd);
        return false;
    }

    if (result != 0) {
        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(fd, &writeSet);
        timeval tv{};
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;

        const int selectResult = select(fd + 1, nullptr, &writeSet, nullptr, &tv);
        if (selectResult <= 0) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": connect() timed out");
            close(fd);
            return false;
        }

        int soError = 0;
        socklen_t len = sizeof(soError);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &soError, &len);
        if (soError != 0) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": connect() reported error " + std::to_string(soError));
            close(fd);
            return false;
        }
    }

    // Back to blocking mode for the rx thread; disable Nagle so FIRE_AT goes
    // out immediately instead of waiting to coalesce with other traffic --
    // that coalescing delay is exactly the kind of jitter we're trying to avoid.
    fcntl(fd, F_SETFL, flags);
    constexpr int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    // Recv timeout so the rx loop can periodically check `running` and exit cleanly.
    timeval rcvTimeout{};
    rcvTimeout.tv_sec = 0;
    rcvTimeout.tv_usec = 200000; // 200 ms
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout));

    link.fd = fd;
    return true;
}

bool CatapultLauncher::connectAll(const int timeoutMs) {
    bool allOk = true;

    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        m_setState(link, CatapultState::Connecting);

        if (!m_connectLink(link, timeoutMs)) {
            m_setState(link, CatapultState::Fault);
            allOk = false;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(link.heartbeatMutex);
            link.lastHeartbeat = std::chrono::steady_clock::now();
        }

        link.running = true;
        link.rxThread = std::thread(&CatapultLauncher::m_rxLoop, this, std::ref(link));
        m_setState(link, CatapultState::Connected);
        LOG_INFO("Catapult " + std::to_string(link.id) + ": connected to " + link.ip + ":" + std::to_string(link.port));
    }

    if (!m_watchdogRunning) {
        m_watchdogRunning = true;
        m_watchdogThread = std::thread(&CatapultLauncher::m_watchdogLoop, this);
    }

    return allOk;
}

void CatapultLauncher::disconnectAll() {
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        link.running = false;
        if (link.fd >= 0) {
            shutdown(link.fd, SHUT_RDWR);
            close(link.fd);
            link.fd = -1;
        }
        if (link.rxThread.joinable()) {
            link.rxThread.join();
        }
        m_setState(link, CatapultState::Disconnected);
    }
}

bool CatapultLauncher::m_sendPacket(const Link& link, const CatapultPacket& pkt) {
    if (link.fd < 0) return false;
    const ssize_t sent = send(link.fd, &pkt, sizeof(pkt), MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(sizeof(pkt));
}

bool CatapultLauncher::m_waitForAck(Link& link, const uint16_t seq, const CatapultMsgType expectedType, const int timeoutMs, CatapultPacket& out) {
    std::unique_lock<std::mutex> lock(link.ackMutex);
    const bool got = link.ackCv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&] {
        return link.lastAck.has_value()
            && link.lastAck->seq == seq
            && link.lastAck->type == static_cast<uint8_t>(expectedType);
    });
    if (got) {
        out = *link.lastAck;
    }
    return got;
}

void CatapultLauncher::m_rxLoop(Link& link) const {
    CatapultPacket pkt{};
    size_t haveBytes = 0;
    auto* buf = reinterpret_cast<uint8_t*>(&pkt);

    while (link.running) {
        const ssize_t n = recv(link.fd, buf + haveBytes, sizeof(pkt) - haveBytes, 0);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue; // recv timeout, just re-check `running`
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": recv() error");
            m_setState(link, CatapultState::Fault);
            break;
        }
        if (n == 0) {
            LOG_WARNING("Catapult " + std::to_string(link.id) + ": connection closed by peer");
            m_setState(link, CatapultState::Fault);
            break;
        }

        haveBytes += static_cast<size_t>(n);
        if (haveBytes < sizeof(pkt)) continue;

        if (!catapultValidatePacket(pkt)) {
            // Lost framing sync -- shift one byte and keep looking for the magic byte.
            std::memmove(buf, buf + 1, sizeof(pkt) - 1);
            haveBytes = sizeof(pkt) - 1;
            continue;
        }
        haveBytes = 0;

        switch (static_cast<CatapultMsgType>(pkt.type)) {
            case MSG_HEARTBEAT:
                link.lastStatusBits = pkt.param;
                {
                    std::lock_guard<std::mutex> lock(link.heartbeatMutex);
                    link.lastHeartbeat = std::chrono::steady_clock::now();
                }
                break;

            case MSG_FAULT:
                link.lastStatusBits = pkt.param;
                LOG_ERROR("Catapult " + std::to_string(link.id) + ": reported fault, status=0x"
                          + std::to_string(pkt.param));
                m_setState(link, CatapultState::Fault);
                break;

            case MSG_ARM_ACK:
            case MSG_DISARM_ACK:
            case MSG_FIRE_ACK:
            case MSG_ABORT_ACK: {
                link.lastStatusBits = pkt.param;
                {
                    std::lock_guard<std::mutex> lock(link.heartbeatMutex);
                    link.lastHeartbeat = std::chrono::steady_clock::now();
                }
                std::lock_guard<std::mutex> ackLock(link.ackMutex);
                link.lastAck = pkt;
                link.ackCv.notify_all();
                break;
            }

            default:
                LOG_WARNING("Catapult " + std::to_string(link.id) + ": unexpected message type "
                            + std::to_string(pkt.type));
                break;
        }
    }
}

void CatapultLauncher::m_setState(Link& link, const CatapultState state) const {
    link.state = state;
    if (m_statusCallback) {
        std::lock_guard<std::mutex> lock(m_callbackMutex);
        m_statusCallback(link.id, state, link.lastStatusBits.load());
    }
}

bool CatapultLauncher::armAll(const int timeoutMs) {
    if (m_links.empty()) return false;

    std::vector<uint16_t> seqs;
    seqs.reserve(m_links.size());

    // Fire off ARM to every catapult first, then wait -- don't wait link by
    // link, or N-1 round trips get serialized into the timeout budget.
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        const uint16_t seq = ++link.seq;
        seqs.push_back(seq);
        m_setState(link, CatapultState::Arming);
        if (!m_sendPacket(link, catapultMakePacket(MSG_ARM, seq, 0))) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": failed to send ARM");
        }
    }

    bool allArmedOk = true;
    for (size_t i = 0; i < m_links.size(); ++i) {
        Link& link = *m_links[i];
        CatapultPacket ack{};
        const bool ok = m_waitForAck(link, seqs[i], MSG_ARM_ACK, timeoutMs, ack);

        if (!ok || !(ack.param & STATUS_ARMED)) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": failed to arm");
            m_setState(link, CatapultState::Fault);
            allArmedOk = false;
            continue;
        }
        m_setState(link, CatapultState::Armed);
    }

    if (!allArmedOk) {
        LOG_ERROR("Not all catapults armed -- disarming everyone for safety.");
        disarmAll();
        return false;
    }

    LOG_INFO("All catapults armed.");
    return true;
}

void CatapultLauncher::disarmAll() {
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        if (link.fd < 0) continue;
        const uint16_t seq = ++link.seq;
        m_sendPacket(link, catapultMakePacket(MSG_DISARM, seq, 0));

        CatapultPacket ack{};
        m_waitForAck(link, seq, MSG_DISARM_ACK, 1000, ack); // best-effort, don't block safety on this
        m_setState(link, CatapultState::Connected);
    }
}

bool CatapultLauncher::fireAll(const uint32_t countdownMs, const int ackTimeoutMs) {
    if (!allArmed()) {
        LOG_ERROR("fireAll() refused: not every catapult is armed.");
        return false;
    }

    std::vector<uint16_t> seqs;
    seqs.reserve(m_links.size());

    // The whole point: send FIRE_AT to every board back-to-back with no other
    // work interleaved. Each board counts `countdownMs` down locally and
    // releases on its own, so simultaneity only depends on how tight this
    // loop is (microseconds) plus one-way network jitter (asymmetric, but
    // typically << countdownMs on a local Wi-Fi AP).
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        const uint16_t seq = ++link.seq;
        seqs.push_back(seq);
        m_sendPacket(link, catapultMakePacket(MSG_FIRE_AT, seq, countdownMs));
        m_setState(link, CatapultState::Countdown);
    }

    bool allFired = true;
    for (size_t i = 0; i < m_links.size(); ++i) {
        Link& link = *m_links[i];
        CatapultPacket ack{};
        const bool ok = m_waitForAck(link, seqs[i], MSG_FIRE_ACK, ackTimeoutMs, ack);

        if (!ok) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": no FIRE_ACK received -- did it actually release?");
            m_setState(link, CatapultState::Fault);
            allFired = false;
            continue;
        }
        // ack.param carries the board's own millis() at release time. Boards
        // don't share a clock with the GCS or each other, so this is only
        // useful as a rough diagnostic (e.g. logged deltas across many test
        // launches), not as a synchronization mechanism.
        LOG_INFO("Catapult " + std::to_string(link.id) + ": released, onboard t=" + std::to_string(ack.param) + " ms");
        m_setState(link, CatapultState::Launched);
    }

    return allFired;
}

void CatapultLauncher::abortAll() {
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        if (link.fd < 0) continue;
        const uint16_t seq = ++link.seq;
        m_sendPacket(link, catapultMakePacket(MSG_ABORT, seq, 0));
        m_setState(link, CatapultState::Connected);
    }
    LOG_WARNING("Catapult launch sequence aborted.");
}

CatapultState CatapultLauncher::getState(const uint8_t id) const {
    for (const auto& linkPtr : m_links) {
        if (linkPtr->id == id) return linkPtr->state.load();
    }
    return CatapultState::Disconnected;
}

bool CatapultLauncher::allArmed() const {
    if (m_links.empty()) return false;
    return std::ranges::all_of(m_links, [](const auto& linkPtr) {
        return linkPtr->state.load() == CatapultState::Armed;
    });
}

void CatapultLauncher::setStatusCallback(StatusCallback cb) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_statusCallback = std::move(cb);
}

void CatapultLauncher::m_watchdogLoop() {
    while (m_watchdogRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        const auto now = std::chrono::steady_clock::now();

        for (auto& linkPtr : m_links) {
            Link& link = *linkPtr;
            const CatapultState state = link.state.load();
            if (state == CatapultState::Disconnected || state == CatapultState::Fault) continue;

            // Idle keepalive: lets the board's own GCS-liveness watchdog reset even
            // when we're not actively arming/firing, so it never self-disarms just
            // because nothing happened to be sent for a while.
            if (now - link.lastPingSent > std::chrono::milliseconds(PING_INTERVAL_MS)) {
                m_sendPacket(link, catapultMakePacket(MSG_PING, 0, 0));
                link.lastPingSent = now;
            }

            if (state != CatapultState::Armed && state != CatapultState::Countdown) continue;

            std::chrono::steady_clock::time_point last;
            {
                std::lock_guard<std::mutex> lock(link.heartbeatMutex);
                last = link.lastHeartbeat;
            }
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();

            if (elapsedMs > HEARTBEAT_TIMEOUT_MS) {
                LOG_ERROR("Catapult " + std::to_string(link.id)
                          + ": heartbeat lost while " + stateName(state)
                          + " -- treating as fault (board should self-disarm independently).");
                m_setState(link, CatapultState::Fault);
            }
        }
    }
}