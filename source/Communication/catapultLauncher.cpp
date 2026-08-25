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



    std::string describeStatusBits(const uint32_t bits) {
        std::string s;
        s += "servoLocked=";        s += (bits & STATUS_COCKED)         ? "YES" : "NO";
        s += "safetyPinRemoved=";   s += (bits & STATUS_SAFETY_PIN_IN)  ? "YES" : "NO";
        s += "armed=";              s += (bits & STATUS_ARMED)          ? "YES" : "NO";
        s += "countdown=";          s += (bits & STATUS_COUNTDOWN)      ? "YES" : "NO";
        if (bits & STATUS_LOW_BATTERY) s += " [LOW BATTERY]";
        if (bits & STATUS_GCS_TIMEOUT) s += " [SELF-DISARMED: GCS TIMEOUT]";

        return s;
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

    for (const auto&[id, port, expectedIp] : endpoints) {
        auto link = std::make_unique<Link>();
        link->id = id;
        link->port = port;
        link->expectedIp = expectedIp;
        m_links.push_back(std::move(link));
    }
}

bool CatapultLauncher::m_bindAndListen(Link& link) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": socket() failed");
        return false;
    }

    constexpr int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(link.port);

    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": bind() failed on port " + std::to_string(link.port));
        close(fd);
        return false;
    }

    if (listen(fd, 1) < 0) {
        LOG_ERROR("Catapult " + std::to_string(link.id) + ": listen() failed on port " + std::to_string(link.port));
        close(fd);
        return false;
    }

    link.listenFd = fd;
    return true;
}

bool CatapultLauncher::connectAll(const int timeoutMs) {
    bool allOk = true;

    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        m_setState(link, CatapultState::Connecting);

        if (!m_bindAndListen(link)) {
            m_setState(link, CatapultState::Fault);
            allOk = false;
            continue;
        }

        link.running = true;
        link.linkThread = std::thread(&CatapultLauncher::m_linkLoop, this, std::ref(link));
        LOG_INFO("Catapult " + std::to_string(link.id) + ": listening on port " + std::to_string(link.port));
    }

    // Give each link up to timeoutMs (shared budget, not per-link) to accept
    // its first connection so connectAll() reports a meaningful result.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        if (link.listenFd < 0) continue; // already marked Fault above

        while (link.fd < 0 && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (link.fd < 0) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": no connection within timeout (port " + std::to_string(link.port) + ") -- will keep listening in the background");
            allOk = false;
        } else {
            LOG_INFO("Catapult " + std::to_string(link.id) + ": connected from " + link.peerIp);
        }
    }

    if (!m_watchdogRunning) {
        m_watchdogRunning = true;
        m_watchdogThread = std::thread(&CatapultLauncher::m_watchdogLoop, this);
    }

    return allOk;
}

void CatapultLauncher::disconnectAll() const {
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        link.running = false;
        if (link.fd >= 0) {
            shutdown(link.fd, SHUT_RDWR);
            close(link.fd);
            link.fd = -1;
        }
        if (link.listenFd >= 0) {
            close(link.listenFd);
            link.listenFd = -1;
        }
        if (link.linkThread.joinable()) {
            link.linkThread.join();
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
        return link.lastAck.has_value() && link.lastAck->seq == seq && link.lastAck->type == static_cast<uint8_t>(expectedType);
    });

    if (got) {
        out = *link.lastAck;
    }
    return got;
}

// Runs for the lifetime of the link: waits for a connection, services it
// until it drops, then goes back to waiting, so a launcher that reboots
// or briefly loses Wi-Fi reconnects without needing connectAll() again.
void CatapultLauncher::m_linkLoop(Link& link) const {
    CatapultPacket pkt{};
    size_t haveBytes = 0;
    auto* buf = reinterpret_cast<uint8_t*>(&pkt);

    while (link.running) {
        if (link.fd < 0) {
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(link.listenFd, &readSet);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 200000; // 200 ms poll so we can recheck `running`

            const int selectResult = select(link.listenFd + 1, &readSet, nullptr, nullptr, &tv);
            if (selectResult <= 0) continue;

            sockaddr_in peerAddr{};
            socklen_t peerLen = sizeof(peerAddr);
            const int clientFd = accept(link.listenFd, reinterpret_cast<sockaddr*>(&peerAddr), &peerLen);
            if (clientFd < 0) continue;

            char ipStr[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &peerAddr.sin_addr, ipStr, sizeof(ipStr));

            if (!link.expectedIp.empty() && link.expectedIp != ipStr) {
                LOG_WARNING("Catapult " + std::to_string(link.id) + ": rejected connection from " + ipStr + " (expected " + link.expectedIp + ")");
                close(clientFd);
                continue;
            }

            constexpr int one = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

            timeval rcvTimeout{};
            rcvTimeout.tv_sec = 0;
            rcvTimeout.tv_usec = 200000;
            setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout));

            haveBytes = 0;
            link.peerIp = ipStr;
            {
                std::lock_guard<std::mutex> lock(link.heartbeatMutex);
                link.lastHeartbeat = std::chrono::steady_clock::now();
            }
            link.fd = clientFd;
            m_setState(link, CatapultState::Connected);
            LOG_INFO("Catapult " + std::to_string(link.id) + ": connected from " + ipStr);
            continue;
        }

        const ssize_t n = recv(link.fd, buf + haveBytes, sizeof(pkt) - haveBytes, 0);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue; // recv timeout, just re-check `running`
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": recv() error");
            close(link.fd);
            link.fd = -1;
            m_setState(link, CatapultState::Fault);
            continue;
        }
        if (n == 0) {
            LOG_WARNING("Catapult " + std::to_string(link.id) + ": connection closed by peer");
            close(link.fd);
            link.fd = -1;
            m_setState(link, CatapultState::Fault);
            continue;
        }

        haveBytes += static_cast<size_t>(n);
        if (haveBytes < sizeof(pkt)) continue;

        if (!catapultValidatePacket(pkt)) {
            // Lost framing sync, shift one byte and keep looking for the magic byte.
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
                m_notifyStatus(link);
                break;

            case MSG_FAULT:
                link.lastStatusBits = pkt.param;
                LOG_ERROR("Catapult " + std::to_string(link.id) + ": reported fault, status=0x" + std::to_string(pkt.param));
                m_setState(link, CatapultState::Fault);
                break;

            case MSG_ARM_ACK:
            case MSG_DISARM_ACK:
            case MSG_FIRE_AT_ACK:
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
                LOG_WARNING("Catapult " + std::to_string(link.id) + ": unexpected message type " + std::to_string(pkt.type));
                break;
        }
    }

    if (link.fd >= 0) {
        close(link.fd);
        link.fd = -1;
    }
}

void CatapultLauncher::m_setState(Link& link, const CatapultState state) const {
    link.state = state;
    m_notifyStatus(link);
}

void CatapultLauncher::m_notifyStatus(const Link &link) const {
    if (m_statusCallback) {
        std::lock_guard<std::mutex> lock(m_callbackMutex);
        m_statusCallback(link.id, link.state.load(), link.lastStatusBits.load());
    }
}

bool CatapultLauncher::armAll(const int timeoutMs) const {
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
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": failed to send ARM (not connected?)");
        }
    }

    bool allArmedOk = true;
    for (size_t i = 0; i < m_links.size(); ++i) {
        Link& link = *m_links[i];
        CatapultPacket ack{};
        const bool ok = m_waitForAck(link, seqs[i], MSG_ARM_ACK, timeoutMs, ack);

        if (!ok) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": no ARM_ACK received (unreachable, or not "
                      "connected) -- last known status: " + describeStatusBits(link.lastStatusBits.load()));
            m_setState(link, CatapultState::Fault);
            allArmedOk = false;
            continue;
        }
        if (!(ack.param & STATUS_ARMED)) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": refused to arm (" + describeStatusBits(ack.param) + ")");
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

void CatapultLauncher::disarmAll() const {
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

bool CatapultLauncher::fireAll(const uint32_t countdownMs, const int acceptTimeoutMs, const int releaseGraceMs) const {
    if (!allArmed()) {
        LOG_ERROR("fireAll() refused: not every catapult is armed.");
        return false;
    }

    if (static_cast<uint32_t>(acceptTimeoutMs) >= countdownMs) {
        LOG_WARNING("fireAll(): acceptTimeoutMs (" + std::to_string(acceptTimeoutMs)
                    + "ms) is not comfortably shorter than countdownMs (" + std::to_string(countdownMs)
                    + "ms) -- there may be little to no time left to actually abort before release.");
    }

    std::vector<uint16_t> seqs;
    seqs.reserve(m_links.size());

    // The whole point: send FIRE_AT to every board back-to-back with no other
    // work interleaved. Each board counts `countdownMs` down locally and
    // releases on its own, so simultaneity only depends on how tight this
    // loop is (microseconds) plus one-way network jitter (asymmetric, but
    // typically << countdownMs on a local Wi-Fi AP).
    const auto sentAt = std::chrono::steady_clock::now();
    for (auto& linkPtr : m_links) {
        Link& link = *linkPtr;
        const uint16_t seq = ++link.seq;
        seqs.push_back(seq);
        m_sendPacket(link, catapultMakePacket(MSG_FIRE_AT, seq, countdownMs));
        m_setState(link, CatapultState::Countdown);
    }

    // Phase 1: the actual abort window: every catapult re-checks armed +
    // interlocks right when FIRE_AT arrives and acks immediately, well
    // before its local countdown elapses. If anyone doesn't confirm within
    // acceptTimeoutMs, there's still time left in the countdown to cancel
    // everyone before anything releases.
    bool allAccepted = true;
    for (size_t i = 0; i < m_links.size(); ++i) {
        Link& link = *m_links[i];
        CatapultPacket ack{};
        const bool ok = m_waitForAck(link, seqs[i], MSG_FIRE_AT_ACK, acceptTimeoutMs, ack);

        if (!ok) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": no FIRE_AT_ACK received -- last known status: "
                      + describeStatusBits(link.lastStatusBits.load()));
            allAccepted = false;
            continue;
        }
        if (!(ack.param & STATUS_COUNTDOWN)) {
            LOG_ERROR("Catapult " + std::to_string(link.id) + ": refused FIRE_AT (" + describeStatusBits(ack.param) + ")");
            allAccepted = false;
            continue;
        }
        LOG_INFO("Catapult " + std::to_string(link.id) + ": confirmed FIRE_AT, counting down.");
    }

    if (!allAccepted) {
        LOG_ERROR("Not every catapult confirmed the fire command -- aborting all before release.");
        abortAll();
        return false;
    }

    LOG_INFO("All catapults confirmed FIRE_AT -- releasing in " + std::to_string(countdownMs) + " ms.");

    // Phase 2: best-effort only, does NOT gate the return value. By this
    // point every catapult has already committed to firing; an abort can no
    // longer reliably stop them together, so there's nothing left to act on
    // here except logging what actually happened.
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - sentAt).count();
    const int remainingMs = std::max<int>(0, static_cast<int>(countdownMs) - static_cast<int>(elapsedMs));
    const int releaseWaitMs = remainingMs + releaseGraceMs;

    for (size_t i = 0; i < m_links.size(); ++i) {
        Link& link = *m_links[i];
        CatapultPacket ack{};
        const bool ok = m_waitForAck(link, seqs[i], MSG_FIRE_ACK, releaseWaitMs, ack);

        if (!ok) {
            LOG_WARNING("Catapult " + std::to_string(link.id) + ": accepted FIRE_AT but no release confirmation received (diagnostic-only, it likely still fired).");
            m_setState(link, CatapultState::Launched);
            continue;
        }
        // ack.param carries the board's own millis() at release time. Boards
        // don't share a clock with the GCS or each other, so this is only
        // useful as a rough diagnostic (e.g. logged deltas across many test
        // launches), not as a synchronization mechanism.
        LOG_INFO("Catapult " + std::to_string(link.id) + ": released, onboard t=" + std::to_string(ack.param) + " ms");
        m_setState(link, CatapultState::Launched);
    }

    return true;
}

void CatapultLauncher::abortAll() const {
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

std::vector<uint8_t> CatapultLauncher::configureIds() const {
    std::vector<uint8_t> ids;
    ids.reserve(m_links.size());
    for (const auto& linkPtr : m_links) {
        ids.push_back(linkPtr->id);
    }
    return ids;
}

std::string CatapultLauncher::describeStatus(const uint8_t id) const {
    for (const auto& linkPtr : m_links) {
        if (linkPtr->id != id) continue;

        const Link& link = *linkPtr;
        std::string s = "Launcher " + std::to_string(id) + ": " + stateName(link.state.load());

        if (link.fd > 0) {
            s += " (" + describeStatusBits(link.lastStatusBits.load()) + ")";
        } else {
            s += " -- not connected";
        }
        return s;
    }
    return "Catapult " + std::to_string(id) + ": not configured";
}

void CatapultLauncher::m_watchdogLoop() const {
    while (m_watchdogRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        const auto now = std::chrono::steady_clock::now();

        for (auto& linkPtr : m_links) {
            Link& link = *linkPtr;
            const CatapultState state = link.state.load();
            if (state == CatapultState::Disconnected || state == CatapultState::Fault || link.fd < 0) continue;

            // Idle keepalive: lets the board's own GCS-liveness watchdog reset even
            // when we're not actively arming/firing, so it never self-disarms just
            // because nothing happened to be sent for a while.

            // This intentionally does NOT skip Countdown. It used to (the
            // reasoning being "the board is about to ACK anyway, a PING here
            // is redundant traffic"), but that only holds for the brief
            // acceptTimeoutMs accept-phase, not the whole remaining
            // countdown. Skipping it for the full Countdown duration meant
            // the firmware's own GCS_TIMEOUT_MS watchdog (2000ms, see the
            // .ino) was racing the countdown from the same t=0 (the FIRE_AT
            // packet) with nothing to reset it in between: any countdownMs
            // approaching 2000ms could self-disarm the board right at
            // release (spurious post-launch fault), and anything at or
            // above 2000ms could self-disarm it before the fire time was
            // ever reached at all, silently dropping the launch.
            if (now - link.lastPingSent > std::chrono::milliseconds(PING_INTERVAL_MS)) {
                m_sendPacket(link, catapultMakePacket(MSG_PING, 0, 0));
                link.lastPingSent = now;
            }

            // Idle keepalive: lets the board's own GCS-liveness watchdog reset even
            // when we're not actively arming/firing, so it never self-disarms just
            // because nothing happened to be sent for a while. Skipped during an
            // active Countdown: the board is about to reply with FIRE_ACK anyway,
            // and a PING landing right before release would be redundant traffic
            // sharing the wire with the ACK the whole handshake is waiting on.
            if (state != CatapultState::Countdown && now - link.lastPingSent > std::chrono::milliseconds(PING_INTERVAL_MS)) {
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
