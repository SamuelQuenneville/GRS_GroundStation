/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef CATAPULTLAUNCHER_H
#define CATAPULTLAUNCHER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <cstring>
#include <algorithm>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "Log/programLogger.h"
#include "Definitions/catapultProtocol.h"

enum class CatapultState {
    Disconnected,
    Connecting,
    Connected,   // TCP link up, heartbeating, not armed
    Arming,      // ARM sent, waiting for ARM_ACK
    Armed,       // both catapults confirmed armed and cocked
    Countdown,   // FIRE_AT sent, waiting for local release
    Launched,
    Fault
};

struct CatapultEndpoint {
    uint8_t id;
    uint16_t port = CATAPULT_PORT;
    std::string expectedIp;
};

// Controls N launch catapults (nominally 2), each an ESP32 that dials IN to
// this GCS over Wi-Fi/TCP (one listen port per catapult see
// CatapultEndpoint). Firing simultaneity is achieved by sending every
// catapult a FIRE_AT(countdownMs) command back-to-back and letting each
// board fire off its own local hardware timer after the countdown elapses
// this only depends on one-way network jitter between the GCS and each
// board, not on round-trip time or on the two boards sharing a clock.
//
// Safety model: catapults only fire after an explicit two-step ARM -> FIRE.
// armAll() requires every catapult to ack "cocked + armed" before returning
// success; if any catapult fails to arm, whichever ones did arm are
// automatically disarmed so we never leave a single catapult live. Each
// board also runs its own GCS-heartbeat watchdog and self-disarms if it
// loses contact with the GCS; the GCS mirrors that by watching heartbeats
// too and reporting Fault if a link goes quiet while armed/counting down.
class CatapultLauncher {

public:
    using StatusCallback = std::function<void(uint8_t id, CatapultState state, uint32_t statusBits)>;

    CatapultLauncher();
    ~CatapultLauncher();

    void configure(const std::vector<CatapultEndpoint>& endpoints);

    // Opens a listen socket per catapult and waits up to timeoutMs for each
    // one's first connection. A launcher that connects later (e.g. it was
    // still booting) will still be picked up in the background, this only
    // affects what connectAll() itself reports as success/failure.
    bool connectAll(int timeoutMs = 3000);
    void disconnectAll();

    // Blocks until every catapult acks ARM with STATUS_ARMED set, or timeoutMs
    // elapses. On partial failure, automatically disarms whichever catapults
    // did arm and returns false.
    bool armAll(int timeoutMs = 3000);
    void disarmAll();

    // Requires armAll() success. Sends FIRE_AT to every catapult as fast as
    // possible, one after another with no work in between. Returns true once
    // every catapult acks that it actually released (STATUS_COUNTDOWN cleared
    // via MSG_FIRE_ACK), or false if any didn't within timeoutMs.
    bool fireAll(uint32_t countdownMs = 500, int ackTimeoutMs = 3000);

    // Cancels a pending countdown and disarms. Safe to call in any state.
    void abortAll();

    [[nodiscard]] CatapultState getState(uint8_t id) const;
    [[nodiscard]] bool allArmed() const;
    void setStatusCallback(StatusCallback cb);

    static constexpr int HEARTBEAT_TIMEOUT_MS = 2000;
    static constexpr int PING_INTERVAL_MS = 500;

private:
    struct Link {
        uint8_t id = 0;
        uint16_t port = CATAPULT_PORT;
        std::string expectedIp;

        int listenFd = -1;
        int fd = -1;              // -1 until a launcher has connected
        std::string peerIp;       // set once a client connects

        std::atomic<CatapultState> state{CatapultState::Disconnected};
        std::atomic<uint32_t> lastStatusBits{0};
        std::atomic<uint16_t> seq{0};

        std::thread linkThread;
        std::atomic<bool> running{false};

        std::mutex heartbeatMutex;
        std::chrono::steady_clock::time_point lastHeartbeat;
        std::chrono::steady_clock::time_point lastPingSent;

        std::mutex ackMutex;
        std::condition_variable ackCv;
        std::optional<CatapultPacket> lastAck;
    };

    std::vector<std::unique_ptr<Link>> m_links;

    StatusCallback m_statusCallback;
    mutable std::mutex m_callbackMutex;

    std::thread m_watchdogThread;
    std::atomic<bool> m_watchdogRunning{false};

    static bool m_bindAndListen(Link& link);
    void m_linkLoop(Link& link); // accepts, then services one connection, repeats
    static bool m_sendPacket(const Link& link, const CatapultPacket& pkt);

    static bool m_waitForAck(Link& link, uint16_t seq, CatapultMsgType expectedType, int timeoutMs, CatapultPacket& out);
    void m_setState(Link& link, CatapultState state) const;
    void m_watchdogLoop();
};

#endif //CATAPULTLAUNCHER_H