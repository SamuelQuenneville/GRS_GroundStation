/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef DASHBOARDSERVER_H
#define DASHBOARDSERVER_H

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "httplib.h"
#include "dashboardTypes.h"

class DashboardServer {

public:
    // port: the single port the dashboard is served on (static files +
    //       WebSocket both happen here, at ws(s)://host:port/ws).
    // staticRoot: path to resources/dashboard (contains index.html/css/js).
    // broadcastRateHz: how often buffered telemetry is pushed to clients.
    DashboardServer(uint16_t port, std::string staticRoot, int broadcastRateHz = 5);
    ~DashboardServer();

    DashboardServer(const DashboardServer&) = delete;
    DashboardServer& operator=(const DashboardServer&) = delete;

    void start();
    void stop();

    // Thread-safe.
    // Just updates the buffered snapshot, the broadcast thread decides when to actually send it out.
    void updateTelemetry(const UavTelemetrySnapshot& snapshot);
    void updatePayloadTelemetry(const PayloadTelemetrySnapshot& snapshot);
    void updateLauncherTelemetry(const LauncherTelemetrySnapshot& snapshot);
    void updateNmpcTelemetry(const NmpcTelemetrySnapshot& snapshot);

    size_t connectedBrowserCount() const;

private:
    void m_broadcastLoop();

    uint16_t m_port;
    std::string m_staticRoot;
    int m_broadcastRateHz;

    std::unique_ptr<httplib::Server> m_httpServer;
    std::thread m_serverThread;

    mutable std::mutex m_clientsMutex;
    std::vector<httplib::ws::WebSocket*> m_clients;

    std::mutex m_snapshotsMutex;
    std::unordered_map<std::string, UavTelemetrySnapshot> m_latestSnapshots;

    PayloadTelemetrySnapshot m_latestPayloadSnapshot;
    bool m_hasPayloadSnapshot = false;

    std::unordered_map<std::string, LauncherTelemetrySnapshot> m_latestLauncherSnapshots;

    NmpcTelemetrySnapshot m_latestNmpcSnapshot;
    bool m_hasNmpcSnapshot = false;

    std::atomic<bool> m_running{false};
    std::thread m_broadcastThread;
};


#endif //DASHBOARDSERVER_H