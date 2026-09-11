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
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "httplib.h"
#include "dashboardTypes.h"

/// Embedded HTTP + WebSocket server for the operator dashboard. See
/// docs/Dashboard.md for the REST endpoint table, the snapshot-cache-and-
/// serve threading model, and the handler-injection pattern used below.
class DashboardServer {

public:
    /// @param port Single port serving both static files and the
    ///        WebSocket endpoint (ws(s)://host:port/ws).
    /// @param staticRoot Path to the directory containing index.html/css/js.
    /// @param broadcastRateHz How often buffered telemetry is pushed to
    ///        clients.
    DashboardServer(uint16_t port, std::string staticRoot, int broadcastRateHz = 5);
    ~DashboardServer();

    DashboardServer(const DashboardServer&) = delete;
    DashboardServer& operator=(const DashboardServer&) = delete;

    void start();
    void stop();

    /// Thread-safe. Only updates the buffered snapshot -- the broadcast
    /// thread decides when to actually send it out.
    void updateTelemetry(const UavTelemetrySnapshot& snapshot);
    void updatePayloadTelemetry(const PayloadTelemetrySnapshot& snapshot);
    void updateLauncherTelemetry(const LauncherTelemetrySnapshot& snapshot);
    void updateNmpcTelemetry(const NmpcTelemetrySnapshot& snapshot);
    void setOrigin(const OriginSnapshot& snapshot);
    void setTrajectory(const TrajectorySnapshot& snapshot);

    /// Wires the trajectory-generator sidebar on setup3d.html; see
    /// docs/Dashboard.md's REST endpoint table for what each handler
    /// backs. `generateHandler` must not mutate controller state (pure
    /// preview); `applyHandler` commits, and its return value is sent back
    /// as the response. Both may throw -- caught and returned as a 400
    /// JSON error body.
    void setTrajectoryGeneratorDefaults(const TrajectoryGenerationParams& defaults);
    void setTrajectoryGenerateHandler(std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler);
    void setTrajectoryApplyHandler(std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler);

    /// Backs GET /api/trajectory/live-positions ("Capture live positions").
    /// Computed on demand, not cached/pushed -- only read on an explicit
    /// operator click, not at telemetry rates.
    void setLivePositionsHandler(std::function<LivePositionsSnapshot()> handler);

    /// Backs POST /api/origin/from-payload ("Set origin from payload GPS").
    /// `handler` should capture the payload's current GPS fix, set it as
    /// the NavigationFrameManager origin, and return the resulting
    /// OriginSnapshot; may throw if no payload GPS fix is available yet.
    void setOriginFromPayloadHandler(std::function<OriginSnapshot()> handler);

    /// Backs POST /api/trajectory/save ("Save current trajectory").
    /// `handler` should write whatever's currently loaded in the NMPC
    /// controller to disk and return the path written to; may throw if
    /// there's nothing to save or the write fails.
    void setSaveTrajectoryHandler(std::function<std::string()> handler);

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

    OriginSnapshot m_originSnapshot;
    TrajectorySnapshot m_trajectorySnapshot;
    bool m_hasTrajectorySnapshot = false;

    TrajectoryGenerationParams m_trajectoryGeneratorDefaults;
    std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> m_trajectoryGenerateHandler;
    std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> m_trajectoryApplyHandler;
    std::function<LivePositionsSnapshot()> m_livePositionsHandler;
    std::function<OriginSnapshot()> m_originFromPayloadHandler;
    std::function<std::string()> m_saveTrajectoryHandler;

    std::atomic<bool> m_running{false};
    std::thread m_broadcastThread;
};


#endif //DASHBOARDSERVER_H