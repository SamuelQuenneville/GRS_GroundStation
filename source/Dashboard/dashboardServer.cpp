/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "dashboardServer.h"

DashboardServer::DashboardServer(const uint16_t port, std::string staticRoot, const int broadcastRateHz)
    : m_port(port),
      m_staticRoot(std::move(staticRoot)),
      m_broadcastRateHz(broadcastRateHz),
      m_httpServer(std::make_unique<httplib::Server>())
{

}

DashboardServer::~DashboardServer() {
    stop();
}

void DashboardServer::start() {
    if (m_running.exchange(true)) return;

    m_httpServer->Get("/api/origin", [this](const httplib::Request&, httplib::Response& res) {
        OriginSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            snapshot = m_originSnapshot;
        }
        res.set_content(snapshot.toJson(), "application/json");
    });

    m_httpServer->Get("/api/trajectory", [this](const httplib::Request&, httplib::Response& res) {
        TrajectorySnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            if (m_hasTrajectorySnapshot) snapshot = m_trajectorySnapshot;
        }
        res.set_content(snapshot.toJson(), "application/json");
    });

    m_httpServer->Get("/api/trajectory/generator-defaults", [this](const httplib::Request&, httplib::Response& res) {
        TrajectoryGenerationParams defaults;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            defaults = m_trajectoryGeneratorDefaults;
        }
        res.set_content(defaults.toJson(), "application/json");
    });

    m_httpServer->Post("/api/trajectory/generate", [this](const httplib::Request& req, httplib::Response& res) {
        std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            handler = m_trajectoryGenerateHandler;
        }
        if (!handler) {
            res.status = 503;
            res.set_content(JsonWriter().add("error", "trajectory generator not available").str(), "application/json");
            return;
        }
        try {
            const auto params = TrajectoryGenerationParams::fromJson(req.body);
            res.set_content(handler(params).toJson(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(JsonWriter().add("error", std::string(e.what())).str(), "application/json");
        }
    });

    m_httpServer->Post("/api/trajectory/apply", [this](const httplib::Request& req, httplib::Response& res) {
        std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            handler = m_trajectoryApplyHandler;
        }
        if (!handler) {
            res.status = 503;
            res.set_content(JsonWriter().add("error", "trajectory generator not available").str(), "application/json");
            return;
        }
        try {
            const auto params = TrajectoryGenerationParams::fromJson(req.body);
            res.set_content(handler(params).toJson(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(JsonWriter().add("error", std::string(e.what())).str(), "application/json");
        }
    });

    m_httpServer->Get("/api/trajectory/live-positions", [this](const httplib::Request&, httplib::Response& res) {
        std::function<LivePositionsSnapshot()> handler;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            handler = m_livePositionsHandler;
        }
        if (!handler) {
            res.set_content(LivePositionsSnapshot{}.toJson(), "application/json"); // available=false
            return;
        }
        res.set_content(handler().toJson(), "application/json");
    });

    m_httpServer->set_mount_point("/", m_staticRoot);

    m_httpServer->set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-cache");
    });

    m_httpServer->WebSocket("/ws", [this](const httplib::Request&, httplib::ws::WebSocket& ws) {
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            m_clients.push_back(&ws);
        }

        std::string msg;
        while (ws.read(msg)) {
            // Dashboard telemetry is server -> browser only for now, this is
            // where dashboard-issued commands (e.g. arm/disarm buttons) would come in.
        }

        std::lock_guard<std::mutex> lock(m_clientsMutex);
        std::erase(m_clients, &ws);
    });

    // httplib's listen() blocks for the life of the server, so run it on
    // its own thread. Binds to loopback only: this dashboard has no
    // authentication, so it shouldn't be reachable from other machines.
    // Point this at "0.0.0.0" instead if you want LAN access.
    m_serverThread = std::thread([this]() { m_httpServer->listen("localhost", m_port); });

    m_broadcastThread = std::thread(&DashboardServer::m_broadcastLoop, this);
}

void DashboardServer::stop() {
    if (!m_running.exchange(false)) return;

    if (m_broadcastThread.joinable()) {
        m_broadcastThread.join();
    }

    // Close any open browser connections before tearing down the listener,
    // so their handler threads unblock from ws.read() promptly.
    std::vector<httplib::ws::WebSocket*> openClients;
    {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        openClients = m_clients;
    }

    for (auto* ws : openClients) {
        ws->close(httplib::ws::CloseStatus::GoingAway, "server shutting down");
    }

    m_httpServer->stop();
    if (m_serverThread.joinable()) {
        m_serverThread.join();
    }
}

void DashboardServer::updateTelemetry(const UavTelemetrySnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_latestSnapshots[snapshot.id] = snapshot;
}

void DashboardServer::updatePayloadTelemetry(const PayloadTelemetrySnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_latestPayloadSnapshot = snapshot;
    m_hasPayloadSnapshot = true;
}

void DashboardServer::updateLauncherTelemetry(const LauncherTelemetrySnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_latestLauncherSnapshots[snapshot.id] = snapshot;
}

void DashboardServer::updateNmpcTelemetry(const NmpcTelemetrySnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_latestNmpcSnapshot = snapshot;
    m_hasNmpcSnapshot = true;
}

void DashboardServer::setOrigin(const OriginSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_originSnapshot = snapshot;
}

void DashboardServer::setTrajectory(const TrajectorySnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_trajectorySnapshot = snapshot;
    m_hasTrajectorySnapshot = true;
}

void DashboardServer::setTrajectoryGeneratorDefaults(const TrajectoryGenerationParams& defaults) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_trajectoryGeneratorDefaults = defaults;
}

void DashboardServer::setTrajectoryGenerateHandler(std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_trajectoryGenerateHandler = std::move(handler);
}

void DashboardServer::setTrajectoryApplyHandler(std::function<TrajectorySnapshot(const TrajectoryGenerationParams&)> handler) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_trajectoryApplyHandler = std::move(handler);
}

void DashboardServer::setLivePositionsHandler(std::function<LivePositionsSnapshot()> handler) {
    std::lock_guard<std::mutex> lock(m_snapshotsMutex);
    m_livePositionsHandler = std::move(handler);
}

size_t DashboardServer::connectedBrowserCount() const {
    std::lock_guard<std::mutex> lock(m_clientsMutex);
    return m_clients.size();
}

void DashboardServer::m_broadcastLoop() {
    const auto period = std::chrono::milliseconds(1000 / std::max(1, m_broadcastRateHz));

    while (m_running) {
        std::vector<std::string> payloads;
        {
            std::lock_guard<std::mutex> lock(m_snapshotsMutex);
            payloads.reserve(m_latestSnapshots.size());
            for (const auto& [id, snapshot] : m_latestSnapshots) {
                (void)id;
                payloads.push_back(snapshot.toJson());
            }

            if (m_hasPayloadSnapshot) {
                payloads.push_back(m_latestPayloadSnapshot.toJson());
            }

            for (const auto& [id, snapshot] : m_latestLauncherSnapshots) {
                (void)id;
                payloads.push_back(snapshot.toJson());
            }

            if (m_hasNmpcSnapshot) {
                payloads.push_back(m_latestNmpcSnapshot.toJson());
            }
        }

        if (!payloads.empty()) {
            std::vector<httplib::ws::WebSocket*> targets;
            {
                std::lock_guard<std::mutex> lock(m_clientsMutex);
                targets = m_clients;
            }

            for (auto* ws : targets) {
                for (const auto& json : payloads) ws->send(json);
            }
        }

        std::this_thread::sleep_for(period);
    }
}