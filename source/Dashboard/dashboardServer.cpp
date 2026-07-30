/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "dashboardServer.h"


DashboardServer::DashboardServer() {

}

DashboardServer::~DashboardServer() {
    stop();
}

void DashboardServer::start() {
    m_thread = std::thread(&DashboardServer::run, this);
}

void DashboardServer::stop() {
    m_server.stop();

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void DashboardServer::run() {
    m_server.set_mount_point("/", "./dashboard");
    m_server.listen("localhost", 8080);
}