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

#include <thread>
#include <fstream>
#include <sstream>

#include "httplib.h"

class DashboardServer {

public:

    DashboardServer();
    ~DashboardServer();

    void start();
    void stop();

private:

    void run();

    httplib::Server m_server;
    std::thread m_thread;
};


#endif //DASHBOARDSERVER_H