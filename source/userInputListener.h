/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef USERINPUTLISTENER_H
#define USERINPUTLISTENER_H

#include <thread>
#include <atomic>
#include <functional>

#include "groundStationApp.h"

class UserInputListener {

public:
    UserInputListener(std::function<void(const std::string&)> callback);
    ~UserInputListener();

    void start();
    void stop();

private:
    void m_listen() const; // Function that runs in the thread
    std::thread m_inputThread;
    std::atomic<bool> m_running;
    std::function<void(const std::string&)> m_commandCallback;
};

#endif //USERINPUTLISTENER_H
