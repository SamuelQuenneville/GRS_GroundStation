/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */
#include <utility>

#include "userInputListener.h"

UserInputListener::UserInputListener(std::function<void(const std::string&)> callback)
    : m_running(false)
    , m_commandCallback(std::move(callback))
{

}

UserInputListener::~UserInputListener() {
    stop();
}

void UserInputListener::start() {
    m_running = true;
    m_inputThread = std::thread(&UserInputListener::m_listen, this);
}

void UserInputListener::stop() {
    m_running = false;
    if (m_inputThread.joinable()) {
        m_inputThread.join();
    }
}

void UserInputListener::m_listen() const {
    std::string command;
    while (m_running) {
        std::cout << "\nSERVER->";
        std::getline(std::cin, command);

        if (!m_running) {
            break;
        }
        if (!command.empty()) {
            m_commandCallback(command);
        }
    }
}
