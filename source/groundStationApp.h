/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GROUNDSTATION_H
#define GROUNDSTATION_H

#include "Communication/communicationManager.h"
#include "Control/controlInterface.h"

static constexpr std::string_view DEFAULT_REMOTE_IP = "127.0.0.1";
static constexpr int DEFAULT_TCP_PORT = 5760; // Ardupilot specifics

class GroundStationApp {

public:
    GroundStationApp();
    ~GroundStationApp();

    void start();
    void stop();

    void armAll();
    void setGuidedAll();

    void initMatlabController(const char* ip, uint16_t port) const;
    void startController() const;
    void setControllerFrequency(double frequency) const;

    void parseCommandFile(const std::string& file) const;
    static bool parseUavCommandsLine(const std::string& line, double& time, uavCommands& command, bool& shouldMove, bool& endSimulation);

    std::map<uint8_t, uavStates>  getControllerInput();
    void updateControlOutput(const std::map<uint8_t, uavCommands>& uavCommands);
    void updateControlOutput(const std::map<uint8_t, uavCommands>& uavCommands, const std::map<uint8_t, bool>& shouldMove, const std::map<uint8_t, bool>& endSimulation);

    void setNumberOfUavs(int numUavs);
    [[nodiscard]] int getNumberOfUavs() const;

private:
    CommunicationManager m_communicationManager;

    int m_numberOfUavs = 0;

    void m_run();
    void m_updateState();

    std::thread m_communicationThread;
    std::atomic<bool> m_running;

    std::unique_ptr<ControlInterface> m_controlInterface;
    std::mutex m_dataMutex;
};



#endif //GROUNDSTATION_H
