/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GCS_H
#define GCS_H

#include "Communication/communicationManager.h"
#include "Control/controlInterface.h"
#include "Control/controlDispatcher.h"
#include "gcsConfig.h"

class GroundControlStation {

public:
    GroundControlStation();
    ~GroundControlStation();

    void initialize(const gcsConfig& config);

    void start();
    void stop();

    void connectAll();
    void armAll() const;
    void setModeAll(const std::string& mode) const;
    void startController() const;
    void initLaunch() const;
    void fetchParam(int sysId) const;

private:
    gcsConfig m_gcsConfig;

    std::unique_ptr<CommunicationManager> m_communicationManager;
    std::unique_ptr<ControlDispatcher>    m_controlDispatcher;
    std::unique_ptr<ControlInterface>     m_controlInterface;

    void m_parseCommandFile(const std::string& file) const;
    static bool m_parseUavCommandsLine(const std::string& line, uavCommandsFlags& commands);

    void m_supervisorLoop();
    std::thread m_supervisorThread;
    std::atomic<bool> m_running;
};

#endif //GCS_H
