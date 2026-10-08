/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef NAVIGATIONFRAMEMANAGER_H
#define NAVIGATIONFRAMEMANAGER_H

#include <map>
#include <mutex>
#include <string>

#include "Definitions/communicationStructures.h"
#include "Geo/geodeticConverter.h"

class NavigationFrameManager {

public:
    NavigationFrameManager() = default;
    ~NavigationFrameManager() = default;

    bool isInitialized() const;

    // False before setOrigin(); isInitialized() also needs the offsets from initializeOffset().
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    // GCS origin (the tether anchor). Clears every offset: they are relative to it.
    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    // A vehicle's EKF origin (GPS_GLOBAL_ORIGIN, altitude AMSL), the point its
    // LOCAL_POSITION_NED is relative to. A changed origin clears its offset.
    void setEkfOrigin(uint8_t sysId, double latitudeDegrees, double longitudeDegrees, double altitude);
    // Every tick: computes the offset of vehicles that have none yet (their EKF
    // origin in the GCS frame) and checks every offset against the vehicle's
    // global position while it is slow.
    void initializeOffset(const std::map<uint8_t, uavStates>& states);
    // UAVs 1..numUavs have an offset that passed the check; reason says why not.
    bool frameReady(int numUavs, std::string& reason) const;
    std::map<uint8_t, uavStates> toNavigationFrame(std::map<uint8_t, uavStates>& states) const;

    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

private:
    // Guards everything below: setOrigin() runs on the console or dashboard
    // thread while the control loop converts states every tick. A torn
    // reference would be baked into an offset that is computed only once.
    mutable std::mutex m_mutex;

    GeodeticConverter m_geodeticConverter;

    struct Geodetic {
        double latitudeDegrees, longitudeDegrees, altitude;
    };
    std::map<uint8_t, Geodetic> m_ekfOrigins;
    std::map<uint8_t, grs::Vec3d> m_uavFrameOffsets;
    // Why each sysId's offset is still deferred (1 no EKF origin, 2 offset too large), to log each reason once.
    std::map<uint8_t, int> m_offsetDeferred;
    // Last check of each offset: |global position - (local position + offset)| [m].
    std::map<uint8_t, double> m_offsetResidual;
    bool m_initialized{false};
    bool m_noOriginLogged{false}; // "no GCS origin" logged once until setOrigin()

};

#endif //NAVIGATIONFRAMEMANAGER_H
