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

#include "Definitions/communicationStructures.h"
#include "Geo/geodeticConverter.h"


class NavigationFrameManager {

public:
    NavigationFrameManager() = default;
    ~NavigationFrameManager() = default;

    bool isInitialized() const;

    // True once setOrigin() was called; isInitialized() also needs the UAV offsets from initializeOffset().
    bool hasOrigin() const;
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void initializeOffset(std::map<uint8_t, uavStates>& states, bool sitl);
    std::map<uint8_t, uavStates> toNavigationFrame(std::map<uint8_t, uavStates>& states) const;

    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

private:
    // Guards everything below: setOrigin() runs on the console or dashboard
    // thread while the control loop converts states every tick. A torn
    // reference would be baked into an offset that is computed only once.
    mutable std::mutex m_mutex;

    GeodeticConverter m_geodeticConverter;

    std::map<uint8_t, grs::Vec3d> m_uavFrameOffsets;
    bool m_initialized{false};

};

#endif //NAVIGATIONFRAMEMANAGER_H
