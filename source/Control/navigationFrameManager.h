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

#include "Definitions/communicationStructures.h"
#include "Geo/geodeticConverter.h"


class NavigationFrameManager {

public:
    NavigationFrameManager() = default;
    ~NavigationFrameManager() = default;

    bool isInitialized() const;

    // True once setOrigin() has established a geodetic reference -- distinct
    // from isInitialized() above, which additionally requires
    // initializeOffset() to have run against live UAV states. This is what
    // "is there an origin to show" means for setup/orientation tooling: an
    // operator who just set an origin should see it immediately.
    bool hasOrigin() const;
    bool getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const;

    void setOrigin(double latitudeDegrees, double longitudeDegrees, double altitude);
    void initializeOffset(std::map<uint8_t, uavStates>& states, bool sitl);
    std::map<uint8_t, uavStates> toNavigationFrame(std::map<uint8_t, uavStates>& states) const;

    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

private:
    GeodeticConverter m_geodeticConverter;

    std::map<uint8_t, grs::Vec3d> m_uavFrameOffsets;
    bool m_initialized{false};

};

#endif //NAVIGATIONFRAMEMANAGER_H
