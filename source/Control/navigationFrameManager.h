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

    void setOrigin(const double& latitudeDegrees, const double& longitudeDegrees, const double& altitude);
    void initializeOffset(std::map<uint8_t, uavStates>& states, bool sitl);
    std::map<uint8_t, uavStates> toNavigationFrame(std::map<uint8_t, uavStates>& states) const;

    void debugConvert(double latitudeDegrees, double longitudeDegrees, double altitude) const;

private:
    GeodeticConverter m_geodeticConverter;

    std::map<uint8_t, grs::Vec3d> m_uavFrameOffsets;
    bool m_initialized{false};

};

#endif //NAVIGATIONFRAMEMANAGER_H
