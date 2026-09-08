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
    // Guards everything below -- setOrigin() (console thread or the
    // dashboard's "Set origin from payload GPS" HTTP handler thread) races
    // against m_controlLoop() (initializeOffset()/toNavigationFrame(),
    // every tick) with zero synchronization otherwise. GeodeticConverter::
    // initializeReference() writes ~20 doubles before m_haveReference=true,
    // so an unguarded read from the control loop mid-click could see a
    // torn reference and bake a corrupted offset into m_uavFrameOffsets --
    // which then persists, since initializeOffset() now only computes an
    // offset once per uavId.
    mutable std::mutex m_mutex;

    GeodeticConverter m_geodeticConverter;

    std::map<uint8_t, grs::Vec3d> m_uavFrameOffsets;
    bool m_initialized{false};

};

#endif //NAVIGATIONFRAMEMANAGER_H
