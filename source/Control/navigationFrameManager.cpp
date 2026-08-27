/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "navigationFrameManager.h"

void NavigationFrameManager::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    m_geodeticConverter.initializeReference(latitudeDegrees, longitudeDegrees, altitude);
}

bool NavigationFrameManager::isInitialized() const {
    return m_initialized;
}

void NavigationFrameManager::initializeOffset(std::map<uint8_t, uavStates>& states, bool sitl) {

    m_uavFrameOffsets.clear();

    if (!m_geodeticConverter.isInitialized()) {
        LOG_ERROR("GeodeticConverter is not initialized");
        return;
    }

    for (const auto& [uavId, state] : states) {

        // Convert GPS to NED
        double north;
        double east;
        double down;
        m_geodeticConverter.geodeticToNed(state.latitudeDegree, state.longitudeDegree, state.altitudeAmslMeter, north, east, down);

        grs::Vec3d posGpsNed(north, east, down);
        grs::Vec3d posEkfNed(state.northMeter, state.eastMeter, state.downMeter);

        grs::Vec3d offset = grs::Vec3d::zeros();
        if (!sitl) {
            offset = posGpsNed - posEkfNed;
        }

        m_uavFrameOffsets.emplace(uavId, offset);

        LOG_INFO("Offset: " + std::to_string(offset[0]) + ", " + std::to_string(offset[1]) + ", " + std::to_string(offset[2]));
    }

    m_initialized = true;
}

void NavigationFrameManager::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    // Convert GPS to NED
    double north, east, down;
    m_geodeticConverter.geodeticToNed(latitudeDegrees, longitudeDegrees, altitude, north, east, down);

    LOG_DEBUG(std::to_string(north) + ", " + std::to_string(east) + ", " + std::to_string(down));
}

std::map<uint8_t, uavStates> NavigationFrameManager::toNavigationFrame(std::map<uint8_t, uavStates>& states) const {

    std::map<uint8_t, uavStates> statesOut{};

    if (!m_initialized) {
        LOG_ERROR("NavigationFrame is not initialized");
        return statesOut;
    }

    for (const auto& [uavId, state] : states) {

        uavStates navState = state;

        // Apply offset
        grs::Vec3d posEkfNed(state.northMeter, state.eastMeter, state.downMeter);
        grs::Vec3d posNavNed = posEkfNed + m_uavFrameOffsets.at(uavId);

        navState.northMeter = static_cast<float>(posNavNed[0]);
        navState.eastMeter  = static_cast<float>(posNavNed[1]);
        navState.downMeter  = static_cast<float>(posNavNed[2]);

        statesOut[uavId] = navState;
    }

    return statesOut;
}
