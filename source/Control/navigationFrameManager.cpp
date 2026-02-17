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

        std::cout << "Offset: " << offset << std::endl;
    }

    m_initialized = true;
}

void NavigationFrameManager::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    // Convert GPS to NED
    double north, east, down;
    m_geodeticConverter.geodeticToNed(latitudeDegrees, longitudeDegrees, altitude, north, east, down);

    std::cout << north << ", " << east << ", " << down << std::endl;
}

bool NavigationFrameManager::toNavigationFrame(std::map<uint8_t, uavStates>& states) const {

    if (!m_initialized) {
        LOG_ERROR("NavigationFrame is not initialized");
        return false;
    }

    for (auto& [uavId, state] : states) {

        auto it = m_uavFrameOffsets.find(uavId);
        if (it == m_uavFrameOffsets.end()) {
            LOG_ERROR("Missing offset for UAV");
            continue;
        }

        // Apply offset
        grs::Vec3d pos(state.northMeter, state.eastMeter, state.downMeter);
        pos += pos + it->second;

        state.northMeter = static_cast<float>(pos[0]);
        state.eastMeter  = static_cast<float>(pos[1]);
        state.downMeter  = static_cast<float>(pos[2]);
    }

    return true;
}
