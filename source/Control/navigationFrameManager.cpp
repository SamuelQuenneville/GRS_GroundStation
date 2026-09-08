/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "navigationFrameManager.h"

void NavigationFrameManager::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    std::lock_guard lock(m_mutex);
    m_geodeticConverter.initializeReference(latitudeDegrees, longitudeDegrees, altitude);
}

bool NavigationFrameManager::isInitialized() const {
    std::lock_guard lock(m_mutex);
    return m_initialized;
}

// navigationFrameManager.cpp
bool NavigationFrameManager::hasOrigin() const {
    std::lock_guard lock(m_mutex);
    return m_geodeticConverter.isInitialized();
}

bool NavigationFrameManager::getOrigin(double& latitudeDegrees, double& longitudeDegrees, double& altitude) const {
    std::lock_guard lock(m_mutex);
    if (!m_geodeticConverter.isInitialized()) return false;
    double latRad, lonRad;
    m_geodeticConverter.getReference(latRad, lonRad, altitude);
    latitudeDegrees = grs::radToDeg(latRad);
    longitudeDegrees = grs::radToDeg(lonRad);
    return true;
}

void NavigationFrameManager::initializeOffset(std::map<uint8_t, uavStates>& states, bool sitl) {
    std::lock_guard lock(m_mutex);

    if (!m_geodeticConverter.isInitialized()) {
        LOG_ERROR("GeodeticConverter is not initialized");
        return;
    }

    // Incremental, not a one-shot snapshot: MAVSDK connects each system
    // (UAV, payload) on its own async timeline, so latestStates grows one
    // sysId at a time across control-loop ticks. A uavId already holding an
    // offset is left untouched -- a system connecting late must not reset
    // everyone else's offset -- and a uavId not seen before gets one
    // computed now. Called every tick (see m_controlLoop) so a system that
    // connects well after the first one still gets an offset instead of
    // never getting one.
    for (const auto& [uavId, state] : states) {
        if (m_uavFrameOffsets.contains(uavId)) continue;

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

        LOG_INFO("Offset for sysId " + std::to_string(uavId) + ": " + std::to_string(offset[0]) + ", " + std::to_string(offset[1]) + ", " + std::to_string(offset[2]));
    }

    if (!m_uavFrameOffsets.empty()) {
        m_initialized = true;
    }
}

void NavigationFrameManager::debugConvert(const double latitudeDegrees, const double longitudeDegrees, const double altitude) const {
    std::lock_guard lock(m_mutex);

    // Convert GPS to NED
    double north, east, down;
    m_geodeticConverter.geodeticToNed(latitudeDegrees, longitudeDegrees, altitude, north, east, down);

    LOG_DEBUG(std::to_string(north) + ", " + std::to_string(east) + ", " + std::to_string(down));
}

std::map<uint8_t, uavStates> NavigationFrameManager::toNavigationFrame(std::map<uint8_t, uavStates>& states) const {
    std::lock_guard lock(m_mutex);

    std::map<uint8_t, uavStates> statesOut{};

    if (!m_initialized) {
        LOG_ERROR("NavigationFrame is not initialized");
        return statesOut;
    }

    for (const auto& [uavId, state] : states) {

        const auto offsetIt = m_uavFrameOffsets.find(uavId);
        if (offsetIt == m_uavFrameOffsets.end()) {
            // This system's offset hasn't been computed yet (it connected
            // too recently -- initializeOffset() runs earlier this same
            // tick, but a system whose first telemetry sample arrives in
            // between could still be missed by one tick). Skip it now
            // rather than .at()-throwing and taking down the control loop
            // thread; it'll be included starting next tick.
            continue;
        }

        uavStates navState = state;

        // Apply offset
        grs::Vec3d posEkfNed(state.northMeter, state.eastMeter, state.downMeter);
        grs::Vec3d posNavNed = posEkfNed + offsetIt->second;

        navState.northMeter = static_cast<float>(posNavNed[0]);
        navState.eastMeter  = static_cast<float>(posNavNed[1]);
        navState.downMeter  = static_cast<float>(posNavNed[2]);

        statesOut[uavId] = navState;
    }

    return statesOut;
}
