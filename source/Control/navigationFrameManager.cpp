/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "navigationFrameManager.h"

#include <cmath>
#include <utility>

namespace {
// Global vs local position through the offset, vehicle still [m]. Both come
// from the same EKF, so only message timing and rounding remain.
constexpr double kOffsetTolerance = 1.0;
}

void NavigationFrameManager::setOrigin(const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    std::lock_guard lock(m_mutex);
    m_geodeticConverter.initializeReference(latitudeDegrees, longitudeDegrees, altitude);
    if (!m_uavFrameOffsets.empty()) {
        LOG_WARNING("Origin changed: frame offsets recomputed");
    }
    m_uavFrameOffsets.clear();
    m_offsetDeferred.clear();
    m_offsetResidual.clear();
    m_initialized = false;
}

void NavigationFrameManager::setEkfOrigin(const uint8_t sysId, const double latitudeDegrees, const double longitudeDegrees, const double altitude) {
    std::lock_guard lock(m_mutex);
    const auto it = m_ekfOrigins.find(sysId);
    if (it != m_ekfOrigins.end() && std::abs(it->second.latitudeDegrees - latitudeDegrees) < 1e-8
        && std::abs(it->second.longitudeDegrees - longitudeDegrees) < 1e-8 && std::abs(it->second.altitude - altitude) < 0.01) {
        return; // repeated
    }
    if (it != m_ekfOrigins.end()) {
        LOG_WARNING("sysId " + std::to_string(sysId) + ": EKF origin changed, frame offset recomputed");
    }
    LOG_INFO("sysId " + std::to_string(sysId) + ": EKF origin lat=" + std::to_string(latitudeDegrees) + ", lon="
             + std::to_string(longitudeDegrees) + ", alt=" + std::to_string(altitude));
    m_ekfOrigins[sysId] = {latitudeDegrees, longitudeDegrees, altitude};
    m_uavFrameOffsets.erase(sysId);
    m_offsetResidual.erase(sysId);
}

bool NavigationFrameManager::isInitialized() const {
    std::lock_guard lock(m_mutex);
    return m_initialized;
}

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

void NavigationFrameManager::initializeOffset(const std::map<uint8_t, uavStates>& states) {
    std::lock_guard lock(m_mutex);

    if (!m_geodeticConverter.isInitialized()) {
        LOG_ERROR("GeodeticConverter is not initialized");
        return;
    }

    for (const auto& [uavId, state] : states) {
        if (!m_uavFrameOffsets.contains(uavId)) {
            // The offset is the vehicle's EKF origin in the GCS frame: exact,
            // unlike a difference of two messages sampled at different times.
            const auto origin = m_ekfOrigins.find(uavId);
            if (origin == m_ekfOrigins.end()) {
                if (std::exchange(m_offsetDeferred[uavId], 1) != 1) {
                    LOG_WARNING("sysId " + std::to_string(uavId) + ": waiting for its EKF origin (GPS_GLOBAL_ORIGIN), frame offset deferred");
                }
                continue;
            }
            grs::Vec3d offset = grs::Vec3d::zeros();
            m_geodeticConverter.geodeticToNed(origin->second.latitudeDegrees, origin->second.longitudeDegrees,
                                              origin->second.altitude, offset[0], offset[1], offset[2]);

            // EKF origin (boot point) and the GCS origin (anchor) are on the same
            // field: kilometers mean a wrong origin.
            constexpr double kMaxOffset = 5000.0; // [m]
            const double offsetNorm = std::sqrt(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);
            if (!(offsetNorm < kMaxOffset)) {
                if (std::exchange(m_offsetDeferred[uavId], 2) != 2) {
                    LOG_ERROR("sysId " + std::to_string(uavId) + ": frame offset of " + std::to_string(offsetNorm)
                              + " m refused (wrong GCS or EKF origin)");
                }
                continue;
            }
            m_offsetDeferred.erase(uavId);
            m_uavFrameOffsets.emplace(uavId, offset);
            LOG_INFO("Offset for sysId " + std::to_string(uavId) + ": " + std::to_string(offset[0]) + ", "
                     + std::to_string(offset[1]) + ", " + std::to_string(offset[2]));
        }

        // Check: the global position must equal the local one plus the offset.
        // Only while slow (< 1 m/s): GLOBAL_POSITION_INT and LOCAL_POSITION_NED
        // arrive separately, so in flight their sampling times differ.
        const double speed = std::hypot(state.northMeterSecond, state.eastMeterSecond, state.downMeterSecond);
        if (!(speed < 1.0) || !std::isfinite(state.latitudeDegree) || !std::isfinite(state.longitudeDegree)
            || (state.latitudeDegree == 0.0 && state.longitudeDegree == 0.0)) {
            continue;
        }
        double north, east, down;
        m_geodeticConverter.geodeticToNed(state.latitudeDegree, state.longitudeDegree, state.altitudeAmslMeter, north, east, down);
        const grs::Vec3d& offset = m_uavFrameOffsets.at(uavId);
        const double residual = std::hypot(north - (state.northMeter + offset[0]), east - (state.eastMeter + offset[1]),
                                           down - (state.downMeter + offset[2]));
        const auto previous = m_offsetResidual.find(uavId);
        const bool wasOk = previous != m_offsetResidual.end() && previous->second <= kOffsetTolerance;
        if (!(residual <= kOffsetTolerance) && (previous == m_offsetResidual.end() || wasOk)) {
            LOG_ERROR("sysId " + std::to_string(uavId) + ": global and local positions disagree by " + std::to_string(residual)
                      + " m through the frame offset (tolerance " + std::to_string(kOffsetTolerance) + " m)");
        } else if (residual <= kOffsetTolerance && previous != m_offsetResidual.end() && !wasOk) {
            LOG_INFO("sysId " + std::to_string(uavId) + ": frame offset consistent again (" + std::to_string(residual) + " m)");
        }
        m_offsetResidual[uavId] = residual;
    }

    m_initialized = !m_uavFrameOffsets.empty();
}

bool NavigationFrameManager::frameReady(const int numUavs, std::string& reason) const {
    std::lock_guard lock(m_mutex);
    for (int id = 1; id <= numUavs; ++id) {
        const auto sysId = static_cast<uint8_t>(id);
        const auto residual = m_offsetResidual.find(sysId);
        if (!m_uavFrameOffsets.contains(sysId)) {
            reason = "sysId " + std::to_string(id) + " has no frame offset (GCS origin or EKF origin missing)";
        } else if (residual == m_offsetResidual.end()) {
            reason = "sysId " + std::to_string(id) + " frame offset not checked yet (needs a global position while still)";
        } else if (!(residual->second <= kOffsetTolerance)) {
            reason = "sysId " + std::to_string(id) + " global and local positions disagree by "
                     + std::to_string(residual->second) + " m through the frame offset";
        } else {
            continue;
        }
        return false;
    }
    return true;
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
            // Connected after this tick's initializeOffset(): included next tick.
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
