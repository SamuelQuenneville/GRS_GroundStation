/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "geodeticConverter.h"

GeodeticConverter::GeodeticConverter()
    : m_haveReference(false)
{
    // Could be a real coordinate, but here it required an initialization
    m_latitudeRadiansRef  = 0.0;
    m_longitudeRadiansRef = 0.0;
    m_altitudeRef         = 0.0;

    m_ecefRefX = 0.0;
    m_ecefRefY = 0.0;
    m_ecefRefZ = 0.0;
}

bool GeodeticConverter::isInitialized() const {
    return m_haveReference;
}

void GeodeticConverter::getReference(double& latitudeRadians, double& longitudeRadians, double& altitude) const {
    latitudeRadians  = m_latitudeRadiansRef;
    longitudeRadians = m_longitudeRadiansRef;
    altitude         = m_altitudeRef;
}

void GeodeticConverter::initializeReference(const double latitudeDegrees, const double longitudeDegrees, const double altitude) {

    // Save NED origin
    m_latitudeRadiansRef  = grs::degToRad(latitudeDegrees);
    m_longitudeRadiansRef = grs::degToRad(longitudeDegrees);
    m_altitudeRef         = altitude;

    // Compute ECEF of NED origin
    geodeticToEcef(latitudeDegrees, longitudeDegrees, altitude, m_ecefRefX, m_ecefRefY, m_ecefRefZ);

    // Compute ECEF to NED and NED to ECEF matrices
    const double phiP = std::atan2(m_ecefRefZ, std::sqrt(std::pow(m_ecefRefX, 2) + std::pow(m_ecefRefY, 2)));

    m_ecefToNed = m_nedToEcefRotation(phiP, m_longitudeRadiansRef);
    m_nedToEcef = m_nedToEcefRotation(m_latitudeRadiansRef, m_longitudeRadiansRef).transpose();

    m_haveReference = true;
}

void GeodeticConverter::geodeticToEcef(const double latitudeDegrees, const double longitudeDegrees, const double altitude, double& x, double& y, double& z) {

    // Convert geodetic coordinates to ECEF.
    const double latitudeRadians = grs::degToRad(latitudeDegrees);
    const double longitudeRadians = grs::degToRad(longitudeDegrees);

    const double sLat = std::sin(latitudeRadians);
    const double cLat = std::cos(latitudeRadians);
    const double sLon = std::sin(longitudeRadians);
    const double cLon = std::cos(longitudeRadians);

    const double xi = sqrt(1 - kFirstEccentricitySquared * sLat * sLat);

    x = (kSemimajorAxis / xi + altitude) * cLat * cLon;
    y = (kSemimajorAxis / xi + altitude) * cLat * sLon;
    z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sLat;
}

void GeodeticConverter::ecefToNed(const double x, const double y, const double z, double& north, double& east, double& down) const {
    // Converts ECEF coordinate position into local-tangent-plane NED.
    // Coordinates relative to given ECEF coordinate frame.

    grs::Vec3d vec = grs::Vec3d::zeros();
    vec[0] = x - m_ecefRefX;
    vec[1] = y - m_ecefRefY;
    vec[2] = z - m_ecefRefZ;

    grs::Vec3d res = m_ecefToNed * vec;
    north = res[0];
    east  = res[1];
    down  = -res[2];
}

void GeodeticConverter::geodeticToNed(const double latitudeDegrees, const double longitudeDegrees, const double altitude, double& north, double& east, double& down) const {

    // Geodetic position to local NED frame
    if (!isInitialized()) {
        LOG_ERROR("GeodeticConverter is not initialized");
        return;
    }

    double x, y, z;
    geodeticToEcef(latitudeDegrees, longitudeDegrees, altitude, x, y, z);
    ecefToNed(x, y, z, north, east, down);
}

grs::Matrix3d GeodeticConverter::m_nedToEcefRotation(const double latitudeRadians, const double longitudeRadians) {

    const double sLat = std::sin(latitudeRadians);
    const double cLat = std::cos(latitudeRadians);
    const double sLon = std::sin(longitudeRadians);
    const double cLon = std::cos(longitudeRadians);

    grs::Matrix3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
}
