/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GEODETICCONVERTER_H
#define GEODETICCONVERTER_H

#include "Mathematics/math.h"

// World Geodetic System 1984 (WGS84) ellipsoid
constexpr double kSemimajorAxis = 6378137.0;
constexpr double kFirstEccentricitySquared = 6.69437999014 * 0.001;

class GeodeticConverter {

public:
    GeodeticConverter() = default;

    [[nodiscard]] bool isInitialized() const;
    void getReference(double& latitudeRadians, double& longitudeRadians, double& altitude) const;

    void initializeReference(double latitudeDegrees, double longitudeDegrees, double altitude);

    static void geodeticToEcef(double latitudeDegrees, double longitudeDegrees, double altitude, double& x, double& y, double& z);
    void ecefToNed(double x, double y, double z, double& north, double& east, double& down) const;
    void geodeticToNed(double latitudeDegrees, double longitudeDegrees, double altitude, double& north, double& east, double& down) const;

private:
    bool m_haveReference = false;

    double m_latitudeRadiansRef = 0.0;
    double m_longitudeRadiansRef = 0.0;
    double m_altitudeRef = 0.0;

    double m_ecefRefX = 0.0;
    double m_ecefRefY = 0.0;
    double m_ecefRefZ = 0.0;

    grs::Matrix3d m_ecefToNeu; // ECEF to North-East-Up at the reference

    // Rows North, East, Up of the tangent plane at a geodetic latitude/longitude.
    static grs::Matrix3d m_ecefToNeuRotation(double latitudeRadians, double longitudeRadians);
};

#endif //GEODETICCONVERTER_H