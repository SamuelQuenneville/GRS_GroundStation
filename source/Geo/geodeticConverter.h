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
#include "Log/programLogger.h"

// Constants defined by the World Geodetic System 1984 (WGS84
constexpr double kSemimajorAxis = 6378137.0;
constexpr double kSemiminorAxis = 6356752.3142;
constexpr double kFirstEccentricitySquared = 6.69437999014 * 0.001;
constexpr double kSecondEccentricitySquared = 6.73949674228 * 0.001;
constexpr double kFlattening = 1.0 / 298.257223563;

class GeodeticConverter {

public:
    GeodeticConverter();
    ~GeodeticConverter() = default;

    [[nodiscard]] bool isInitialized() const;
    void getReference(double& latitudeRadians, double& longitudeRadians, double& altitude) const;

    void initializeReference(double latitudeDegrees, double longitudeDegrees, double altitude);

    static void geodeticToEcef(double latitudeDegrees, double longitudeDegrees, double altitude, double& x, double& y, double& z);
    void ecefToNed(double x, double y, double z, double& north, double& east, double& down) const;
    void geodeticToNed(double latitudeDegrees, double longitudeDegrees, double altitude, double& north, double& east, double& down) const;

private:
    bool m_haveReference;

    double m_latitudeRadiansRef;
    double m_longitudeRadiansRef;
    double m_altitudeRef;

    double m_ecefRefX;
    double m_ecefRefY;
    double m_ecefRefZ;

    grs::Matrix3d m_ecefToNed;
    grs::Matrix3d m_nedToEcef;

    static grs::Matrix3d m_nedToEcefRotation(double latitudeRadians, double longitudeRadians);
};

#endif //GEODETICCONVERTER_H