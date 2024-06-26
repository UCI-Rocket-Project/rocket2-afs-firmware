#include "coordHelpers.h"

#include <cmath>
#define M_PI 3.14159265358979323

// Constants for WGS84 ellipsoid
const double WGS84_A = 6378137.0;  // semi-major axis in meters
const double WGS84_E = 0.081819190842622;
const double WGS84_INV_FLATTENING = 298.257223563;  // inverse flattening

// Convert degrees to radians
double Deg2Rad(double deg) { return deg * M_PI / 180.0; }

// Convert radians to degrees
double Rad2Deg(double rad) { return rad * 180.0 / M_PI; }

// Convert GPS coordinates (latitude, longitude, altitude) to ECEF coordinates
void Gps2Ecef(double lat, double lon, double alt, double& x, double& y, double& z) {
    double clat = cos(lat * M_PI / 180.0);
    double slat = sin(lat * M_PI / 180.0);
    double clon = cos(lon * M_PI / 180.0);
    double slon = sin(lon * M_PI / 180.0);

    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);

    x = (N + alt) * clat * clon;
    y = (N + alt) * clat * slon;
    z = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
}

// Convert NED (North-East-Down) velocity to ECEF velocity
void Ned2EcefVel(double lat, double lon, double alt, double vn, double ve, double vd, double& vx, double& vy, double& vz) {
    double x, y, z;
    Gps2Ecef(lat, lon, alt, x, y, z);

    // Convert NED velocity to ECEF velocity
    vx = -ve * sin(Deg2Rad(lon)) - vn * sin(Deg2Rad(lat)) * cos(Deg2Rad(lon)) + vd * cos(Deg2Rad(lat)) * cos(Deg2Rad(lon));
    vy = ve * cos(Deg2Rad(lon)) - vn * sin(Deg2Rad(lat)) * sin(Deg2Rad(lon)) + vd * cos(Deg2Rad(lat)) * sin(Deg2Rad(lon));
    vz = vn * cos(Deg2Rad(lat)) + vd * sin(Deg2Rad(lat));
}
