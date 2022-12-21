#include <math.h>
#include <stdlib.h>
#define PI 3.14159265

class GeoCoordinate
{
public:
    double latitude;
    double longitude;

    GeoCoordinate() {};
    GeoCoordinate(const char *_latitude, const char *_longitude) { latitude = atof(_latitude); longitude = atof(_longitude); }
    GeoCoordinate(double _latitude, double _longitude) {latitude = _latitude * PI / 180; longitude = _longitude * PI / 180; }
};


double getBearing(GeoCoordinate startCoord, GeoCoordinate endCoord);
double getDistance(GeoCoordinate startCoord, GeoCoordinate endCoord);
