#include <math.h>
#include "geocoords.h"



/*
class GeoCoordinate
{
};*/

double getBearing(GeoCoordinate startCoord, GeoCoordinate endCoord)
{
    double y = sin(endCoord.longitude - startCoord.longitude) * cos(endCoord.latitude);
    double x = cos(startCoord.latitude) * sin(endCoord.latitude) - sin(startCoord.latitude) * cos(endCoord.latitude) * cos(endCoord.longitude - startCoord.longitude);
    double bearingRadians = atan2(y, x);
    double bearingDegrees = bearingRadians * (180 / PI);
    bearingDegrees = fmod(bearingDegrees + 360, 360);

    return bearingDegrees;
}

double getDistance(GeoCoordinate startCoord, GeoCoordinate endCoord)
{
    double distance = acos(sin(startCoord.latitude) * sin(endCoord.latitude) + cos(startCoord.latitude) * cos(endCoord.latitude) * cos(endCoord.longitude - startCoord.longitude) ) * 6371;

    return distance;
}
