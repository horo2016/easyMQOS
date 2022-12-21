#ifndef __PARSEGPS_H__
#define __PARSEGPS_H__

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

bool gpsparser(char* data, double* lon, double* lat, double* HDOP, int* numSV);


#endif
