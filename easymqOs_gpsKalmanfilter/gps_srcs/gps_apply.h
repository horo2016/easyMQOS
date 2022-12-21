/* To use these functions:

   1. Start with a KalmanFilter created by alloc_filter_velocity2d.
   2. At fixed intervals, call update_velocity2d with the lat/long.
   3. At any time, to get an estimate for the current position,
   use the functions: get_lat_long
 */

#ifndef __GPS_H__
#define __GPS_H__

#include <stdio.h>
#include "gps_kalman.h"
#include "coordinate_sys.h"


// recommended useage
void gps_init(double noise);
void gps_update(double lat, double lon, double seconds_since_last_timestep);
void gps_read(double* lat, double* lon);
double get_distance(double LatFrom, double LonFrom, double LatTo, double LonTo);


// see https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
extern const double PI;
extern const double EARTH;

/* Create a GPS filter that only tracks two dimensions of position and
   velocity.
   The inherent assumption is that changes in velocity are randomly
   distributed around 0.
   Noise is a parameter you can use to alter the expected noise.
   1.0 is the original, and the higher it is, the more a path will be
   "smoothed".
   Free with free_filter after using. */
KalmanFilter alloc_filter_velocity2d(double noise);

/* Set the seconds per timestep in the velocity2d model. */
void set_seconds_per_timestep(KalmanFilter f,
			      double seconds_per_timestep);

/* Update the velocity2d model with new gps data. */
void update_velocity2d(KalmanFilter f, double lat, double lon,
		       double seconds_since_last_update);


/* Extract a lat long from a velocity2d Kalman filter. */
void get_lat_long(KalmanFilter f, double* lat, double* lon);

/* Extract velocity with lat-long-per-second units from a velocity2d
   Kalman filter. */
void get_velocity(KalmanFilter f, double* delta_lat, double* delta_lon);

/* Extract a bearing from a velocity2d Kalman filter.
   0 = north, 90 = east, 180 = south, 270 = west */
double get_bearing(double LatFrom, double LonFrom, double LatTo, double LonTo);

///* Convert a lat, long, delta lat, and delta long into mph.*/
//double calculate_mph(double lat, double lon,
//		     double delta_lat, double delta_lon);
//
///* Extract speed in miles per hour from a velocity2d Kalman filter. */
//double get_mph(KalmanFilter f);

double get_distance(double LatFrom, double LonFrom, double LatTo, double LonTo);
extern Location myGps_filter(double Lat, double Lon,float gpsSpeed,float gpsBearing,float imuHeading,float *fusion_heading);

#endif
