/* Applying Kalman filters to GPS data. */

#include <math.h>
#include "gps_apply.h"
#include "coordinate_sys.h"

const double EARTHR = 6371000.8;

const double PI = 3.141592653589793;

KalmanFilter ff;


// interface for serial
void gps_init(double noise){
  ff = alloc_filter_velocity2d(noise);
}

void gps_update(double lat, double lon, double seconds_since_last_timestep){
  update_velocity2d(ff, lat, lon, seconds_since_last_timestep);
}

void gps_read(double* lat, double* lon){
  get_lat_long(ff, lat, lon);
}


KalmanFilter alloc_filter_velocity2d(double noise) {
  /* The state model has four dimensions:
     x, y, x', y'
     Each time step we can only observe position, not velocity, so the
     observation vector has only two dimensions.
  */
  KalmanFilter f = alloc_filter(4, 2);

  /* Assuming the axes are rectilinear does not work well at the
     poles, but it has the bonus that we don't need to convert between
     lat/long and more rectangular coordinates. The slight inaccuracy
     of our physics model is not too important.
   */
  //double v2p = 0.001;//never used
  set_identity_matrix(f.state_transition);
  set_seconds_per_timestep(f, 1.0);
	     
  /* We observe (x, y) in each time step */
  set_matrix(f.observation_model,
	     1.0, 0.0, 0.0, 0.0,
	     0.0, 1.0, 0.0, 0.0);

  /* Noise in the world. */
  double pos = 0.000001;
  set_matrix(f.process_noise_covariance,
	     pos, 0.0, 0.0, 0.0,
	     0.0, pos, 0.0, 0.0,
	     0.0, 0.0, 1.0, 0.0,
	     0.0, 0.0, 0.0, 1.0);

  /* Noise in our observation */
  set_matrix(f.observation_noise_covariance,
	     pos * noise, 0.0,
	     0.0, pos * noise);

  /* The start position is totally unknown, so give a high variance */
  set_matrix(f.state_estimate, 0.0, 0.0, 0.0, 0.0);
  set_identity_matrix(f.estimate_covariance);
  const double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
  scale_matrix(f.estimate_covariance, trillion);

  return f;
}

/* The position units are in thousandths of latitude and longitude.
   The velocity units are in thousandths of position units per second.

   So if there is one second per timestep, a velocity of 1 will change
   the lat or long by 1 after a million timesteps.

   Thus a typical position is hundreds of thousands of units.
   A typical velocity is maybe ten.
*/
void set_seconds_per_timestep(KalmanFilter f,
			      double seconds_per_timestep) {
  /* unit_scaler accounts for the relation between position and
     velocity units */
  double unit_scaler = 0.001;
  f.state_transition.data[0][2] = unit_scaler * seconds_per_timestep;
  f.state_transition.data[1][3] = unit_scaler * seconds_per_timestep;
}

void update_velocity2d(KalmanFilter f, double lat, double lon,
		       double seconds_since_last_timestep) {
  set_seconds_per_timestep(f, seconds_since_last_timestep);
  set_matrix(f.observation, lat * 1000.0, lon * 1000.0);
  update(f);
}

//int read_lat_long(FILE* file, double* lat, double* lon) {
//  while (1) {
//    /* If we find a lat long pair, we're done */
//    if (2 == fscanf(file, "%lf,%lf", lat, lon)) {
//      return 1;
//    }
//
//    /* Advance to the next line */
//    int ch;
//    while ((ch = getc(file)) != '\n') {
//      if (EOF == ch) {
//	return 0;
//      }
//    }
//  }
//}


void get_lat_long(KalmanFilter f, double* lat, double* lon) {
  *lat = f.state_estimate.data[0][0] / 1000.0;
  *lon = f.state_estimate.data[1][0] / 1000.0;
}


void get_velocity(KalmanFilter f, double* delta_lat, double* delta_lon) {
  *delta_lat = f.state_estimate.data[2][0] / (1000.0 * 1000.0);
  *delta_lon = f.state_estimate.data[3][0] / (1000.0 * 1000.0);
}
/* See
   http://www.movable-type.co.uk/scripts/latlong.html
   for formulas */
double get_bearing(double LatFrom, double LonFrom, double LatTo, double LonTo)
{
  double lat, lon, delta_lon, x, y,lat_end;
   

  /* Convert to radians */
  double to_radians = PI / 180.0;
  lat  =LatFrom* to_radians;
  lat_end = LatTo* to_radians;
  
  lon  =LonFrom* to_radians;
  delta_lon= LonTo* to_radians; -LonFrom* to_radians;
   
   
  
  /* Do math */
   
  y = sin(delta_lon) * cos(lat_end);
  x = cos(lat) * sin(lat_end) - sin(lat) * cos(lat_end) * cos(delta_lon);
  double bearing = atan2(y, x);

  /* Convert to degrees */
  bearing = bearing / to_radians;
  while (bearing >= 360.0) {
    bearing -= 360.0;
  }
  while (bearing < 0.0) {
    bearing += 360.0;
  }
    
  return bearing;
}
/* See
   http://www.movable-type.co.uk/scripts/latlong.html
   for formulas */
//double get_bearing(KalmanFilter f) {
//  double lat, lon, delta_lat, delta_lon, x, y;
//  get_lat_long(f, &lat, &lon);
//  get_velocity(f, &delta_lat, &delta_lon);
//
//  /* Convert to radians */
//  double to_radians = PI / 180.0;
//  lat *= to_radians;
//  lon *= to_radians;
//  delta_lat *= to_radians;
//  delta_lon *= to_radians;
//  
//  /* Do math */
//  double lat1 = lat - delta_lat;
//  y = sin(delta_lon) * cos(lat);
//  x = cos(lat1) * sin(lat) - sin(lat1) * cos(lat) * cos(delta_lon);
//  double bearing = atan2(y, x);
//
//  /* Convert to degrees */
//  bearing = bearing / to_radians;
//  while (bearing >= 360.0) {
//    bearing -= 360.0;
//  }
//  while (bearing < 0.0) {
//    bearing += 360.0;
//  }
//    
//  return bearing;
//}

//double calculate_mph(double lat, double lon,
//		     double delta_lat, double delta_lon) {
//  /* First, let's calculate a unit-independent measurement - the radii
//     of the earth traveled in each second. (Presumably this will be
//     a very small number.) */
//  
//  /* Convert to radians */
//  double to_radians = PI / 180.0;
//  lat *= to_radians;
//  lon *= to_radians;
//  delta_lat *= to_radians;
//  delta_lon *= to_radians;
//
//  /* Haversine formula */
//  double lat1 = lat - delta_lat;
//  double sin_half_dlat = sin(delta_lat / 2.0);
//  double sin_half_dlon = sin(delta_lon / 2.0);
//  double a = sin_half_dlat * sin_half_dlat + cos(lat1) * cos(lat)
//    * sin_half_dlon * sin_half_dlon;
//  double radians_per_second = 2 * atan2(1000.0 * sqrt(a),
//					1000.0 * sqrt(1.0 - a));
//  
//  /* Convert units */
//  double miles_per_second = radians_per_second * EARTH_RADIUS_IN_MILES;
//  double miles_per_hour = miles_per_second * 60.0 * 60.0;
//  return miles_per_hour;
//}
//
//double get_mph(KalmanFilter f) {
//  double lat, lon, delta_lat, delta_lon;
//  get_lat_long(f, &lat, &lon);
//  get_velocity(f, &delta_lat, &delta_lon);
//  return calculate_mph(lat, lon, delta_lat, delta_lon);
//}

/*
 * add by ycnalin, date 13/10/17
 * get the distance of two points, return a double distance with unit m
 * note that latFrom LonTo with a unit of degree
 * See
 * https://en.wikipedia.org/wiki/Great-circle_distance
 * for formulas
 */
double get_distance(double LatFrom, double LonFrom, double LatTo, double LonTo)
{
	double LatFrom1, LonFrom1, LatTo1, LonTo1, LonDiff;
	double Temp1, Temp2, Temp3;
	double Distance;
	LatFrom1 = LatFrom*PI / 180;
	LonFrom1 = LonFrom*PI / 180;
	LatTo1 = LatTo*PI / 180;
	LonTo1 = LonTo*PI / 180;
	LonDiff = LonTo1 - LonFrom1;
	Temp1 = cos(LatTo1)*sin(LonDiff);
	Temp1 = Temp1*Temp1;
	Temp2 = cos(LatFrom1)*sin(LatTo1) - sin(LatFrom1)*cos(LatTo1)*cos(LonDiff);
	Temp2 = Temp2*Temp2;
	Temp3 = sin(LatFrom1)*sin(LatTo1) + cos(LatFrom1)*cos(LatTo1)*cos(LonDiff);
	Distance = atan2(sqrt(Temp1 + Temp2) , Temp3);
	Distance = EARTHR*Distance;
	return Distance;
}

Location myGps_filter(double Lat, double Lon,float gpsSpeed,float gpsBearing,float imuHeading,float *fusion_heading)
{

	 Location gps_filter;
   
   static char first_init =0;
	 static Location last_gps;
	 static double last_imuheading,last_gpsheading;
	 if(first_init ==0)
	 {
			first_init =1;
			last_gps.lat = gps_filter.lat = Lat;
			last_gps.lng = gps_filter.lng = Lon;
			 last_imuheading = *fusion_heading = imuHeading;
			 
			return gps_filter;
	 }
    if(gpsSpeed <0.4 || gpsSpeed >0.8 || gpsBearing == 0)
		{
		    gps_filter = last_gps;
				last_imuheading = *fusion_heading = imuHeading;
				return gps_filter;
		}
	 	double l_dist = get_distance(last_gps.lat,last_gps.lng,Lat,Lon);
	  double l_bearing = get_bearing(last_gps.lat, last_gps.lng,Lat,Lon);
	/*	if(  gpsBearing == 0)
		{
				gps_filter = last_gps;
				return gps_filter;
		}*/
		double imuHeading_error = fabs(imuHeading - last_imuheading);
		double gpsHeading_error = fabs(gpsBearing - last_gpsheading);
		last_gpsheading = gpsBearing;
		last_gps.lat = gps_filter.lat = Lat;
		last_gps.lng = gps_filter.lng = Lon;

    float k=imuHeading_error/(imuHeading_error + gpsHeading_error);
		*fusion_heading =imuHeading+  k*(imuHeading-gpsBearing);
	return gps_filter;

		
}
