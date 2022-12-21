#ifndef GPS_MAIN_H
#define GPS_MAIN_H


#include "coordinate_sys.h"


#ifdef __cplusplus
extern "C" {
#endif
extern 
void *GpsThread(void *);


extern 
int  gps_task(void(*pfunc)(Location ,float,float));
#ifdef __cplusplus
}
#endif

#endif
