#ifndef GEtLIDAR_h
#define GEtLIDAR_h





// #ifdef __cplusplus
// extern "C" {
// #endif




typedef struct
{
 unsigned short  dis[360];
 unsigned long timeStamp;
}lidar_t;
extern char fixing ;
 
extern void lidar_task(void(*pfunc)(lidar_t));
// #ifdef __cplusplus
// }
// #endif

#endif
