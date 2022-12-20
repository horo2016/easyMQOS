#ifndef STM32_CONTROL_H
#define STM32_CONTROL_H


#ifdef __cplusplus
extern "C" {
#endif

#include "odometry.h"

typedef struct {
char valid;
char X_V;
char Y_V;

}CVFEED;
extern float velspeed ;
extern float angspeed ;
extern CVFEED cv_res;
extern unsigned int positionx;
extern int  odom_task(void(*pfunc)(sensors_msg_odom ));
extern void cmd_vel_callback(const char * cmd_vel);
extern void cmd_send(const char cmd_v,int speed);
extern void cmd_send2(float vspeed,float aspeed);

 
#ifdef __cplusplus
}
#endif
#endif

