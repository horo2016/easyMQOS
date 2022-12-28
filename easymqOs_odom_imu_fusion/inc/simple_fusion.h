#ifndef __EKF_FUSION_H
#define __EKF_FUISION_H

#include <iostream>
 
#include <cmath>
#include "sensor_msg.h"
#include "rtquaternion.h"


void imu_callback(sensors_msg_imu msg);
void encoders_callback( int l_encodes, int r_encodes);
// encoders callback
void odom_callback(sensors_msg_odom msg);
int fusion_task(void(*pfunc)(sensors_msg_odom ));
#endif
