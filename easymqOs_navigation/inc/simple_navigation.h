#ifndef __EKF_FUSION_H
#define __EKF_FUISION_H

#include <string>
#include <iostream>
 
#include <cmath>
#include "sensor_msg.h"
#include "rtquaternion.h"
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void imu_callback(sensors_msg_imu msg);
void imu_callback_085(sensors_msg_imu msg);
void encoders_callback( int l_encodes, int r_encodes);
// encoders callback
void odom_callback(sensors_msg_odom msg);
int navigation_task(void(*pfunc)(sensors_msg_odom ));
void *data_CollectTask (void *);

void public_map_raw_datas(int len, char *buf);
void public_map_traj(vector<Point> traj,Point p,int ang);
extern unsigned short lidar_dis[361];
extern char navigation_run;
extern vector<Point> wayPoints;
extern Vector3 _gPosition ; 
#endif
