#ifndef _SENSOR_MSG_H
#define  _SENSOR_MSG_H


#include <time.h>
#include <string>
using namespace std;
#ifdef __cplusplus
extern "C" {
#endif


typedef struct Quaternion
{

    double x;
    double y;
    double z;
    double w;
};
typedef struct Vector3
{

    double x;
    double y;
    double z;
};
typedef struct sensors_msg_imu
{
    /* data */

    unsigned int seq  ;
    time_t stamp_ss  ; // 时间戳
    time_t stamp_ms  ; // 时间戳
    Quaternion orientation  ; // 姿态
    double orientation_covariance[9]   ;// 姿态协方差
    Vector3 angular_velocity  ; // 角速度
    double angular_velocity_covariance[9];   // 角速度协方差
    Vector3 linear_acceleration ; // 线加速度
    double linear_acceleration_covariance[9] ;  // 线加速度协方差
     Vector3 mag ; // 角度  ROS中没有 单独加的
}__attribute__((packed));

typedef struct sensors_msg_odom
{
    /* data */

    unsigned int seq  ;
    time_t stamp_ss  ; // 时间戳
    time_t stamp_ms  ; // 时间戳
    Quaternion orientation  ; // 姿态
    Vector3 angular_velocity  ; // 角速度
    Vector3 linear_velocity ; // 线速度
    Vector3 position ; // 
    int left_encoders;
    int right_encoders;
}__attribute__((packed));



#ifdef __cplusplus
}
#endif
#endif