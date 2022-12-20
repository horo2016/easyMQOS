#ifndef ODOMETRY_H
#define ODOMETRY_H



#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>
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

typedef struct
{
  float LeftSpeed;
	float RightSpeed;
	unsigned int LeftPusle;
	unsigned int RightPusle;
	float leftdistance;//��ʻ�ľ��� ��λm
	float Rightdistance;
}wheelInf;
extern wheelInf wheelParam;
extern float wheel_interval ;// 
extern float position_x;
extern float position_y;
extern float oriention;
extern float velocity_linear;
extern float velocity_angular;
extern float velocity_linear_x;
extern float velocity_linear_y;
extern float velocity_linear_l;
extern float velocity_linear_r;
extern void odometry(float right,float left);
extern void odometry_simple(float right,float left);
extern long long getsecond();
extern long long getmiilsecond();
#ifdef __cplusplus
}
#endif
#endif