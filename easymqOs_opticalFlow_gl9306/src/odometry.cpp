#include "odometry.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>

wheelInf wheelParam;
/***********************************************  ???  *****************************************************************/

float position_x=0;//
float position_y=0;
float oriention=0.0;//??
float velocity_linear=0;//y??????????
float velocity_angular=0;//?????
float velocity_linear_x=0,velocity_linear_y=0;//x??y??????????
float velocity_linear_l=0,velocity_linear_r=0;
/***********************************************  ????  *****************************************************************/
 float odometry_right,odometry_left;//?????????????????

/***********************************************  ????  *****************************************************************/

float wheel_interval= 0.268f;//    272.0f;      m  //  1.0146
//float wheel_interval=276.089f;    //??????=????/0.987

float multiplier=2.0f;           //?????
float line_number=2.0f;       //????????  AB 2

float deceleration_ratio=35.0f;  //????? 170r
float wheel_diameter= 0.100f;     //??????????
float pi_1_2=1.570796f;          //??/2
float pi=3.141593f;              //??
float pi_3_2=4.712389f;          //??*3/2
float pi_2_1=6.283186f;          //??*2
//float dt=0.50f;                // 500ms

float pusle_cnt =11.0f;
float oriention_interval=0;  //dt????????��?

float sin_=0;        //???????
float cos_=0;

float delta_distance=0,delta_oriention=0;   
float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;
float oriention_1=0;
char once=1;



/****************************************************************************************************************/
long long getmillis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count = tv.tv_sec * 1000000 + tv.tv_usec;
	//printf("stamp:%ld \n",tv.tv_sec);
	//printf("us:%ld \n",tv.tv_usec);
    return (count / 1000);
}
long long getsecond()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count = tv.tv_sec ;
	
    return count ;
}
long long getmiilsecond()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count =  tv.tv_usec;
	
    return count / 1000;
	
    return count ;
}

long long  current_time, last_time;
void odometry_simple(float right,float left)
{   
	odometry_right = right;
	odometry_left = left;
    if(once)  
    {
		 
        const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio*pusle_cnt);
        const_angle=const_frame/wheel_interval;
        once=0;

        
        current_time = getmillis();
        last_time = getmillis();
        return ;
    }

   
  
    current_time = getmillis();
 
    //根据设置的速度更新里程信息
    //printf("current_time=%ld ,last_time=%ld \n",current_time , last_time);
    double dt = (float)(current_time - last_time)/1000;//ms
    //printf("dt:%f",dt);
    velocity_linear = 0.5f*(odometry_right+odometry_left)*const_frame / dt;
	velocity_angular = (odometry_right -odometry_left) *const_angle / dt;
	 
	double delta_x = velocity_linear * cos(oriention)  * dt;
	double delta_y = velocity_linear * sin(oriention)  * dt;
	double delta_th = velocity_angular * dt;
	
	position_x = position_x + delta_x;
	position_y = position_y + delta_y;
	 
	oriention += delta_th;
	// if(oriention > pi)
    // {
    //     oriention -= pi_2_1;
    // }
    // else if(oriention < -pi)
	// {
	// 	oriention += pi_2_1;
	// }
	// printf("*************odom output********************\n");
	// printf("linear vel:%3f,ang vel:%3f,\n",velocity_linear,velocity_angular);
	// printf("position_x:%3f,position_y:%3f,\n",position_x,position_y);
    // printf("heading:%3f\n",oriention*180/3.1415);
    last_time = current_time;

}

//https://blog.csdn.net/forrest_z/article/details/55001231 
