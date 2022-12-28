#include "simple_fusion.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include <chrono>
#include <thread>
#include "sensor_msg.h"
#include "rtquaternion.h"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "kalman.h"
char process_init =0;
float act_heading;	
float init_heading =0;
Kalman xAccelFilter(0.225, 0.125, 1, 0);

Kalman gyrolFilter(0.025, 0.25, 1, 0);
float px=0,py=0;
Vector3 _gPosition ; 
float dis_odom =0;
float vx =0;
 
 // data collection parameters
int num_data =100;
int imu_counter = 0;

// data storage elements
float sum_accel[3];
float sum_gyro[3];
float acc_min_x =999;
float acc_max_x =0;
float vt =0,s_d=0;
long time_prev_;
long time_prev_ms;
float gyro_ang =0;//单位时间内转过的而角度
float last_heading =0;
float last_act_heading= -999;
float odom_theta =0 ;

char once_phase =0;

 


int ticks_l_curr ;//msg->data[0]; // total ticks left wheel
int ticks_r_curr;
float _g_odom_ang_rad=0;
float _g_last_odom_ang_rad =0,_g_last_imu_ang_rad =0;
float delata_theta_thesh =0.0115;//need check  /500ms  0.0625
float not_running=0.005,is_running=0.010;
float statis =0;
float g_distance_acc =0;
float g_anga_rad =0;
// encoders callback
void encoders_callback(    int l_encodes,  int r_encodes)
{		
	 ticks_l_curr = l_encodes;//msg->data[0]; // total ticks left wheel
	 ticks_r_curr = r_encodes;//msg->data[1]; // total ticks right wheel

	if(process_init ==0)
		return;
	printf("encoders :L:%d ,R:%d \n",l_encodes,r_encodes);
    if(last_act_heading == -999){
	  last_act_heading =act_heading;
	  return ;
	}
	// Compute distance moved by each wheel	
	float Dl = (ticks_l_curr)*0.000203;
	float Dr = (ticks_r_curr)*0.000203;
	float Dc = (Dl+Dr)/2.0;
	float theta  =(Dr-Dl)/0.24 ; // theta  车宽度为l=0.2m

	vx = Dc;
    float delta_theta_odom = theta - _g_last_odom_ang_rad ;
	_g_last_odom_ang_rad = theta;
	float delta_theta_imu = gyro_ang - _g_last_imu_ang_rad ;
	_g_last_imu_ang_rad = gyro_ang;

	float finanl_theta =0;
	float delta_theta_g_o = delta_theta_imu - delta_theta_odom;
	printf("delta_theta_imu:%f,delta_theta_odom:%f ,=%f \n",delta_theta_imu , delta_theta_odom,delta_theta_g_o);
	if(abs(delta_theta_g_o) > delata_theta_thesh){
		finanl_theta = gyro_ang;
		printf("final theta from imu gyro\n");
	}
		
	else 	if(abs(delta_theta_g_o) < delata_theta_thesh){
		finanl_theta = theta;
		printf("final theta from odom theta \n");
	}
		

	
	printf("finanl_theta :%f \n",finanl_theta);
 	_g_odom_ang_rad += theta;

	g_anga_rad += gyro_ang;
	printf("odom_degree:(%f),\n",_g_odom_ang_rad*180/3.14);
 	printf("imu_degree:%f \n",g_anga_rad*180/3.14);

	dis_odom += Dc;
	g_distance_acc += s_d;
	printf("distance_acc :%f \n",s_d);
	printf("distance_odom :%f \n",Dc);
	float w = theta*2;//500ms 角速度
    
	printf("gyro_degree:%f/500ms,odom theta:%f/500ms \n",gyro_ang,theta);
	if(abs(gyro_ang) < not_running && abs(theta) > is_running)
	{
		//statis ++;
		printf("error :stuck \n");
		return ;
	}else {
		//statis --;
	}
	if(statis >= 2)
	{
		printf("error :stuck \n");
		return ;
	}


	_gPosition.z += finanl_theta;//rad
    float   o_x = Dc *cos(_gPosition.z);
	float   o_y = Dc *sin(_gPosition.z);
	
	_gPosition.x = _gPosition.x +  o_x; //degree
	_gPosition.y = _gPosition.y +  o_y;
	 


	printf("robot pose (%f,%f,%f),\n",_gPosition.x,_gPosition.y,_gPosition.z*180/3.14);


	// _ekf.update( z );
    // Eigen::Matrix<double, 4, 1> x_now = _ekf.getStateX();
	//std::cout<<"X = "<<std::endl<<x_now<<std::endl;
	//printf("--->%f \n",x_now[2]*180/3.14);

	once_phase =1;  //完成一个阶段
 
}

// encoders callback
void odom_callback(sensors_msg_odom _msg)
{		
 
    
}

float acc_biases[3] ;
float gyro_biases[3];

// imu callback
// updates orientation and position using quaternion math and physics kinematic equations
// Prediction Step/Time Propogation/State Update/Vehicle Kinematics step
//更新姿态位置通过运动学 预测时间步长
void imu_callback(sensors_msg_imu msg)
{
	
	
	// IMU data
	// geometry_msgs::Vector3 w = ;msg->angular_velocity;
	Vector3 gyro = msg.angular_velocity;
	// geometry_msgs::Vector3 f = msg->linear_acceleration;
	Vector3 acc = msg.linear_acceleration;
	Vector3 mag = msg.mag;
	
	
		 
	 
	const time_t t_s = msg.stamp_ss;
	const time_t t_ms = msg.stamp_ms;
   
      // // Initialize.
 
    // Calculate dt.
	if(process_init == 0)
	 {
		if (imu_counter < num_data)
		{
			printf("init collect imu frame:%d \n",imu_counter);
			sum_accel[0] += (acc.x);
			sum_accel[1] += (acc.y);
			sum_accel[2] += (acc.z);
			sum_gyro[0] += (gyro.x);
			sum_gyro[1] += (gyro.y);
			sum_gyro[2] += (gyro.z);
			 
			if(acc_max_x < acc.x )
				acc_max_x = acc.x;
			if(acc_min_x > acc.x )
				acc_min_x = acc.x;
			// increment counter
			imu_counter++;
			return ;
		} else {
				 acc_biases[0] = sum_accel[0] / num_data;
				 acc_biases[1] = sum_accel[1] / num_data;
				 acc_biases[2] = sum_accel[2] / num_data;
				 gyro_biases[0] = sum_gyro[0] / num_data;
				 gyro_biases[1] = sum_gyro[1] / num_data;
				 gyro_biases[2] = sum_gyro[2] / num_data;

				 
				printf("init accel bias:%f,%f,%f\n",acc_biases[0],acc_biases[1],acc_biases[2]);
				printf("init gyro bias:%f,%f,%f\n",gyro_biases[0],gyro_biases[1],gyro_biases[2]);
				process_init =1;
		}
		printf("acc min,max (%f,%f) \n ",acc_min_x,acc_max_x);	
		time_prev_ = t_s;
        time_prev_ms = t_ms;
		last_heading = atan2(mag.y,mag.x)*180/3.1415;
			if(last_heading<0)last_heading=last_heading+360;
	        if(last_heading<100)last_heading =last_heading*1.5;
			else if(last_heading<140)last_heading =last_heading*1.5;
			else if(last_heading<150)last_heading =last_heading*1.45;
			else if(last_heading<160)last_heading =last_heading*1.40;
			else if(last_heading<170)last_heading =last_heading*1.35;
			else if(last_heading<180)last_heading =last_heading*1.3;
			else if(last_heading<190)last_heading =last_heading*1.25;
			else if(last_heading<200)last_heading =last_heading*1.2;
			else if(last_heading<210)last_heading =last_heading*1.15;
			else if(last_heading<220)last_heading =last_heading*1.10;
			else if(last_heading<230)last_heading =last_heading*1.05;
			else if(last_heading<240)last_heading =last_heading*1.01;

			if(last_heading < -180)                   //#Note:numpy and OpenCV X and Y reversed
					last_heading = last_heading + 360 ;
			else if( last_heading > 180)
				last_heading = last_heading - 360 ;
		
				return;
			}
    double dt;
    
    if(t_ms > time_prev_ms)
        dt = (double) (t_ms - time_prev_ms )/1000;//s
    else if(t_ms < time_prev_ms)  
        dt =   (double) (t_ms+1000 - time_prev_ms )/1000;//s
    time_prev_ms = t_ms;
  

    float acc_current_x ;
	 if(acc.x > acc_max_x)
         acc_current_x = (acc.x -(acc_min_x+acc_max_x)/2);
	else if(acc.x < acc_min_x)
         acc_current_x = (acc.x -(acc_min_x+acc_max_x)/2);
	else  acc_current_x =0.0;

   
    acc_current_x = acc.x - acc_biases[0];
	if(abs(acc_current_x) < 0.005)//静止阈值
		acc_current_x =0;
	if(vx ==0)
        vt =0;
	//printf("x axix accx:%f \n",acc_current_x);
	s_d += vt*dt + dt*dt*acc_current_x*0.5*9.8;

	vt = vt+acc_current_x*dt*9.8;
	//printf("x vt :%f \n",vt);

	if(once_phase ==1)
	{
	  
	  
       s_d =0;
		
	   gyro_ang =0;
	   once_phase =0;
	}


//	printf("current acc=%f. \n",acc.x);
//	printf("IMU current vt=%f,dis=%f,fusionacc=%f \n",vt,s_d,acc_current_x);
	float current_gyro_velocity = gyro.z-gyro_biases[2];
//	printf("z gyro velocity:%f \n",current_gyro_velocity);
	if(abs(current_gyro_velocity) >0.0021)
		gyro_ang += current_gyro_velocity *dt;
	 


   float  heading = atan2(mag.y,mag.x)*180/3.1415;
			if(heading<0)heading=heading+360;
	        if(heading<100)heading =heading*1.5;
			else if(heading<140)heading =heading*1.5;
			else if(heading<150)heading =heading*1.45;
			else if(heading<160)heading =heading*1.40;
			else if(heading<170)heading =heading*1.35;
			else if(heading<180)heading =heading*1.3;
			else if(heading<190)heading =heading*1.25;
			else if(heading<200)heading =heading*1.2;
			else if(heading<210)heading =heading*1.15;
			else if(heading<220)heading =heading*1.10;
			else if(heading<230)heading =heading*1.05;
			else if(heading<240)heading =heading*1.01;

	if(heading < -180)                   //#Note:numpy and OpenCV X and Y reversed
				heading = heading + 360 ;
			else if( heading > 180)
            heading = heading - 360 ;

  
    
     float gyro_delta_hading = gyro_ang*180/3.1415926/10*1.5;
	if(gyro_delta_hading < -180)                   //#Note:numpy and OpenCV X and Y reversed
            gyro_delta_hading = gyro_delta_hading + 360 ;
        else if( gyro_delta_hading > 180)
            gyro_delta_hading = gyro_delta_hading - 360 ;




	// printf("mag heading =%f,gyroheading=%f,fusionheading:%f   \n",heading,gyro_delta_hading,act_heading);


	// Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
	//float roll = atan(  sqrt(pow(msg.linear_acceleration.x, 2) + pow(msg.linear_acceleration.z, 2)),msg.linear_acceleration.y ) * 180 / M_PI;
	//float pitch = atan(sqrt(pow(msg.linear_acceleration.y, 2) + pow(msg.linear_acceleration.z, 2))/ msg.linear_acceleration.x ) * 180 / PI;
	//与自然z轴角度
  //  float  pitch=atan(sqrt(pow(msg.linear_acceleration.x,2)+pow(msg.linear_acceleration.y,2))/msg.linear_acceleration.z) * 180 / M_PI;
    
    //与自然X轴的角度
	//  float yaw =atan(msg.linear_acceleration.x/sqrt(pow(msg.linear_acceleration.y,2)+pow(msg.linear_acceleration.z,2))) * 180 / M_PI;
    //与自然Y轴的角度
   // float roll =  atan(msg.linear_acceleration.y/sqrt(pow(msg.linear_acceleration.x, 2) + pow(msg.linear_acceleration.z, 2))) * 180 / M_PI;

 //   printf("roll:%f,pitch :%f,yaw:%f \n",roll,pitch,yaw);
	// return ;
	

 
	// double time_now = imu_data(0) / 1000;
	// double w_now = imu_data(3);			

	// std::cout<<"timestamp = "<<time_now<<std::endl;
  //  printf("dt :%f \n",dt);
	 // _ekf.predict((abs(gyro.z)>0.01)?gyro.z:0,dt,(abs(gyro.z)>0.01)?gyro.z:0);
 
 // Ayx=(atan2(acc.x,acc.y));
 
    // gyrolFilter.update_2(gyro.z *dt);
	// float dd =gyrolFilter.GetValue()*180/3.14;
	// static float ddd=0;
	// ddd += dd;
	// printf("get %f ,%f\n",gyrolFilter.GetValue()*180/3.14,ddd);
	
}
int main1()
{
	std::cout<<"--------------------- EKF FUSION--------------------"<<std::endl;

	// for displaying
 

	return 0;
}
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
void set_callback_function(sensors_msg_odom _msg ,void(*pfunc)(sensors_msg_odom))
{
   (*pfunc)(_msg);
   return ;
}
int fusion_task(void(*pfunc)(sensors_msg_odom ))
{

	sensors_msg_odom _msg_odom;
	RTQuaternion _quaternion;
	Vector3 rpy;
	while(1)
	{
		_msg_odom.position = _gPosition;

		_msg_odom.stamp_ss = getsecond();
		_msg_odom.stamp_ms = getmiilsecond();
		rpy.x =0;
		rpy.y =0;
		rpy.z = _gPosition.z;
		_quaternion.fromEuler(rpy);

		_msg_odom.position.x = _gPosition.x;
		_msg_odom.position.y = _gPosition.y;
		_msg_odom.position.z = _gPosition.z;
		_msg_odom.orientation.w = _quaternion.scalar();
		_msg_odom.orientation.x = _quaternion.x();
		_msg_odom.orientation.y = _quaternion.y();
		_msg_odom.orientation.z = _quaternion.z();

		_msg_odom.right_encoders =ticks_r_curr;
		_msg_odom.left_encoders = ticks_l_curr;

		set_callback_function(_msg_odom, pfunc);
		usleep(500000);
	}

	return 0;
}