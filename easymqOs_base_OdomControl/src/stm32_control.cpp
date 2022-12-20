/*********************************
 * 串口节点，订阅cmd_vel话题并发布odometry话题
 * 从cmd_vel话题中分解出速度值通过串口送到移动底盘
 * 从底盘串口接收里程消息整合到odometry话题用于发布
 * 
 * @StevenShi
 * *******************************/
/* include head_files*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/select.h>
#include <fcntl.h>   //low_level i/o
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "stm32_control.h"
#include <asm/types.h>  //for videodev2.h
#include <linux/videodev2.h>
#include <pthread.h>

//call  socket 
// #include "socket_tcp.h"
 #include "osp_common.h"
 #include "osp_syslog.h"
 #include "odometry.h"
#include "Uart_comm.h"
#include "rtquaternion.h"
CVFEED cv_res;


#define	sBUFFERSIZE	16//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	64//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

float velspeed =0.0;
float angspeed =0.0;
unsigned int positionx =0;

/************************************
 * 串口数据发送格式共15字节
 * head head len  linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff u8  float       float      float      u8
 * **********************************/
/**********************************************************
 * 串口接收数据格式共27字节
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xaa 0xaa float      float      float   float   float         float(yaw)   u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;
float_union Vx,Vy,Ang_v;

int control_fd;

pthread_mutex_t cmd_mutex;

 

 

int send2fd(unsigned char *dat,int len)
{
	pthread_mutex_lock(&cmd_mutex); /*获取互斥锁*/
	write(control_fd, dat,sBUFFERSIZE);  
	pthread_mutex_unlock(&cmd_mutex); /*释放互斥锁*/
}
/**********************************************************
 * 数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void data_pack(const char * cmd_vel){
	unsigned char i;
	float_union Vx,Vy,Ang_v;
	//Vx.fvalue = cmd_vel.linear.x;
	//Vy.fvalue = cmd_vel.linear.y;
	//Ang_v.fvalue = cmd_vel.angular.z;
	memcpy(Vx.cvalue,cmd_vel+3,4);
	memcpy(Vy.cvalue,cmd_vel+7,4);
	memcpy(Ang_v.cvalue,cmd_vel+11,4);
	printf("vx speed:%.2f \n",Vx.fvalue);
	printf("ang  speed:%.2f \n",Ang_v.fvalue);
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	s_buffer[2] = 15;
	//Vx
	s_buffer[3] = Vx.cvalue[0];
	s_buffer[4] = Vx.cvalue[1];
	s_buffer[5] = Vx.cvalue[2];
	s_buffer[6] = Vx.cvalue[3];
	//Vy
	s_buffer[7] = Vy.cvalue[0];
	s_buffer[8] = Vy.cvalue[1];
	s_buffer[9] = Vy.cvalue[2];
	s_buffer[10] = Vy.cvalue[3];
	//Ang_v
	s_buffer[11] = Ang_v.cvalue[0];
	s_buffer[12] = Ang_v.cvalue[1];
	s_buffer[13] = Ang_v.cvalue[2];
	s_buffer[14] = Ang_v.cvalue[3];
	//CRC
	s_buffer[15] = s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13]^s_buffer[14];
	
	//for(i=0;i<16;i++){
	//	printf("0x%02x ",s_buffer[i]);
	//}
	
	//ser.write(s_buffer,sBUFFERSIZE);
	write(control_fd, s_buffer,sBUFFERSIZE);  
}

/**********************************************************
 * 数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void cmd_send(const char cmd_v,int speed)
{
	unsigned char i;
	char cmd_vel[16];
	
//前后 左右 停止 ，带角速度和线速度同时存在
	if(cmd_v ==1 ){
	  Vx.fvalue = speed;
	  
	}
   else if(cmd_v == 2){
	   Vx.fvalue =  speed;
		
   	}
     else if(cmd_v == 3)
     	{
     	Vx.fvalue = 0; 
	   Ang_v.fvalue = -0.4;
		 }
     else if(cmd_v == 4){
	   Ang_v.fvalue = 0.4;
		 Vx.fvalue = 0; 
     	}
   else if(cmd_v == 0){
         Vx.fvalue = 0; 
		 Ang_v.fvalue = 0;
		  //Ang_v.fvalue = 0;
    }
     else if(cmd_v == 5){
          
		 Ang_v.fvalue = speed;
		 
    }
	  Vy.fvalue = 0;
	  
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	s_buffer[2] = 15;
	//Vx
	memcpy(s_buffer+3,Vx.cvalue,4);
	memcpy(s_buffer+7,Vy.cvalue,4);
	memcpy(s_buffer+11,Ang_v.cvalue,4);
	
	//CRC
	s_buffer[15] = s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6] \
	^s_buffer[7]^s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11] \
	^s_buffer[12]^s_buffer[13]^s_buffer[14];
	
	//for(i=0;i<16;i++){
	//	printf("0x%02x ",s_buffer[i]);
	//}
	
	//ser.write(s_buffer,sBUFFERSIZE);
	write(control_fd, s_buffer,sBUFFERSIZE);  
}
//w > 0 左转 w <0 右转
void cmd_send2(float vspeed,float aspeed)
{
	unsigned char i;
	char cmd_vel[16];
	
//前后 左右 停止 ，带角速度和线速度同时存在
	
     
	Ang_v.fvalue = aspeed;
	Vx.fvalue = vspeed; 
	Vy.fvalue = 0;
  
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	s_buffer[2] = 15;
	//Vx
	memcpy(s_buffer+3,Vx.cvalue,4);
	memcpy(s_buffer+7,Vy.cvalue,4);
	memcpy(s_buffer+11,Ang_v.cvalue,4);
	
	//CRC
	s_buffer[15] = s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6] \
	^s_buffer[7]^s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11] \
	^s_buffer[12]^s_buffer[13]^s_buffer[14];
	
	//for(i=0;i<16;i++){
	//	printf("0x%02x ",s_buffer[i]);
	//}
	
	//ser.write(s_buffer,sBUFFERSIZE);
	send2fd(s_buffer,sBUFFERSIZE);
}

//接收数据分析与校验
unsigned char data_analysis(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	int i;

	if((buffer[0]==0xaa) && (buffer[1]==0xaa)){
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11]^buffer[12]^buffer[13]^
				buffer[14]^buffer[15]^buffer[16]^buffer[17]^buffer[18]^buffer[19]^
				buffer[20]^buffer[21]^buffer[22]^buffer[23]^buffer[24]^buffer[25];
				//buffer[26]^buffer[27]^buffer[28]^buffer[29]^buffer[30]^buffer[31]^
			//	buffer[32]^buffer[33];
		//ROS_INFO("check sum:0x%02x",csum);
		
		if(csum == buffer[26]){
			ret = 1;//校验通过，数据包正确
		}
		else {
		  ret =0;//校验失败，丢弃数据包
		  DEBUG(LOG_ERR,"crc failed %d %d \n",csum,buffer[34]);
		}
	}
	/*
	for(i=0;i<rBUFFERSIZE;i++)
	  ROS_INFO("0x%02x",buffer[i]);
	*/
	return ret;

}
unsigned char data_check(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	int i;

	if((buffer[0]==0xaa) && (buffer[1]==0xaa)){
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9];
		if(csum == buffer[10]){
			ret = 1;//校验通过，数据包正确
		}
		else {
		  ret =0;//校验失败，丢弃数据包
		  DEBUG(LOG_ERR,"crc failed %d %d \n",csum,buffer[34]);
		}
	}
	return ret;
}
//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const char * cmd_vel){
	//ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	//ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	//std::cout << "Twist Received" << std::endl;	

	data_pack(cmd_vel);
}

void set_callback_function(sensors_msg_odom _msg ,void(*pfunc)(sensors_msg_odom))
{
   (*pfunc)(_msg);
   return ;
}
int  odom_task(void(*pfunc)(sensors_msg_odom ))
{
	
	unsigned int i;	
	char serialDev[256]={0};
	strcpy(serialDev, STMDEVICE);
	//char *dev ="/dev/ttystm";//这里有改动
	int i_nread;	
	control_fd= OpenDev(serialDev,115200);
	if(control_fd>0){
		printf("%s open ok \n",serialDev);
	}
	else {
		   printf("%s open failed \n",serialDev);
	}
	int  ret = pthread_mutex_init(&cmd_mutex, NULL);
     if(ret != 0) {
         perror("mutex init failure");
         return 1;
     }
    Vx.fvalue = 0; 
	Vy.fvalue = 0; 
	Ang_v.fvalue = 0; 
	float_union posx,posy,vx,vy,va,yaw,vr,vl;
	sensors_msg_odom _msg_odom;
	RTQuaternion _quaternion;
	Vector3 rpy;
	while(1)	
	{	
     	if((i_nread=read(control_fd,r_buffer,rBUFFERSIZE))>0)            
			{	
		//	  printf("receive length :%d \n",i_nread);
		//	  printf("%x ",r_buffer[0]);
			  if(data_check(r_buffer) != 0){
				int i;
				for(i=0;i<4;i++){
					vl.cvalue[i] = r_buffer[2+i];//encoder
					vr.cvalue[i] = r_buffer[6+i];//encoder
					
					//vr.cvalue[i] = r_buffer[26+i];
					//vl.cvalue[i] = r_buffer[30+i];
				}	
				//  printf("***********parse Result**************\n");
				//  printf("From car receive vl.cvalue:%.3f \n",vl.fvalue);
				//  printf("From car receive vr.cvalue:%.3f \n",vr.fvalue);
				 float vell,velr;
	
				//vell =(float)vl.fvalue*100/4/101/11*314;
				odometry_simple(vr.fvalue,vl.fvalue);
				
				
				_msg_odom.linear_velocity.x = velocity_linear;
				_msg_odom.linear_velocity.y = 0.0;
				_msg_odom.angular_velocity.z = velocity_angular;
				_msg_odom.orientation.w =0.0;

				_msg_odom.position.x = position_x;
				_msg_odom.position.y = position_y;
				_msg_odom.position.z = 0.0;
				_msg_odom.stamp_ss = getsecond();
				_msg_odom.stamp_ms = getmiilsecond();
				rpy.x =0;
				rpy.y =0;
				rpy.z = oriention;
				_quaternion.fromEuler(rpy);
				_msg_odom.orientation.w = _quaternion.scalar();
				_msg_odom.orientation.x = _quaternion.x();
				_msg_odom.orientation.y = _quaternion.y();
				_msg_odom.orientation.z = _quaternion.z();

				_msg_odom.left_encoders =0- (int)vl.fvalue;
				_msg_odom.right_encoders = 0-(int)vr.fvalue;
				 set_callback_function(_msg_odom, pfunc);
				}
			}
			
		    // printf("imu serial read byte length:%d\n",i_nread);//32 bytes
			 memset(r_buffer,0,rBUFFERSIZE);
			 usleep(200000);
	}
	close(control_fd);
	printf("control_fd serial closed");     
}
	



