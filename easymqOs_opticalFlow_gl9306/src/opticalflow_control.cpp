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
#include "opticalflow_control.h"
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
 * head head  left encoder  right encoder        CRC
 * 0xaa 0xaa     float      float               u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;
float_union Vx,Vy,Ang_v;

int control_fd;

pthread_mutex_t cmd_mutex;



unsigned char data_check(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	int i;

	if((buffer[0]==0xFE) && (buffer[1]==0x04)){
		csum = buffer[2]+buffer[3]+buffer[4]+buffer[5];
		if(csum == buffer[6]){
			ret = 1;//校验通过，数据包正确
		}
		else {
		  ret =0;//校验失败，丢弃数据包
		  DEBUG(LOG_ERR,"crc failed %d %d \n",csum,buffer[34]);
		}
	}
	return ret;
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
	int i_nread;	
	control_fd= OpenDev(serialDev,19200);
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
	float_union posx,posy;
sensors_msg_odom _msg_odom;
	while(1)	
	{	
     	if((i_nread=read(control_fd,r_buffer,rBUFFERSIZE))>0)            
			{	
			//   printf("receive length :%d \n",i_nread);
			//   for (  i = 0; i < i_nread; i++)
			//   {
			// 	/* code */
 			// 		printf("%x  ",r_buffer[i]);
			//   }
			  
 			//  printf("\n");
			  if(data_check(r_buffer) != 0){
					
				int flow_x = short(r_buffer[2]|r_buffer[3]<<8);
				int flow_y = short(r_buffer[4]|r_buffer[5]<<8);
				 float vell,velr;

				printf("optical flow \n");
				printf(" (%d,%d) \n" ,flow_x,flow_y);
				// _msg_odom.position.x = position_x;
				// _msg_odom.position.y = position_y;
				// _msg_odom.position.z = 0.0;
				// _msg_odom.stamp_ss = getsecond();
				// _msg_odom.stamp_ms = getmiilsecond();
		
				
				//  set_callback_function(_msg_odom, pfunc);
				}
			}
			
		    // printf("imu serial read byte length:%d\n",i_nread);//32 bytes
			 memset(r_buffer,0,rBUFFERSIZE);
			 usleep(30000);
	}
	close(control_fd);
	printf("control_fd serial closed");     
}
	



