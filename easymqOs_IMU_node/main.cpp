#include "easy_mqos.h"
#include "Mqtt/client_shared.h"
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "Mqtt/client_pub_sub.h"
#include "Mqtt/mosquitto.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <math.h>
/*pthread*/
#include "sys/ipc.h"
#include "sys/msg.h"
#include "pthread.h"
#include <arpa/inet.h>
#include <net/if.h>
#include "method_parse.h"
#include "cJSON.h"
#include "easy_mqos.h"
#include "imu.h"
using namespace std;

void IMU_filter_callback(sensors_msg_imu _imu)
{
   
   
    char topic_buf[2048]={0};
    char  value_buf[4096]={0};
    char  send_buf[4096]={0};


#if 1
    cJSON * root =  cJSON_CreateObject();
  if(!root) {
         printf("get root faild !\n");
     }
    cJSON_AddItemToObject(root, "sq", cJSON_CreateNumber(_imu.seq));//
    cJSON_AddItemToObject(root, "sc",cJSON_CreateNumber(_imu.stamp_ss)); 
     cJSON_AddItemToObject(root, "ms",cJSON_CreateNumber(_imu.stamp_ms)); 
    cJSON_AddItemToObject(root, "ow", cJSON_CreateNumber(_imu.orientation.w));
    cJSON_AddItemToObject(root, "ox", cJSON_CreateNumber(_imu.orientation.x));
    cJSON_AddItemToObject(root, "oy",cJSON_CreateNumber(_imu.orientation.y));
    cJSON_AddItemToObject(root, "oz", cJSON_CreateNumber(_imu.orientation.z));
    cJSON_AddItemToObject(root, "gx", cJSON_CreateNumber(_imu.angular_velocity.x));
    cJSON_AddItemToObject(root, "gy", cJSON_CreateNumber(_imu.angular_velocity.y));
    cJSON_AddItemToObject(root, "gz", cJSON_CreateNumber(_imu.angular_velocity.z));
    cJSON_AddItemToObject(root, "ax", cJSON_CreateNumber(_imu.linear_acceleration.x));
    cJSON_AddItemToObject(root, "ay", cJSON_CreateNumber(_imu.linear_acceleration.y));
    cJSON_AddItemToObject(root, "az", cJSON_CreateNumber(_imu.linear_acceleration.z));
    cJSON_AddItemToObject(root, "mx", cJSON_CreateNumber(_imu.mag.x));
    cJSON_AddItemToObject(root, "my", cJSON_CreateNumber(_imu.mag.y));
    cJSON_AddItemToObject(root, "mz", cJSON_CreateNumber(_imu.mag.z));

   
    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));
    unsigned int length = strlen(value_buf);
   // printf("length:%d \n :%s \n",strlen(value_buf),value_buf);
    //sprintf(topic_buf,"%s/state/gps",chargename);
    cJSON_Delete(root);
	#endif
#if defined MQTT_REMOTE_SERVER 
	//	sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
		//sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif
   // printf("%f %f %f \n",_imu.mag.x,_imu.mag.y,_imu.mag.z);
   // printf("heading :%f   \n",atan2(_imu.mag.y,_imu.mag.x)*180/3.14);
	//system(send_buf);	

    
   
#if  0
    char tmp_buf[2048]={0};
    memcpy(tmp_buf, (char*)&_imu.seq, sizeof(unsigned int));
    memcpy(tmp_buf+sizeof(unsigned int), (char*)&_imu.stamp_ss, sizeof(time_t));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t), (char*)&_imu.orientation.w, sizeof(double));//下边4个是姿态的四元素
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+sizeof(double), (char*)&_imu.orientation.x, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+2*sizeof(double), (char*)&_imu.orientation.y, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+3*sizeof(double), (char*)&_imu.orientation.z, sizeof(double));
    //下边3个是角速度
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+4*sizeof(double), (char*)&_imu.angular_velocity.x, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+5*sizeof(double), (char*)&_imu.angular_velocity.y, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+6*sizeof(double), (char*)&_imu.angular_velocity.z, sizeof(double));
     //下边3个是线加速度
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+7*sizeof(double), (char*)&_imu.linear_acceleration.x, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+8*sizeof(double), (char*)&_imu.linear_acceleration.y, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+9*sizeof(double), (char*)&_imu.linear_acceleration.z, sizeof(double));
      //下边3个是mag
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+10*sizeof(double), (char*)&_imu.mag.x, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+11*sizeof(double), (char*)&_imu.mag.y, sizeof(double));
    memcpy(tmp_buf+sizeof(unsigned int)+sizeof(time_t)+12*sizeof(double), (char*)&_imu.mag.z, sizeof(double));
    unsigned int length = sizeof(unsigned int)+sizeof(time_t)+12*sizeof(double);
    printf("length:%d \n",length);
#endif 
   // printf("message:%s \n",tmp_buf);
    publish_message(length,value_buf);
 
 

    return ;
}
/*******************************************************************************
* function name	: main
* description	: main function for 
* param[in] 	: none
* param[out] 	: none
* return 		: 0-success,-1-fail
*******************************************************************************/
int  main (int argc, char ** argv)
{
    
     easymqos *e_demo = new easymqos(CLIENT_PUB);
 
    e_demo->Init_Pub("/sensors/imu_node_pub");
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

    IMU_Task(IMU_filter_callback);//启动任务并设置回调函数

    return 0;
}