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
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/stat.h>
#include <assert.h>


/*pthread*/
#include "sys/ipc.h"
#include "sys/msg.h"
#include "pthread.h"
#include <arpa/inet.h>
#include <net/if.h>
 
#include "cJSON.h"
#include "easy_mqos.h"
#include "opticalflow_control.h"
#include "odometry.h"
using namespace std;
bool process_messages = true;
int msg_count = 0;




//发布话题的回调
void odom_public_callback(sensors_msg_odom _odom_msg)
{

    char tmp_buf[0xff]={0};
    char topic_buf[0xff]={0};
    char  value_buf[4096]={0};
    char  send_buf[4096]={0};

#if 1
    cJSON * root =  cJSON_CreateObject();
  if(!root) {
         printf("get root faild !\n");
     }
    
    cJSON_AddItemToObject(root, "sc",cJSON_CreateNumber(_odom_msg.stamp_ss)); 
    cJSON_AddItemToObject(root, "ms",cJSON_CreateNumber(_odom_msg.stamp_ms)); 
    cJSON_AddItemToObject(root, "px", cJSON_CreateNumber(_odom_msg.position.x));//
    cJSON_AddItemToObject(root, "py",cJSON_CreateNumber(_odom_msg.position.y));//
    cJSON_AddItemToObject(root, "pz", cJSON_CreateNumber(_odom_msg.position.z));//
    cJSON_AddItemToObject(root, "vx",cJSON_CreateNumber(_odom_msg.linear_velocity.x)); 
    cJSON_AddItemToObject(root, "vz",cJSON_CreateNumber(_odom_msg.angular_velocity.z)); 
	//四元数
    cJSON_AddItemToObject(root, "ox", cJSON_CreateNumber(_odom_msg.orientation.x));
    cJSON_AddItemToObject(root, "oy", cJSON_CreateNumber(_odom_msg.orientation.y));
    cJSON_AddItemToObject(root, "oz",cJSON_CreateNumber(_odom_msg.orientation.z));
    cJSON_AddItemToObject(root, "ow", cJSON_CreateNumber(_odom_msg.orientation.w));

	cJSON_AddItemToObject(root, "le",cJSON_CreateNumber(_odom_msg.left_encoders));
    cJSON_AddItemToObject(root, "re", cJSON_CreateNumber(_odom_msg.right_encoders));
  //  cJSON_AddItemToObject(root, "cputemp", cJSON_CreateNumber((char)cpuTemperature));
  //  cJSON_AddItemToObject(root, "\"fusionheading\"", cJSON_CreateNumber((short)fusionheading));
  //  cJSON_AddItemToObject(root, "\"wifisignal\"", cJSON_CreateNumber(wifiSignalStrength));
  //  cJSON_AddItemToObject(root, "\"velspeed\"", cJSON_CreateNumber((int)velspeed));
  //  cJSON_AddItemToObject(root, "\"angspeed\"", cJSON_CreateNumber(angspeed));
  // cJSON_AddItemToObject(root, "\"targetheading\"", cJSON_CreateNumber((int)targetHeading));
  // cJSON_AddItemToObject(root, "\"distance\"", cJSON_CreateNumber((int)waypointRange));
  //  cJSON_AddItemToObject(root, "\"nextwaypoint_lon\"", cJSON_CreateNumber(waypointlongitude));
  //   cJSON_AddItemToObject(root, "\"nextwaypoint_lat\"", cJSON_CreateNumber(waypointlatitude));
  // mqtt_publish(tmp_buf,cJSON_Print(root));
   	#endif
    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));
     
	//sprintf(topic_buf,"%s/state/gps",chargename);

#if defined MQTT_REMOTE_SERVER 
	//	sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
	//	sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif
	//system(send_buf);	
	//printf("length:%d \n :%s \n",strlen(value_buf),value_buf);
    publish_message(strlen(cJSON_Print(root)),value_buf);
    cJSON_Delete(root);

    return ;
}
//发布任务数据获取
void op_odom_PublishTask ()
{
    odom_task(odom_public_callback);
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
 
    e_demo->Init_Pub("/sensors/optical_flow");//发布话题
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

  
  	op_odom_PublishTask();
   
    return 0;
}