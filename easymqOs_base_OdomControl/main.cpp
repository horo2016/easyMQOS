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
#include "stm32_control.h"
#include "odometry.h"
using namespace std;
bool process_messages = true;
int msg_count = 0;

//解析 "control":"1","vel":"0.1","ang":"0.1"
char parse_json_fromMemory(char *buf)
{
	int i = 0;
	int j = 0;
	cJSON *parameter_element = NULL;
	
	
	cJSON *cjson_vel = NULL;
	cJSON *cjson_ang = NULL;
	cJSON *aorno = NULL;
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
		 
	float ang,vel; 
	aorno = cJSON_GetObjectItem(parameter_element, "control");
	cjson_vel = cJSON_GetObjectItem(parameter_element, "vel");
	cjson_ang = cJSON_GetObjectItem(parameter_element, "ang");
	 
	 
	vel = atof(cjson_vel->valuestring);
	ang = atof(cjson_ang->valuestring);
	cmd_send2(vel,ang);
}
//订阅话题的回调
void my_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
	struct mosq_config *cfg;
	int i;
	bool res;

	if(process_messages == false) return;

	assert(obj);
	cfg = (struct mosq_config *)obj;

	if(message->retain && cfg->no_retain) return;
	if(cfg->filter_outs){
		for(i=0; i<cfg->filter_out_count; i++){
			mosquitto_topic_matches_sub(cfg->filter_outs[i], message->topic, &res);
			if(res) return;
		}
	}

	if(cfg->verbose){
		if(message->payloadlen){
		//	printf("topic:%s ", message->topic);
		//	printf("message:%s",message->payload);
		
		//	fwrite(message->payload, 1, message->payloadlen, stdout);
			if(cfg->eol){
				printf("\n");
			}
		}else{
			if(cfg->eol){
				printf("%s (null)\n", message->topic);
			}
		}
		//fflush(stdout);
	}else{
		if(message->payloadlen){
			printf("topic:%s ", message->topic);
			printf("message:%s",message->payload);
		  	//  parse_cjson(message->payload);
			//	fwrite(message->payload, 1, message->payloadlen, stdout);
			parse_json_fromMemory((char*)message->payload);
			if(cfg->eol){
				printf("\n");
			}
			 
		}
	}
	if(cfg->msg_count>0){
		msg_count++;
		if(cfg->msg_count == msg_count){
			process_messages = false;
			mosquitto_disconnect(mosq);
		}
	}
}

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
void *odom_PublishTask (void *)
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
 
    e_demo->Init_Pub("/sensors/odom");//发布话题
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

    vector<string> topicStr_subscribe;  
    topicStr_subscribe.push_back("/basecomm/control"); //系统生成
    topicStr_subscribe.push_back("/cmd/vel");  //手动控制
    e_demo->Init_Sub(topicStr_subscribe);

    pthread_attr_t attr;
    pthread_t pthread_id = 0 ;
	struct sched_param param;
  
	pthread_attr_init (&attr);
	pthread_attr_setschedpolicy (&attr, SCHED_RR);
	param.sched_priority = 5;
	pthread_attr_setschedparam (&attr, &param);
	pthread_create (&pthread_id, &attr, &odom_PublishTask, NULL);
	pthread_attr_destroy (&attr);
    e_demo->Start_Sub("123",my_message_callback);//订阅话题的回调方法
    return 0;
}