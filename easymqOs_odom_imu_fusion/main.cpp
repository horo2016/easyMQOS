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

#include "easy_mqos.h"
#include "simple_fusion.h"
using namespace std;
bool process_messages = true;
int msg_count = 0;

//解析 
/*{
	"sq":
	"sc":	1669031959,
	"ms":	236,
	"ox":	0,
	"oy":	0,
	"oz":	0,
	"ow":	1，
	"gx":	0,
	"gy":	0,
	"gz":	0,
	"ax":	0,
	"ay":	0,
	"az":	0,
	"mx":
	"my":
	"mz":
} */
char parse_json_IMU(char *buf)
{
	cJSON *parameter_element = NULL;
	sensors_msg_imu _msg_imu;


    cJSON *cjson_sc = NULL;
	cJSON *cjson_ms = NULL;

	cJSON *cjson_gx = NULL;
	cJSON *cjson_gy = NULL;
	cJSON *cjson_gz = NULL;

	cJSON *cjson_ax = NULL;
	cJSON *cjson_ay = NULL;
	cJSON *cjson_az = NULL;

	cJSON *cjson_ox = NULL;
	cJSON *cjson_oy = NULL;	 
	cJSON *cjson_oz = NULL;
	cJSON *cjson_ow = NULL;	 
	
	cJSON *cjson_mx = NULL;
	cJSON *cjson_my = NULL;
	cJSON *cjson_mz = NULL;
	
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
	cjson_sc= cJSON_GetObjectItem(parameter_element, "sc");
	cjson_ms= cJSON_GetObjectItem(parameter_element, "ms");	
	cjson_gx = cJSON_GetObjectItem(parameter_element, "gx");
	cjson_gy = cJSON_GetObjectItem(parameter_element, "gy");
	cjson_gz = cJSON_GetObjectItem(parameter_element, "gz");
	 
	cjson_ax = cJSON_GetObjectItem(parameter_element, "ax");		 
	cjson_ay = cJSON_GetObjectItem(parameter_element, "ay");
	cjson_az = cJSON_GetObjectItem(parameter_element, "az");

	cjson_ox = cJSON_GetObjectItem(parameter_element, "ox");
	cjson_oy = cJSON_GetObjectItem(parameter_element, "oy");
    cjson_oz = cJSON_GetObjectItem(parameter_element, "oz");
	cjson_ow = cJSON_GetObjectItem(parameter_element, "ow");

	cjson_mx = cJSON_GetObjectItem(parameter_element, "mx");		 
	cjson_my = cJSON_GetObjectItem(parameter_element, "my");
	cjson_mz = cJSON_GetObjectItem(parameter_element, "mz");
	
    _msg_imu.stamp_ss = cjson_sc->valuedouble;
    _msg_imu.stamp_ms = cjson_ms->valuedouble;
	_msg_imu.angular_velocity.x = (cjson_gx->valuedouble);
	_msg_imu.angular_velocity.y = (cjson_gy->valuedouble);
	_msg_imu.angular_velocity.z = (cjson_gz->valuedouble);
	

	_msg_imu.linear_acceleration.x = (cjson_ax->valuedouble);
	_msg_imu.linear_acceleration.y = (cjson_ay->valuedouble);
	_msg_imu.linear_acceleration.z = (cjson_az->valuedouble);

	_msg_imu.orientation.x = (cjson_ox->valuedouble);
	_msg_imu.orientation.y = (cjson_oy->valuedouble);
	_msg_imu.orientation.z = (cjson_oz->valuedouble);
	_msg_imu.orientation.w = (cjson_ow->valuedouble);

	
	_msg_imu.mag.x = (cjson_mx->valuedouble);
	_msg_imu.mag.y = (cjson_my->valuedouble);
	_msg_imu.mag.z = (cjson_mz->valuedouble);
	
	
    imu_callback(_msg_imu);
	
}
/*{
	"sc":	1669031959,
	"ms":	236,
	"px":	0, //位置x 
	"py":	0, //位置y
	"pz":	0, 
	"vx":	0,//线速度x
	"vz":	0,//角速度a
	"ox":	0, //四元数
	"oy":	0,
	"oz":	0,
	"ow":	1，
	"re":   0,//右轮周期计数，并非累计
	"le":   0 //左侧周期计数，并非累计的
} */
char parse_json_odom(char *buf)
{
	int i = 0;
	int j = 0;
	cJSON *parameter_element = NULL;
	
	
	cJSON *cjson_px = NULL;
	cJSON *cjson_py = NULL;
	cJSON *cjson_pz = NULL;

	cJSON *cjson_vx = NULL;
	cJSON *cjson_az = NULL;

	cJSON *cjson_ox = NULL;
	cJSON *cjson_oy = NULL;	 
	cJSON *cjson_oz = NULL;
	cJSON *cjson_ow = NULL;	 
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
		 
	sensors_msg_odom _msg_odom;
	cjson_px = cJSON_GetObjectItem(parameter_element, "px");
	cjson_py = cJSON_GetObjectItem(parameter_element, "py");
	cjson_pz = cJSON_GetObjectItem(parameter_element, "pz");
	 
	cjson_vx = cJSON_GetObjectItem(parameter_element, "vx");
	cjson_az = cJSON_GetObjectItem(parameter_element, "vz");

	cjson_ox = cJSON_GetObjectItem(parameter_element, "ox");
	cjson_oy = cJSON_GetObjectItem(parameter_element, "oy");
    cjson_oz = cJSON_GetObjectItem(parameter_element, "oz");
	cjson_ow = cJSON_GetObjectItem(parameter_element, "ow");

    
	_msg_odom.position.x = (cjson_px->valuedouble);
	_msg_odom.position.y = (cjson_py->valuedouble);

	_msg_odom.linear_velocity.x = (cjson_vx->valuedouble);
	_msg_odom.angular_velocity.z = (cjson_az->valuedouble);

	_msg_odom.orientation.x = (cjson_ox->valuedouble);
	_msg_odom.orientation.y = (cjson_oy->valuedouble);
	_msg_odom.orientation.z = (cjson_oz->valuedouble);
	_msg_odom.orientation.w = (cjson_ow->valuedouble);

	
	odom_callback(_msg_odom);
	//cmd_send2(vel,ang);
}
char parse_json_encoders(char *buf)
{
	int i = 0;
	int j = 0;
	cJSON *parameter_element = NULL;
	
	
	cJSON *cjson_left = NULL;
	cJSON *cjson_right = NULL;
	cJSON *aorno = NULL;
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
		 
	float left_encoders,right_encoders; 
	cjson_left = cJSON_GetObjectItem(parameter_element, "le");
	cjson_right = cJSON_GetObjectItem(parameter_element, "re");
	 
	 
	left_encoders = (cjson_left->valueint);
	right_encoders = (cjson_right->valueint);
 
	encoders_callback(left_encoders,right_encoders);
	//cmd_send2(vel,ang);
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
		//	printf("topic:%s ", message->topic);
			//printf("message:%s \n",message->payload);
		  	//  parse_cjson(message->payload);
			//	fwrite(message->payload, 1, message->payloadlen, stdout);
			//parse_json_fromMemory((char*)message->payload);
			if(!strcmp("/sensors/imu_node_pub",message->topic))   
				parse_json_IMU((char *)message->payload); 
			else	if(!strcmp("/sensors/imu_filter_pub",message->topic))   
				parse_json_IMU((char *)message->payload); 
			else if(!strcmp("/sensors/odom",message->topic))
				parse_json_encoders((char *)message->payload);
			 
			// if(cfg->eol){
			// 	printf("\n");
			// }
			 
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
  //  odom_task(odom_public_callback);
   fusion_task(odom_public_callback);
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
 
    e_demo->Init_Pub("/sensors/odomcombine");//发布话题
    e_demo->Set_broker("192.168.31.135");
    e_demo->Set_config_Pub();

    vector<string> topicStr_subscribe;  
    topicStr_subscribe.push_back("/sensors/odom"); //
    topicStr_subscribe.push_back("/sensors/imu_node_pub");  //原始数据
  	topicStr_subscribe.push_back("/sensors/imu_filter_pub");//互补滤波的四元数和数据
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