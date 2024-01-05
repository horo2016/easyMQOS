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

#include <opencv2/opencv.hpp>
/*pthread*/
#include "sys/ipc.h"
#include "sys/msg.h"
#include "pthread.h"
#include <arpa/inet.h>
#include <net/if.h>
 #include <sys/time.h>
#include "cJSON.h"
 
#include "easy_mqos.h"
#include "astar.h"
using namespace std;
using namespace cv;
bool process_messages = true;
int msg_count = 0;
float dx	=0; 
//解析实时定位
char parse_json_mapxy(char *buf)
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
		 
	
	cjson_left = cJSON_GetObjectItem(parameter_element, "x");
	cjson_right = cJSON_GetObjectItem(parameter_element, "y");
	 
	 
	start_point_y = atoi(cjson_left->valuestring);//需要反相赋值
	start_point_x = atoi(cjson_right->valuestring);
	printf("recive map/xy (%d,%d) \n",start_point_x,start_point_y);
}
char parse_json_mapgoal(char *buf)
{
	int i = 0;
	int j = 0;
	printf("%d \n",strlen(buf));
	if(strlen(buf)>200)
		return 0;
	cJSON *parameter_element = NULL;
	
	
	cJSON *cjson_left = NULL;
	cJSON *cjson_right = NULL;
	cJSON *_switch = NULL;
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
		 
 	_switch = cJSON_GetObjectItem(parameter_element, "control");
	cjson_left = cJSON_GetObjectItem(parameter_element, "x");
	cjson_right = cJSON_GetObjectItem(parameter_element, "y");
	
	int ss = atoi(_switch->valuestring);//需要反相赋值
	 
	//a*算法中和一般坐标系x,y相反
	goal_point_y = atoi(cjson_left->valuestring);//需要反相赋值
	goal_point_x = atoi(cjson_right->valuestring);

	if(ss ==1)start_find =1;
}
//解析轨迹的最后一个点
char parse_json_traj(char *buf)
{
	int i = 0;
	int j = 0;
	printf("parse_json_traj %d \n",strlen(buf));
	if(strlen(buf)>20000)
		return 0;
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
		 
	
	cjson_left = cJSON_GetObjectItem(parameter_element, "x");
	cjson_right = cJSON_GetObjectItem(parameter_element, "y");
	 
	 
	start_point_y = (cjson_left->valueint);//需要反相赋值
	start_point_x = (cjson_right->valueint);
	printf("recive map/xy (%d,%d) \n",start_point_x,start_point_y);
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
			if(!strcmp("/map/xy",message->topic))   {
				parse_json_mapxy((char *)message->payload); 
			}
			else if(!strcmp("/map/goal",message->topic))
			{
				parse_json_mapgoal((char *)message->payload);

			}else if(!strcmp("/sensors/map",message->topic))
			{
				parse_json_traj((char *)message->payload);

			}
				
				
			 
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

//发布话题的回调函数 这里发布A*规划好的路径点
void path_public_callback(std::vector<Point> msg)
{

    char tmp_buf[0xff]={0};
    char topic_buf[0xff]={0};
    char  value_buf[4096]={0};
    char  send_buf[4096]={0};

	int *pBuf =(int*)malloc(2*msg.size()*sizeof(int));
	for(int i=0,j=0;i<msg.size()*2;i=i+2,j++)
	{
		*(pBuf+i) = msg[j].x;
        *(pBuf+i+1)= msg[j].y;
 
	}


#if defined MQTT_REMOTE_SERVER 
	//	sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
	//	sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif
	//system(send_buf);	
	printf("length:%d  \n",2*msg.size()*sizeof(int));
    publish_message(2*msg.size()*sizeof(int),(char*)pBuf);
 
	free(pBuf);
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
 
    e_demo->Init_Pub("/path/planning");//发布话题
    e_demo->Set_broker("127.0.0.1");//broker 的 IP地址
	e_demo->Set_config_Pub();
    vector<string> topicStr_subscribe;  
    topicStr_subscribe.push_back("/map/xy"); //监听实时定位xy
    topicStr_subscribe.push_back("/map/goal");// 监听需要路径规划的指定的目标点
	topicStr_subscribe.push_back("/sensors/map");// 监听需要路径规划的指定的目标点
    e_demo->Init_Sub(topicStr_subscribe);

    pthread_attr_t attr;
    pthread_t pthread_id = 0 ;
	struct sched_param param;
  
	pthread_attr_init (&attr);
	pthread_attr_setschedpolicy (&attr, SCHED_RR);
	param.sched_priority = 5;
	pthread_attr_setschedparam (&attr, &param);
	pthread_create (&pthread_id, &attr, findPathAstar_task, NULL);// 
	pthread_attr_destroy (&attr);
	
    e_demo->Start_Sub("123",my_message_callback);//订阅话题的回调方法
    return 0;
}