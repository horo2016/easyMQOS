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
 
#include "cJSON.h"
#include "easy_mqos.h"

#include "easy_mqos.h"
#include "simple_navigation.h"
using namespace std;
bool process_messages = true;
int msg_count = 0;
using namespace cv;
//解析 

char parse_json_IMU085(char *buf)
{
	cJSON *parameter_element = NULL;
	sensors_msg_imu _msg_imu;


    cJSON *cjson_sc = NULL;
	cJSON *cjson_ms = NULL;

	cJSON *cjson_roll = NULL;
	cJSON *cjson_pitch = NULL;
	cJSON *cjson_yaw = NULL;

	cJSON *cjson_ax = NULL;
	cJSON *cjson_ay = NULL;
	cJSON *cjson_az = NULL;

	
	
	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
	//cjson_sc= cJSON_GetObjectItem(parameter_element, "sc");
	//cjson_ms= cJSON_GetObjectItem(parameter_element, "ms");	
	cjson_roll = cJSON_GetObjectItem(parameter_element, "roll");
	cjson_pitch = cJSON_GetObjectItem(parameter_element, "pitch");
	cjson_yaw = cJSON_GetObjectItem(parameter_element, "yaw");
	 
	cjson_ax = cJSON_GetObjectItem(parameter_element, "ax");		 
	cjson_ay = cJSON_GetObjectItem(parameter_element, "ay");
	cjson_az = cJSON_GetObjectItem(parameter_element, "az");

	
	
   // _msg_imu.stamp_ss = cjson_sc->valuedouble;
   // _msg_imu.stamp_ms = cjson_ms->valuedouble;
   //注意这里不再是角速度 而实变成了绝对角
	_msg_imu.angular_velocity.x = atof(cjson_roll->valuestring);
	_msg_imu.angular_velocity.y = atof(cjson_pitch->valuestring);
	_msg_imu.angular_velocity.z = atof(cjson_yaw->valuestring);
	

	_msg_imu.linear_acceleration.x = atof(cjson_ax->valuestring);
	_msg_imu.linear_acceleration.y = atof(cjson_ay->valuestring);
	_msg_imu.linear_acceleration.z = atof(cjson_az->valuestring);


	
    imu_callback_085(_msg_imu);
	 return 0;
}
int cmd =-1;
char parse_json_navigation(char *buf)
{
	int i = 0;
	int j = 0;
	cJSON *parameter_element = NULL;
	
	
	cJSON *cjson_va = NULL;
	

	parameter_element = cJSON_Parse(buf);
	if(!parameter_element){
		printf("prase fail\n");
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
		return -1;
	}
		 
	 
	cjson_va = cJSON_GetObjectItem(parameter_element, "value");
	 
	cmd 	 = atoi(cjson_va->valuestring);
	printf("receive cmd:%d \n",cmd);

	 return 0;
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
	 return 0;
}
//unsigned short lidar_dis[361];
char parse_lidar_datas(char *buf)
{
	int i = 0;
	int j = 0;

	unsigned long  t_stamp;
	memset(lidar_dis,0,361*sizeof(unsigned short));
	memcmp(&t_stamp,buf,4);
	//printf("rec:%ld \n",t_stamp);
	memcpy(lidar_dis,buf+4,360*sizeof(unsigned short));
	//for(int i=0;i<5;i++)
	//printf("deg2 dis :%d \n",lidar_dis[2]);
	return 0;
}

char parse_json_path(int len,char *buf)
{
	if(!wayPoints.empty())
	    wayPoints.clear();
	int route_len = len/8;//得到的(x,y)有多少组
	printf("path len:%d\n",route_len);
		
	for(int i = 0,j=0; i < len-1; i=i+8)
	{
		int x = buf[i]+buf[1+i]*256;
		int y = buf[i+4]+buf[5+i]*256;
		printf( "(%d,%d) \n",x,y);
		
		
		Point t(y,x);
		wayPoints.push_back(t);
		j ++;
		

	}
	printf("finanl path len:%d \n",wayPoints.size());
	return 0;
}
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

			 if(!strcmp("/sensors/lidar_node_pub",message->topic))
				parse_lidar_datas((char *)message->payload);
			// else 
			if(!strcmp("/sensors/odom",message->topic))
				parse_json_encoders((char *)message->payload);
			 else if(!strcmp("/sensors/bno085",message->topic))
				parse_json_IMU085((char *)message->payload);
			else if(!strcmp("/navigation/value",message->topic))
				parse_json_navigation((char *)message->payload);
			else if(!strcmp("/path/planning",message->topic))
				parse_json_path(message->payloadlen,(char *)message->payload);
					 
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
    float Dl = (_odom_msg.left_encoders)*0.000203;
	float Dr = (_odom_msg.right_encoders)*0.000203;
	float Dc = (Dl+Dr)/2.0;
    cJSON_AddItemToObject(root, "sc",cJSON_CreateNumber(_odom_msg.stamp_ss)); 
    cJSON_AddItemToObject(root, "ms",cJSON_CreateNumber(_odom_msg.stamp_ms)); 
    cJSON_AddItemToObject(root, "x", cJSON_CreateNumber(_odom_msg.position.x));//
    cJSON_AddItemToObject(root, "y",cJSON_CreateNumber(_odom_msg.position.y));//
    cJSON_AddItemToObject(root, "z", cJSON_CreateNumber(_odom_msg.position.z));//
    cJSON_AddItemToObject(root, "vx",cJSON_CreateNumber(_odom_msg.linear_velocity.x)); 
    cJSON_AddItemToObject(root, "vz",cJSON_CreateNumber(_odom_msg.angular_velocity.z)); 
	cJSON_AddItemToObject(root, "dx",cJSON_CreateNumber(Dc)); 
	
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

   navigation_task(NULL);
}
void *control_task (void *)
{
	Mat image_gray = imread("/home/orangepi/easymqos_mymapping_from_surveyor/word.png",-1);//读取原始图片
	vector<unsigned char> inImage;
	imencode(".png",image_gray,inImage);
	size_t datalen=inImage.size();
	char *msgImage=new char[datalen];
	for(int i=0;i<datalen;i++)
	{
	msgImage[i]=inImage[i];

	}
	public_map_raw_datas(datalen,msgImage);
	vector<Point> traject;
	_gPosition.x =0;
	_gPosition.y =0;
	_gPosition.z =0;
	while(1)
	{
		switch (cmd)
		{
		case /* constant-expression */ 0:
			/* code 停止运行*/
			cmd =-1;
			navigation_run = 0;
			break;
		case 1:
			navigation_run =1;
			/* code 启动导航*/
			cmd =-1;
			break;
		case 2:
			/* code 获取地图*/
		
			public_map_raw_datas(datalen,msgImage);
			cmd =-1;
			break;
		default:
			break;
		}
		Point p(_gPosition.x/10+200,_gPosition.y/10+200);
		traject.push_back(p);
		 
  		// _gPosition ; 
		public_map_traj(traject,p,_gPosition.z);
		usleep(800000);

	}
    
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
 
    e_demo->Init_Pub("/sensors/map");//发布定位话题
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

    vector<string> topicStr_subscribe;  
    topicStr_subscribe.push_back("/sensors/odom"); //
	topicStr_subscribe.push_back("/sensors/bno085");  //原始数据
	topicStr_subscribe.push_back("/sensors/lidar_node_pub");//雷达数据监听
	topicStr_subscribe.push_back("/path/planning");//
	topicStr_subscribe.push_back("/navigation/value");
	
    e_demo->Init_Sub(topicStr_subscribe);

    pthread_attr_t attr;
    pthread_t pthread_id = 0 ;
	struct sched_param param;
   //
	pthread_attr_init (&attr);
	pthread_attr_setschedpolicy (&attr, SCHED_RR);
	param.sched_priority = 5;
	pthread_attr_setschedparam (&attr, &param);
	pthread_create (&pthread_id, &attr, &odom_PublishTask, NULL);
	pthread_attr_destroy (&attr);
 
 	pthread_attr_init (&attr);
	pthread_attr_setschedpolicy (&attr, SCHED_RR);
	param.sched_priority = 5;
	pthread_attr_setschedparam (&attr, &param);
	pthread_create (&pthread_id, &attr, &control_task, NULL);
	pthread_attr_destroy (&attr);
  /*
	pthread_attr_init (&attr);
	pthread_attr_setschedpolicy (&attr, SCHED_RR);
	param.sched_priority = 5;
	pthread_attr_setschedparam (&attr, &param);
	pthread_create (&pthread_id, &attr, &data_CollectTask, NULL);
	pthread_attr_destroy (&attr);
	*/
 
    e_demo->Start_Sub("123",my_message_callback);//订阅话题的回调方法
    return 0;
}