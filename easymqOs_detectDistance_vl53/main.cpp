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
 #include <sys/time.h>
#include "cJSON.h"


#include "easy_mqos.h"
#include "VL53_interface.h"
using namespace std;
bool process_messages = true;
int msg_count = 0;
float dx	=0; 

//解析 
/*{
	"sq":
	"sc":	1669031959,
	"ms":	236,
	"left":	0,
	"head":	0,
	"right":	0,
	
} */


//发布任务数据获取
void vl53_callback(char* _msg)
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
    
    cJSON_AddItemToObject(root, "sc",cJSON_CreateNumber(0)); 
    cJSON_AddItemToObject(root, "ms",cJSON_CreateNumber(0)); 
    cJSON_AddItemToObject(root, "left", cJSON_CreateNumber(_msg[0]));//
    cJSON_AddItemToObject(root, "head",cJSON_CreateNumber(_msg[1]));//
    cJSON_AddItemToObject(root, "right", cJSON_CreateNumber(_msg[2]));//
   

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
 
    e_demo->Init_Pub("/sensors/vl53");//发布话题
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

    vl53_task(vl53_callback);//启动任务并设置回调函数

    return 0;
}