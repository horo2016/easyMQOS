#include "easy_mqos.h"
#include "Mqtt/client_shared.h"
#include <vector>
#include <unistd.h>
#include <sys/types.h>

#include "Mqtt/client_pub_sub.h"
#include "Mqtt/mosquitto.h"
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#include "method_parse.h"
#include "cJSON.h"

#include "getlidars.h"
using namespace std;

void lidar_callback(lidar_t _msg)
{
   
    char tmp_buf[0xff]={0};
    char topic_buf[0xff]={0};
    char  value_buf[4096]={0};
    char  send_buf[4096]={0};

	//memcpy(topic_buf,"/state/gps",sizeof("/state/gps"));
   // for(int i=0;i<360;i++)
    //printf("degree:%d:dis:%d \n",i,_msg.dis[i]);
   // printf("############### \n");
  //  printf("degeree display :%d ,%d \n",(int)ceil(anga) ,(unsigned short)round(distance));
    memcpy(value_buf,&_msg.timeStamp,4);
    memcpy(value_buf+4,_msg.dis,360*sizeof(unsigned short));
      
	//sprintf(topic_buf,"%s/state/gps",chargename);

#if defined MQTT_REMOTE_SERVER 
		sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
			sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif


	//system(send_buf);	

    publish_message(4+360*sizeof(unsigned short),(char*)value_buf);
 
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
 
    e_demo->Init_Pub("/sensors/lidar_node_pub");
    e_demo->Set_broker("127.0.0.1");
    e_demo->Set_config_Pub();

    lidar_task(lidar_callback);//启动任务并设置回调函数

    return 0;
}