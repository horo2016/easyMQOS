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


#include <string.h>

/*pthread*/
#include "sys/ipc.h"
#include "sys/msg.h"
#include "pthread.h"
#include <arpa/inet.h>
#include <net/if.h>
#include "method_parse.h"
#include "cJSON.h"
#include "easy_mqos.h"
#include "gps_main.h"

#include "coordinate_sys.h"
using namespace std;

void gps_filter_callback(Location gpsval,float h,float vel)
{
    char tmp_buf[0xff]={0};
    char topic_buf[0xff]={0};
    char  value_buf[4096]={0};
    char  send_buf[4096]={0};

	//memcpy(topic_buf,"/state/gps",sizeof("/state/gps"));


    cJSON * root =  cJSON_CreateObject();
  if(!root) {
         printf("get root faild !\n");
     }else printf("get root success!\n");

    cJSON_AddItemToObject(root, "\"isvalid\"", cJSON_CreateNumber(1));//在web中解析需要 去掉其中的引号 先标记mark   20230523
    cJSON_AddItemToObject(root, "\"lonti\"", cJSON_CreateNumber(gpsval.lng));//ìí?óname?úμ?
    cJSON_AddItemToObject(root, "\"lati\"",cJSON_CreateNumber(gpsval.lat));//ìí?óname?úμ?
    cJSON_AddItemToObject(root, "\"gpsheading\"", cJSON_CreateNumber(h));//
    cJSON_AddItemToObject(root, "\"gpsvelocity\"",cJSON_CreateNumber(vel)); 
#if 0
    cJSON_AddItemToObject(root, "\"heading\"", cJSON_CreateNumber((short)heading));
    cJSON_AddItemToObject(root, "\"roll\"", cJSON_CreateNumber((short)rollAngle));
    cJSON_AddItemToObject(root, "\"pitch\"",cJSON_CreateNumber((short)pitchAngle));
    cJSON_AddItemToObject(root, "\"cpuload\"", cJSON_CreateNumber((char)cpuPercentage));
    cJSON_AddItemToObject(root, "\"cputemp\"", cJSON_CreateNumber((char)cpuTemperature));
    cJSON_AddItemToObject(root, "\"fusionheading\"", cJSON_CreateNumber((short)fusionheading));
  //  cJSON_AddItemToObject(root, "\"wifisignal\"", cJSON_CreateNumber(wifiSignalStrength));
  //  cJSON_AddItemToObject(root, "\"velspeed\"", cJSON_CreateNumber((int)velspeed));
  //  cJSON_AddItemToObject(root, "\"angspeed\"", cJSON_CreateNumber(angspeed));
   // cJSON_AddItemToObject(root, "\"targetheading\"", cJSON_CreateNumber((int)targetHeading));
   // cJSON_AddItemToObject(root, "\"distance\"", cJSON_CreateNumber((int)waypointRange));
 //   cJSON_AddItemToObject(root, "\"nextwaypoint_lon\"", cJSON_CreateNumber(waypointlongitude));
 //   cJSON_AddItemToObject(root, "\"nextwaypoint_lat\"", cJSON_CreateNumber(waypointlatitude));
   // mqtt_publish(tmp_buf,cJSON_Print(root));
   	#endif
    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));
      
	//sprintf(topic_buf,"%s/state/gps",chargename);

#if defined MQTT_REMOTE_SERVER 
		sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
			sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif


	//system(send_buf);	

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
 
    e_demo->Init_Pub("/sensors/gps_filter_pub");
    e_demo->Set_broker("192.168.56.101");
    e_demo->Set_config_Pub();

    gps_task(gps_filter_callback);//启动任务并设置回调函数

    return 0;
}
