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
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
/*pthread*/
#include "sys/ipc.h"
#include "sys/msg.h"
#include "pthread.h"
#include <arpa/inet.h>
#include <net/if.h>
#include "method_parse.h"
#include "cJSON.h"
using namespace std;

void my_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
easymqos::easymqos()
{
    
}
easymqos::easymqos(int pub_or_sub)
{
    cout << "调用构造函数："<< pub_or_sub << endl;

      cfg = new  mosq_config();
}

easymqos::~easymqos()
{
    cout << "释放内存" << endl;
   if(cfg)
    delete(cfg);
}

bool easymqos::Init_Sub(vector<string> _topic )
{
    
    _topicStr =_topic;
    _topicCnt_sub = _topicStr.size();
   
    cfg->topic_count = _topicCnt_sub;
    cfg->topics = (char **)realloc(cfg->topics, cfg->topic_count*sizeof(char *));
    for(int i =0;i<_topicCnt_sub;i ++)
        cfg->topics[i] =(char*)_topicStr[i].c_str();;
return true;
}
bool easymqos::Init_Pub(char* _topic)
{
    

   // cfg->topic = (char *)realloc(cfg->topic, _topic.size()*sizeof(char));
     cfg->topic =_topic;
    
   return true;
   
}
bool easymqos::Start_Sub()
{
 
    mainSub(cfg,"12345",my_message_callback);
return true;
}
bool easymqos::Set_config_Pub()
{
   
    cout << "1 publicCfg->topic :"<<  cfg->topic<< endl;
     cout << "2publicCfg->host :"<<  cfg->host  << endl;
    mainPub(cfg,"12345");
    return true;
}

 bool easymqos::Set_broker(char* host)
{
 
   cfg->host  = host;
	 return true;
  
}
unsigned int  easymqos::publish_messages(unsigned int len,string message)
{
  publish_message(len,(char *)message.c_str());

}
void parse_cjson(char *a)
{
	
	cJSON *root=cJSON_Parse(a); 
	cJSON *type=cJSON_GetObjectItem(root,"type"); 

	printf("type value =%s\n",type->valuestring);
	if(!strcmp("1",type->valuestring)){
			cJSON *lnt=cJSON_GetObjectItem(root,"lnt");
			printf("lnt value =%s\n",lnt->valuestring);
			cJSON *lat=cJSON_GetObjectItem(root,"lat");
			printf("lat value =%s\n",lat->valuestring);

			push_waypoint(atof(lnt->valuestring),atof(lat->valuestring));
		}	if(!strcmp("2",type->valuestring)){

      		save_waypoint();
		}	if(!strcmp("3",type->valuestring)){
			clear_waypoint();
		}
	cJSON_Delete(root);

}
bool process_messages = true;
int msg_count = 0;

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
 

    
 
