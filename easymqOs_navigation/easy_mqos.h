#ifndef EASY_MQOS_H
#define EASY_MQOS_H

#include <string>
#include <vector>
#include "Mqtt/client_shared.h"
#include "Mqtt/mosquitto.h"
using namespace std;
class easymqos
{

public:
    easymqos();
	easymqos(int pub_or_sub );
	~easymqos();
 
//初始化系统
	bool Init_Sub(vector<string> _topic);
    bool Start_Sub(char *devid,void(*pfunc)(struct mosquitto *, void *, const struct mosquitto_message *));
    bool Init_Pub(char* _topic);
    bool Set_config_Pub();
    bool Set_broker(char* host);
    unsigned int  publish_messages(unsigned int len,string message);
    
public:
    vector<string> _topicStr;
  //  struct mosq_config *subscribeCfg;
   // struct mosq_config *publicCfg;
     struct mosq_config *cfg;
    int _topicCnt_sub;
     
};




#endif