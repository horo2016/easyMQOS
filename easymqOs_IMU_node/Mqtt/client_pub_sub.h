#ifndef CLIENT_PUB_SUB_H
#define CLIENT_PUB_SUB_H
#ifdef __cplusplus
extern "C" {
#endif

//extern int mainSub(int argc, char * argv [ ]);
extern   int mainSub(struct mosq_config *cfgon , char *devid,void(*pfunc)(struct mosquitto *, void *, const struct mosquitto_message *));
extern   int mainPub(struct mosq_config *cfgon, char *devid);
extern int publish_message(unsigned int msglength,char *msg);
#ifdef __cplusplus
}
#endif

#endif 
