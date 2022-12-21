/*
Copyright (c) 2009-2014 Roger Light <roger@atchoo.org>

All rights reserved. This program and the accompanying materials
are made available under the terms of the Eclipse Public License v1.0
and Eclipse Distribution License v1.0 which accompany this distribution.
 
The Eclipse Public License is available at
   http://www.eclipse.org/legal/epl-v10.html
and the Eclipse Distribution License is available at
  http://www.eclipse.org/org/documents/edl-v10.php.
 
Contributors:
   Roger Light - initial implementation and documentation.
*/

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>


#include "mosquitto.h"
#include "client_shared.h"
#include "client_pub_sub.h"


 
#if 0
static void my_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
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
#endif 
static void my_connect_callback(struct mosquitto *mosq, void *obj, int result)
{
	int i;
	struct mosq_config *cfg;

	assert(obj);
	cfg = (struct mosq_config *)obj;

	if(!result){
		for(i=0; i<cfg->topic_count; i++){
			mosquitto_subscribe(mosq, NULL, cfg->topics[i], cfg->qos);
		}
	}else{
		if(result && !cfg->quiet){
			fprintf(stderr, "%s\n", mosquitto_connack_string(result));
		}
	}
}

void my_subscribe_callback(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos)
{
	int i;
	struct mosq_config *cfg;

	assert(obj);
	cfg = (struct mosq_config *)obj;

	if(!cfg->quiet) printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for(i=1; i<qos_count; i++){
		if(!cfg->quiet) printf(", %d", granted_qos[i]);
	}
	if(!cfg->quiet) printf("\n");
}

static void my_log_callback(struct mosquitto *mosq, void *obj, int level, const char *str)
{
	printf("%s\n", str);
}


void init_sub_config(struct mosq_config *cfg,char *devid)
{
    // char tmpBuf[0xff]={0};
	// memset(tmpBuf,0,255);
    // sprintf(tmpBuf,"%s/download/waypoints",devid);
	// memset(cfg, 0, sizeof(*cfg));


	// printf("topic:%s \n",tmpBuf);
	// cfg->host = "192.168.56.101";
	
	// cfg->topic_count =2 ;
	// cfg->topics = realloc(cfg->topics, cfg->topic_count*sizeof(char *));
	// cfg->topics[cfg->topic_count-2] ="test";
	// cfg->topics[cfg->topic_count-1] = strdup(tmpBuf);// "00000000b9065e37/download/control";//strdup(tmpBuf);//strdup(argv[i+1]);
	
	cfg->port = 1883;
	cfg->max_inflight = 20;
	cfg->keepalive = 60;
	cfg->clean_session = true;
	cfg->eol = true;
	cfg->protocol_version = MQTT_PROTOCOL_V31;
}

int clientsub_config_load(struct mosq_config *cfg, int pub_or_sub,char *devid)
{
	int rc;
	FILE *fptr;
	char line[1024];
	int count;
	char *loc = NULL;
	int len;

	char *env;


	init_sub_config(cfg,devid);

	if(cfg->will_payload && !cfg->will_topic){
		fprintf(stderr, "Error: Will payload given, but no will topic given.\n");
		return 1;
	}
	if(cfg->will_retain && !cfg->will_topic){
		fprintf(stderr, "Error: Will retain given, but no will topic given.\n");
		return 1;
	}
	if(cfg->password && !cfg->username){
		if(!cfg->quiet) fprintf(stderr, "Warning: Not using password since username not set.\n");
	}

	if(pub_or_sub == CLIENT_SUB){
		if(cfg->clean_session == false && (cfg->id_prefix || !cfg->id)){
			if(!cfg->quiet) fprintf(stderr, "Error: You must provide a client id if you are using the -c option.\n");
			return 1;
		}
		if(cfg->topic_count == 0){
			if(!cfg->quiet) fprintf(stderr, "Error: You must specify a topic to subscribe to.\n");
			return 1;
		}
	}

	if(!cfg->host){
		cfg->host = "localhost";
	}
	return MOSQ_ERR_SUCCESS;
}


int mainSub(struct mosq_config *cfgon, char *devid,void(*pfunc)(struct mosquitto *, void *, const struct mosquitto_message *))
{
	struct mosq_config cfg = *cfgon;
	struct mosquitto *mosq = NULL;
	int rc;
	
	rc = clientsub_config_load(&cfg, CLIENT_SUB,devid);
	if(rc){
		client_config_cleanup(&cfg);
		
		return 1;
	}
    
	mosquitto_lib_init();

	if(client_id_generate(&cfg, "mosqsub")){
		return 1;
	}

	mosq = mosquitto_new(cfg.id, cfg.clean_session, &cfg);
	if(!mosq){
		switch(errno){
			case ENOMEM:
				if(!cfg.quiet) fprintf(stderr, "Error: Out of memory.\n");
				break;
			case EINVAL:
				if(!cfg.quiet) fprintf(stderr, "Error: Invalid id and/or clean_session.\n");
				break;
		}
		mosquitto_lib_cleanup();
		return 1;
	}
	if(client_opts_set(mosq, &cfg)){
		return 1;
	}
	if(cfg.debug){
		mosquitto_log_callback_set(mosq, my_log_callback);
		mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);
	}
	mosquitto_connect_callback_set(mosq, my_connect_callback);
//	mosquitto_message_callback_set(mosq, my_message_callback);
	mosquitto_message_callback_set(mosq,pfunc);

	rc = client_connect(mosq, &cfg);
	if(rc) return rc;


	rc = mosquitto_loop_forever(mosq, -1, 1);

	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();

	if(cfg.msg_count>0 && rc == MOSQ_ERR_NO_CONN){
		rc = 0;
	}
	if(rc){
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(rc));
	}
	return rc;
}

