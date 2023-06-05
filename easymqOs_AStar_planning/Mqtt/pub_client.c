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


#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include "mosquitto.h"
#include "client_shared.h"

#include "client_pub_sub.h"

#define STATUS_CONNECTING 0
#define STATUS_CONNACK_RECVD 1
#define STATUS_WAITING 2

/* Global variables for use in callbacks. See sub_client.c for an example of
 * using a struct to hold variables for use in callbacks. */
static char *topic = NULL;
static char *message = NULL;
static long msglen = 0;
static int qos = 0;
static int retain = 0;
static int mode = MSGMODE_NONE;
static int status = STATUS_CONNECTING;
static int mid_sent = 0;
static int last_mid = -1;
static int last_mid_sent = -1;
static bool connected = true;
static char *username = NULL;
static char *password = NULL;
static bool disconnect_sent = false;
static bool quiet = false;

static void my_connect_callback(struct mosquitto *mosq, void *obj, int result)
{
	int rc = MOSQ_ERR_SUCCESS;

	if(!result){
		switch(mode){
			case MSGMODE_CMD:
			case MSGMODE_FILE:
			case MSGMODE_STDIN_FILE:
				rc = mosquitto_publish(mosq, &mid_sent, topic, msglen, "mesage", qos, retain);
				break;
			case MSGMODE_NULL:
				rc = mosquitto_publish(mosq, &mid_sent, topic, 0, NULL, qos, retain);
				break;
			case MSGMODE_STDIN_LINE:
				status = STATUS_CONNACK_RECVD;
				break;
		}
		if(rc){
			if(!quiet){
				switch(rc){
					case MOSQ_ERR_INVAL:
						fprintf(stderr, "Error: Invalid input. Does your topic contain '+' or '#'?\n");
						break;
					case MOSQ_ERR_NOMEM:
						fprintf(stderr, "Error: Out of memory when trying to publish message.\n");
						break;
					case MOSQ_ERR_NO_CONN:
						fprintf(stderr, "Error: Client not connected when trying to publish.\n");
						break;
					case MOSQ_ERR_PROTOCOL:
						fprintf(stderr, "Error: Protocol error when communicating with broker.\n");
						break;
					case MOSQ_ERR_PAYLOAD_SIZE:
						fprintf(stderr, "Error: Message payload is too large.\n");
						break;
				}
			}
			mosquitto_disconnect(mosq);
		}
	}else{
		if(result && !quiet){
			fprintf(stderr, "%s\n", mosquitto_connack_string(result));
		}
	}
}

void my_disconnect_callback(struct mosquitto *mosq, void *obj, int rc)
{
	connected = false;
}

void my_publish_callback(struct mosquitto *mosq, void *obj, int mid)
{
	last_mid_sent = mid;
	if(mode == MSGMODE_STDIN_LINE){
		if(mid == last_mid){
			mosquitto_disconnect(mosq);
			disconnect_sent = true;
		}
	}else if(disconnect_sent == false){
		mosquitto_disconnect(mosq);
		disconnect_sent = true;
	}
}

static void my_log_callback(struct mosquitto *mosq, void *obj, int level, const char *str)
{
	printf("%s\n", str);
}

int load_stdin(void)
{
	long pos = 0, rlen;
	char buf[1024];
	char *aux_message = NULL;

	mode = MSGMODE_STDIN_FILE;

	while(!feof(stdin)){
		rlen = fread(buf, 1, 1024, stdin);
		aux_message = realloc(message, pos+rlen);
		if(!aux_message){
			if(!quiet) fprintf(stderr, "Error: Out of memory.\n");
			free(message);
			return 1;
		} else
		{
			message = aux_message;
		}
		memcpy(&(message[pos]), buf, rlen);
		pos += rlen;
	}
	msglen = pos;

	if(!msglen){
		if(!quiet) fprintf(stderr, "Error: Zero length input.\n");
		return 1;
	}

	return 0;
}

int load_file(const char *filename)
{
	long pos, rlen;
	FILE *fptr = NULL;

	fptr = fopen(filename, "rb");
	if(!fptr){
		if(!quiet) fprintf(stderr, "Error: Unable to open file \"%s\".\n", filename);
		return 1;
	}
	mode = MSGMODE_FILE;
	fseek(fptr, 0, SEEK_END);
	msglen = ftell(fptr);
	if(msglen > 268435455){
		fclose(fptr);
		if(!quiet) fprintf(stderr, "Error: File \"%s\" is too large (>268,435,455 bytes).\n", filename);
		return 1;
	}else if(msglen == 0){
		fclose(fptr);
		if(!quiet) fprintf(stderr, "Error: File \"%s\" is empty.\n", filename);
		return 1;
	}else if(msglen < 0){
		fclose(fptr);
		if(!quiet) fprintf(stderr, "Error: Unable to determine size of file \"%s\".\n", filename);
		return 1;
	}
	fseek(fptr, 0, SEEK_SET);
	message = malloc(msglen);
	if(!message){
		fclose(fptr);
		if(!quiet) fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}
	pos = 0;
	while(pos < msglen){
		rlen = fread(&(message[pos]), sizeof(char), msglen-pos, fptr);
		pos += rlen;
	}
	fclose(fptr);
	return 0;
}

void init_pub_config(struct mosq_config *cfg,char *devid)
{
	// memset(cfg, 0, sizeof(*cfg));
	// cfg->host = "192.168.56.101";
	// cfg->topic = "test";

	cfg->port = 1883;
	cfg->max_inflight = 20;
	cfg->keepalive = 60;
	cfg->clean_session = true;
	cfg->eol = true;
	cfg->protocol_version = MQTT_PROTOCOL_V31;


}
int clientpub_config_load(struct mosq_config *cfg, int pub_or_sub,char *devid)
{
	int rc;
	FILE *fptr;
	char line[1024];
	int count;
	char *loc = NULL;
	int len;
	char *args[3];
	char *env;

	args[0] = NULL;

	init_pub_config(cfg,devid);

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



	if(!cfg->host){
		cfg->host = "localhost";
	}
	return MOSQ_ERR_SUCCESS;
}
struct mosq_config _gcfg; 
int mainPub(struct mosq_config *cfgon, char *devid)
{
	
	int rc;
	
	int buf_len = 1024;
	int buf_len_actual;
	int read_len;
	int pos;
  
    _gcfg = *cfgon;
	
	rc = clientpub_config_load(&_gcfg, CLIENT_PUB,devid);
	if(rc){
		 printf("error :%d\n",rc);
		client_config_cleanup(&_gcfg);
         return 1;
	}
    printf("finnal public topic :%s ,port:%d\n",_gcfg.topic,_gcfg.port);
	topic = _gcfg.topic;
	message = _gcfg.message;
	msglen = _gcfg.msglen;
	qos = _gcfg.qos;
	retain = _gcfg.retain;
	mode = _gcfg.pub_mode;
	username = _gcfg.username;
	password = _gcfg.password;
	quiet = _gcfg.quiet;

	mosquitto_lib_init();

	if(client_id_generate(&_gcfg, "mosqpub")){
		return 1;
	}
   // printf("generate:%s \n",cfg.id);
}

int publish_message(unsigned int msglength,char *msg)
{
 
	struct mosquitto *mosq = NULL;
	int rc;
	int rc2;
//	printf("pubsh generate:%s \n",cfg.id);
	//printf("topic :%s ,port:%d\n",_gcfg.topic,_gcfg.port);
	mosq = mosquitto_new(_gcfg.id, true, NULL);
	if(!mosq){
		switch(errno){
			case ENOMEM:
				if(!quiet) fprintf(stderr, "Error: Out of memory.\n");
				break;
			case EINVAL:
				if(!quiet) fprintf(stderr, "Error: Invalid id.\n");
				break;
		}
		mosquitto_lib_cleanup();
		return 1;
	}
	if(_gcfg.debug){
		mosquitto_log_callback_set(mosq, my_log_callback);
	}
	mosquitto_connect_callback_set(mosq, my_connect_callback);
	mosquitto_disconnect_callback_set(mosq, my_disconnect_callback);
	mosquitto_publish_callback_set(mosq, my_publish_callback);

	if(client_opts_set(mosq, &_gcfg)){
		return 1;
	}
	rc = client_connect(mosq, &_gcfg);
	if(rc) return rc;



	rc2 = mosquitto_publish(mosq, &mid_sent, topic,msglength,msg, qos, retain);
	if(rc2){
		if(!quiet) fprintf(stderr, "Error: Publish returned %d, disconnecting.\n", rc2);
		mosquitto_disconnect(mosq);
	}
#if 0
	if(mode == MSGMODE_STDIN_LINE){
		mosquitto_loop_start(mosq);
	}

	do{
		if(mode == MSGMODE_STDIN_LINE){
			if(status == STATUS_CONNACK_RECVD){
				pos = 0;
				read_len = buf_len;
				while(fgets(&buf[pos], read_len, stdin)){
					buf_len_actual = strlen(buf);
					if(buf[buf_len_actual-1] == '\n'){
						buf[buf_len_actual-1] = '\0';
						rc2 = mosquitto_publish(mosq, &mid_sent, topic, buf_len_actual-1, buf, qos, retain);
						if(rc2){
							if(!quiet) fprintf(stderr, "Error: Publish returned %d, disconnecting.\n", rc2);
							mosquitto_disconnect(mosq);
						}
						break;
					}else{
						buf_len += 1024;
						pos += 1023;
						read_len = 1024;
						buf = realloc(buf, buf_len);
						if(!buf){
							fprintf(stderr, "Error: Out of memory.\n");
							return 1;
						}
					}
				}
				if(feof(stdin)){
					last_mid = mid_sent;
					status = STATUS_WAITING;
				}
			}else if(status == STATUS_WAITING){
				if(last_mid_sent == last_mid && disconnect_sent == false){
					mosquitto_disconnect(mosq);
					disconnect_sent = true;
				}
#ifdef WIN32
				Sleep(100);
#else
				usleep(100000);
#endif
			}
			rc = MOSQ_ERR_SUCCESS;
		}else{
			rc = mosquitto_loop(mosq, -1, 1);
		}
	}while(rc == MOSQ_ERR_SUCCESS && connected);

	if(mode == MSGMODE_STDIN_LINE){
		mosquitto_loop_stop(mosq, false);
	}

	if(message && mode == MSGMODE_FILE){
		free(message);
	}
#endif
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();

	if(rc){
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(rc));
	}
	return rc;

}