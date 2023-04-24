/******************************************************************************
* Copyright (c) 2007-2016, ZeroTech Co., Ltd.
* All rights reserved.
*******************************************************************************
* File name     : osp_common.h
* Description   : common function interface
* Version       : v1.0
* Create Time   : 2016/4/13
* Author   		: wangbo
* Modify history:
*******************************************************************************
* Modify Time   Modify person  Modification
* ------------------------------------------------------------------------------
*
*******************************************************************************/

/******************************** Head File Begin **************************/
#ifndef OSP_COMMON_H_
#define OSP_COMMON_H_
/******************************** include ********************************/
#include <stdlib.h>
#include <stdio.h>


/******************************** Defines ********************************/
#define OSP_OK                         0
#define OSP_ERROR                      -1

#define MAX_APP_TASK_NUM               3

#define CAPTURE_MODE                       2
#define SOCKET_UDP                            4
//added by wangbo 20160406
#define HEARTBEAT_MODE	               3
#define LOOP_MODE					   1
#define SOCKET_UNIX_TCP 		       5
#define CAMER_UNIX_KEY					"/tmp/camera_socket.key"

#define APP_TSK_DEFAULT_PRIO           5
#define ENC_TSK_DEFAULT_PRIO           4
/*socket port*/
#define SOCKET_RSV_CAMERA_PORT         7087
#define SOCKET_RCV_GCS_PORT            7088

#define MAX_MSG_BUFSIZE                1024
#define MAX_PRINT_BUFSIZE              (MAX_MSG_BUFSIZE * 3)

#define GCS_HEART_INNERVAL             2000
#define FC_HEART_INNERVAL              1000
#define GCS_SLOW_INNERVAL              1000

#define FC_GET_STATUS_INNERVAL         50

#define SIMULATE_TEST

#define REMAIN_COUNT					5

//added by wangbo 20160406
#define IP_POOL							10
#define HEATBEAT_TIMEOUT 				10	//5s 1s == 2

#define TIMEOUT 						1
#define PACKET_NUM						2
#define PACKET_TIMEOUT					5
#define TIMER_NUM						20
#define TRACKLOG_PATH					"."
#define OTA_BUSY						1
#define OTA_IDLE						0
#define OTA_INIT						-1
#define OTA_FREESPACE_REQURE			200 //Mb

#define	OTA_RET_INSUFFICIENT_DISK_SPACE 	  -1
#define OTA_RET_LOW_BATTERY					  -2
#define OTA_RET_PACKET_NOT_EXIST			  -3
#define OTA_RET_DECOMPRESS_FAILED			  -4
#define OTA_RET_FILE_DAMAGE					  -5
#define OTA_RET_SYS_IS_NEW					  -6

#define MULTI_CONECT_TIMEOUT					2
#define MULTI_CONECT_DELAY						3

#define PRODUCT_NAME					"dobby1"
#define PRODUCT_MODE					"8074"

//#define DUMMY_DEBUG
/******************************** Typdef ************************************/
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;
typedef unsigned long u64;

typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef long long32;
typedef long long long64;
typedef float f16;
typedef double d64;

typedef s32 (*TaskInit) (void *var);
typedef s32 (*TaskMsgHandle) (void *msgAddr, s32 msgSize);

typedef struct IP_ST{
	int ipaddr ;
	int port	;
	int times ;
	int delay ;
	int flag ;
}ipinfo ;

typedef struct AppTaskAttr_tag
{
	s32 mod_id;
	s32 msg_mod;
	s32 msg_id;
	TaskInit task_init;
	TaskMsgHandle task_handle;
	s32 timeout ;		//added by wangbo
	s32 task_pri;
} app_task_attr;

/*msg type stuct*/



/*Mod Id definition, Do not change this content please!*/
enum
{
	MOD_ID_ARM_BASE = 0,
	MOD_ID_CEMSGMNG,
	MOD_ID_CAMERA,
	MOD_ID_SMIF,
	MOD_ID_GCS,
	MOD_ID_LOG,
	MOD_ID_WIFI,
	MOD_ID_FTP,
	MOD_ID_OM,
	MOD_ID_TRACKLOG,
	MOD_ID_FC_CMD,
	MOD_ID_OTA,
	MOD_ID_ARM_MAX = MOD_ID_OTA,
};

/****************************  externs  ******************************/
//added by wangbo 20160405
extern ipinfo ip_pool[IP_POOL] ;
extern char frist_getip_flag ;
extern char frist_sendcamera_flag ;
extern int	socket_snd_gcs_port ;
extern s32 times_count ;
extern s32 sockfd_camera;
extern s32 sockfd_gcs;
extern u32 gcs_main_ip;					// = 0xC0A80010;
extern int ota_busy_flag;


extern s32 OspIsTimerExist (s32 timer_id);
extern void second_timer() ;
extern void multi_client_manager() ;
extern void osp_print_buffer (u8 logid, u8 loglevel, u8 * detail, u8 * buffer, s32 len) ;
//extern long long NOW(void) ;
#endif /*OSP_COMMON_H_ */

/******************************  File end  **********************************/
