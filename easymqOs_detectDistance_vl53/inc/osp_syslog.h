/******************************************************************************
* Copyright (c) 2007-2016, ZeroTech Co., Ltd.
* All rights reserved.
*******************************************************************************
* File name     : osp_syslog.h
* Description   : syslog interface
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
#ifndef OSP_SYSLOG_H
#define OSP_SYSLOG_H

/*******************************  include  *********************************/
#include "osp_common.h"


#ifdef __cplusplus
extern "C" {
#endif

/********************************  Typedef  **********************************/
enum SYSLOG_TYPE
{
	SYSLOG_LOCAL_BUFFER,		/*Print out */
	SYSLOG_LOCAL_FILE,			/*Record to local file */
};

enum SYSLOG_LEVEL
{
	SYSLOG_CRITIC,
	SYSLOG_ALERT,
	SYSLOG_ERR,
	SYSLOG_WARN,
	SYSLOG_INFO,
	SYSLOG_DEBUG,
	SYSLOG_FUNC,
	SYSLOG_TRACE,
};

enum PRINT_LEVEL
{
	LOG_TRACE = 0 ,
	LOG_DEBUG,
	LOG_INFO,
	LOG_WARN,
	LOG_ERR,
} ;

enum
{
	LOG_ID_BASE = 0,
	LOG_ID_CE,
	LOG_ID_FC,
	LOG_ID_CAMERA,
	LOG_ID_GCS,
	LOG_ID_OM,
	LOG_ID_OTHERS,
	LOG_ID_MAX = LOG_ID_OTHERS
};

typedef struct LogTypeAndLevel
{
	u8 type;
	u8 level;
	u8 rsv[2];
} log_type_level;

typedef struct
{
	u8 type;
	u8 rsv;
	u16 str_len;
	u8 *trace_str;
} app_msg_log;

/******************************  defines ************************************/
#define MAX_LOG_LEN                    512
#define  LOG_BUFFER_LEN                1024
#define SYSLOG(log_id, logLev, fmt, arg...)  sprintf(((char *)log_buffer), fmt, ##arg); \
    logwrite(__FILE__,__LINE__,log_id,logLev,(log_buffer));

#define LOG_LEV			LOG_TRACE
#define FILE_LOG_PATH		"/home/app/gpscarbot.log"

//#define STORE_FILE
#ifndef STORE_FILE
#define DEBUG(loglev,fmt,...)  log_debug(__func__,__LINE__,(loglev),NULL,0,(fmt),##__VA_ARGS__);
#define DEBUG_DATA(loglev,data,len,fmt,...) log_debug(__func__,__LINE__,(loglev),(data),(len),(fmt),##__VA_ARGS__);
#else
#define DEBUG(loglev,fmt,...)  log_file_debug(__func__,__LINE__,(loglev),NULL,0,(fmt),##__VA_ARGS__);
#define DEBUG_DATA(loglev,data,len,fmt,...) log_file_debug(__func__,__LINE__,(loglev),(data),(len),(fmt),##__VA_ARGS__);

#endif

/****************************  Extern  ******************************/
extern u8 log_buffer[LOG_BUFFER_LEN];

extern s32 task_log_init (void *var);
extern s32 task_log_handle (void *msgAddr, s32 msgSize);
extern void logwrite (char * file, const s32 line, u8 log_id, u8 level, const u8 * info, ...);
extern int log_debug(const char * func, s32 line,u8 level,char *data,int len,const char * fmt,...);
extern int log_file_debug(const char * func, s32 line,u8 level,char *data,int len,const char * fmt,...) ;
extern log_type_level log_type[LOG_ID_MAX + 1];

#ifdef __cplusplus
}
#endif

#endif

/******************************  File End **********************************/
