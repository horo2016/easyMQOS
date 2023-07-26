/******************************************************************************
* Copyright (c) 2007-2016, ZeroTech Co., Ltd.
* All rights reserved.
*******************************************************************************
* File name     : osp_syslog.c
* Description   : main source file for om application
* Version       : v1.0
* Create Time   : 2016/4/13
* Author   		: wangbo
* Modify history:
*******************************************************************************
* Modify Time   Modify person  Modification
* ------------------------------------------------------------------------------
*
*******************************************************************************/

/******************************* include *********************************/
#include <stdio.h>
#include <sys/time.h>
#include "stdlib.h"
#include "time.h"
#include "stdarg.h"
#include  <string.h>
#include "osp_syslog.h"
#include "osp_common.h"

log_type_level log_type[LOG_ID_MAX + 1] = {
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*BASE*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*CE*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*FC*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*CAMERA*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*GCS*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}, /*OM*/
	{SYSLOG_LOCAL_BUFFER, SYSLOG_DEBUG}/*OTHER*/
};

u8 log_buffer[LOG_BUFFER_LEN];
FILE *file_handle = NULL ;



/*******************************************************************************
* function name	: check_syslog_level
* description	: check system log level
* param[in] 	: log id ;level
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
s32 check_syslog_level (u8 log_id, u8 level)
{
	if (level > log_type[log_id].level){
		return 0;
	}

	return 1;
}

/*******************************************************************************
* function name	: sys_log
* description	:
* param[in] 	:
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
s32 sys_log (u8 * file, u32 line, u8 log_id, u8 level, u8 * info)
{
	u8 buf[sizeof (app_msg_log) + MAX_LOG_LEN];
	time_t cur_time;
	struct tm *tm_buf = NULL;
	u8 u8Type = 0 ;

	if (log_id > LOG_ID_MAX){
		printf ("syslog   mod_id is too big\n");
		return -1;
	}

	if (level > log_type[log_id].level){
		return 0;
	}

	u8Type = log_type[log_id].type;
	switch (u8Type){
		case SYSLOG_LOCAL_FILE:
		case SYSLOG_LOCAL_BUFFER:
			time (&cur_time);
			tm_buf = localtime (&cur_time);
			memset (buf, 0, sizeof (buf));
			sprintf ((char *)buf, "%d/%d/%d %02d:%02d:%02d, line %d,  %s", tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday, tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec, line, info);
			break;

		default:
			printf ("Syslog invalid type: %d for mod %d!\n", log_type[log_id].type, log_id);

			break;
	}

	printf ("%s", buf);

	return 0;
}

/*******************************************************************************
* function name	: logwrite
* description	:
* param[in] 	:
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
void logwrite (char * file, const s32 line, u8 log_id, u8 level, const u8* info, ...)
{
	va_list argl;
	u8 buf[LOG_BUFFER_LEN];
	s32 size;

	memset (buf, 0, sizeof (buf));
	va_start (argl, info);
	size = vsprintf ((char *)buf, (char *)info, argl);
	va_end (argl);

	if (size > 0){
		if (size > MAX_LOG_LEN + 96){
			SYSLOG (log_id, SYSLOG_ERR, "logwrite:Invalid length: %d!\n", size);
			return;
		}
		sys_log ((u8 *) file, line, log_id, level, buf);
	}
	return;
}

/*******************************************************************************
* function name	: log_debug
* description	:
* param[in] 	:
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
int log_debug(const char * func, s32 line,u8 level,char *data,int len,const char * fmt,...)
{
	char *leve[30] = {"TRACE","DEBUG","\033[32mINFO\033[0m","\033[33mWARN\033[0m","\033[31mERROR\033[0m"} ;
	time_t cur_time;
	struct tm *tm_buf = NULL;
	struct timeval  tv;
    struct timezone tz;
	s32 size;
	va_list argl;
	int i = 0 ;

	if(level < LOG_LEV|| level > LOG_ERR)
		return -1;

	time (&cur_time);
	tm_buf = localtime (&cur_time);
	gettimeofday(&tv, &tz);
	printf ("(%04d%02d%02d %02d:%02d:%02d.%03ld)%20s[%4d]<%5s>:  ",\
					tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday, \
					tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec,tv.tv_usec/1000, \
					func,line,leve[level]);
	va_start (argl, fmt);
	size = vprintf (fmt, argl);
	va_end (argl);

	if(data != NULL){
		for( i = 0 ;i < len ;i++){
			printf("0x%02x ",data[ i ]) ;
		}
		printf("\n") ;
	}

	return size;
}
/*******************************************************************************
* function name	: log_file_debug
* description	:
* param[in] 	:
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
int log_file_debug(const char * func, s32 line,u8 level,char *data,int len,const char * fmt,...)
{
	char *leve[30] = {"TRACE","DEBUG","\033[32mINFO\033[0m","\033[33mWARN\033[0m","\033[31mERROR\033[0m"} ;
	time_t cur_time;
	struct tm *tm_buf = NULL;
	s32 size;
	va_list argl;
	int i = 0 ;

	if(level < LOG_LEV|| level > LOG_ERR)
		return -1;

	if(file_handle == NULL){
		file_handle = fopen(FILE_LOG_PATH,"a") ;
		if(file_handle == NULL){
			printf("open %s failed,cannot create log file\n",FILE_LOG_PATH) ;
			return -1 ;
		}
	}

	time (&cur_time);
	tm_buf = localtime (&cur_time);
	fprintf (file_handle,"(%04d%02d%02d %02d:%02d:%02d)%20s[%4d]<%5s>:  ",tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday, tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec,func,line,leve[level]);
	va_start (argl, fmt);
	size = vfprintf (file_handle,fmt, argl);
	va_end (argl);

	if(data != NULL){
		for( i = 0 ;i < len ;i++){
			fprintf(file_handle,"0x%02x ",data[ i ]) ;
		}
		fprintf(file_handle,"\n") ;
	}

	return size;
}

/*******************************************************************************
* function name	: syslog_op_level
* description	:
* param[in] 	:
* param[out] 	: none
* return 		: 0:success;1:failed
*******************************************************************************/
s32 syslog_op_level (u8 log_id, u8 type, u8 level)
{
	if (log_id >= MOD_ID_ARM_MAX){
		printf ("mod_id is too big\n");
		return -1;
	}

	log_type[log_id].type = type;
	log_type[log_id].level = level;

	return 0;
}


