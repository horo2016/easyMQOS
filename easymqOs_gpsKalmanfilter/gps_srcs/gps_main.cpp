/*
* serial port programming
*/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <sys/time.h>
#include <time.h>	
#include <sys/ipc.h>
#include <sys/shm.h>
#include "gps_apply.h"

#include "coordinate_sys.h"
#include "cJSON.h"
#include "serial.h"


#include "gps_main.h"


// void gps_init(double noise);
// void gps_update(double lat, double lon, double seconds_since_last_timestep);
// void gps_read(double* lat, double* lon);
// bool gpsparser(char* data, double* lon, double* lat, double* HDOP, int* numSV)
#define MQTT_SIP "gpscar.xiaovdiy.cn"

#define MYKEY 1234
#define BUF_SIZE 1024


using namespace std;
const double timestep = 0.100;
const double noise = 1.000;
typedef struct
{    
    char isvalid;
    Location gpsInf;
    float gpsheading;
    float gpsvelocity;
}use_shared;
unsigned short point1 = 0;
unsigned short  point_start = 0;

_SaveData Save_Data;


int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0) {
		perror("SetupSerial 1");
		return -1;
	}
	memset(&newtio, 0, sizeof(newtio));

	/*
	* Enable the receiver and set local mode...
	*/
	newtio.c_cflag |= CLOCAL | CREAD;

	/*
	* Set Data Bits
	*/
	newtio.c_cflag &= ~CSIZE;
	switch (nBits) {
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	/*
	* Set Parity Bit
	*/
	switch (nEvent) {
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}

	/*
	*	Set Stop Bit
	*/
	if (nStop == 1)
		newtio.c_cflag &= ~CSTOPB;
	else if (nStop == 2)
		newtio.c_cflag |= CSTOPB;
	/*
	*	Set BaudRate
	*/
	switch (nSpeed) {
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}

	/*
	* Disable Hardware Flow Control
	*/
	newtio.c_cflag &= ~CRTSCTS;

	/*
	* Disable Software Flow Control
	*/
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

	/******************** Noncanonical Mode***********************/
	// 
	// // Set Min Character & Waiting Time
	// 
	// newtio.c_cc[VTIME] = 1; //segement group
	// newtio.c_cc[VMIN] = 68;

	
	// // use raw input and output
	
	// newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
	// newtio.c_oflag &= ~OPOST; /*Output*/
	/********************************************/

	/******************** Canonical Mode***********************/
	/*
	 Raw 妯″紡杈撳嚭.
	*/
 	newtio.c_oflag = 0;
 
	/*
	  ICANON  : 鑷磋兘鏍囧噯杈撳叆, 浣挎墍鏈夊洖搴旀満鑳藉仠鐢�, 骞朵笉閫佸嚭淇″彿浠ュ彨鐢ㄧ▼搴�
	*/
 	newtio.c_lflag = ICANON;
 	/********************************************/

	/*
	* Clear Input Queue
	*/
	tcflush(fd, TCIFLUSH); //TCOFLUSH,TCIOFLUSH

	/*
	* Enforce Now
	*/
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
		perror("com set error");
		return -1;
	}

	printf("set done!\n");
	return 0;
}

int open_port(int fd, int comport) {
	char serialDev[256]={0};
	strcpy(serialDev, GPSDEVICE);
	/*
	if (comport == 1) {
		fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd) {
			perror("Can't Open Serial Port");
			return (-1);
		}
		else
			printf("open ttyAMA0 ......\n");
	}
	else if (comport == 2) {
		fd = open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_NDELAY);

	}
	else if (comport == 3) {
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	
	}
	else if (comport == 4) {
		fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	
	
	}*/
	
	fd = open(serialDev, O_RDWR | O_NOCTTY | O_NDELAY);
	if (-1 == fd) {
		perror("Can't Open Serial Port");
		return (-1);
	}
	else
		printf("open ttyAMA0 %s ......\n",serialDev);
	
	
	/*
	*	Block the serial port
	*/
	if (fcntl(fd, F_SETFL, 0) < 0)
		printf("fcntl failed!\n");
	else
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	
	/*
	*	Test whether port is attached to terminal device
	*/
	if (isatty(STDIN_FILENO) == 0)
		printf("standard input is not a terminal device\n");
	else
		printf("isatty success!\n");
	
	printf("fd-open=%d\n", fd);
	return fd;
}

/*******************************************************************************
* function name	: parseGpsBuffer
* description	: 
*
* param[in] 	: 
* param[out] 	: none
* return 		: none
*******************************************************************************/
void parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
	char i = 0;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;
	//	printf("**************\r\n");
		//printf(Save_Data.GPS_Buffer);

		for (i = 0 ; i <= 9; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					printf("error");	//解析错误
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W
						case 7:memcpy(Save_Data.earthSpeed, subString, subStringNext - subString);
						//	  DEBUG(LOG_DEBUG, "speed:%s \n",Save_Data.earthSpeed);
						break;
						case 8:memcpy(Save_Data.earthHeading, subString, subStringNext - subString);
							//  DEBUG(LOG_DEBUG, "head:%s \n",Save_Data.earthHeading);
						break;
						case 9:memcpy(Save_Data.UTCDate, subString, subStringNext - subString);
						//	DEBUG(LOG_DEBUG, "UTCDate:%s \n",Save_Data.UTCDate);
						break;
						default:break;
					}

					subString = subStringNext;
					Save_Data.isParseData = true;
					if(usefullBuffer[0] == 'A')
						Save_Data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false;

				}
				else
				{
						//解析错误
				}
			}


		}
	}
}
/*******************************************************************************
* function name	: parseGpsBuffer
* description	: 
*
* param[in] 	: 
* param[out] 	: none
* return 		: none
*******************************************************************************/
int gps_Data_Deal(unsigned char *datv,int length)
{
	unsigned char Res;
	unsigned char i=0;
	
    //printf("igps rec len:%d  \n",length);
    for(i=0;i<length;i++)
   	{       
	//	printf("%c",datv[i]);
	    if(datv[i]=='\n')	
			break;
    }
	point1 =i;
	//DEBUG(LOG_DEBUG,"gps rec total:%d \n",i);
	 for(i=0;i<length;i++)
   	{
        if(datv[i]=='G')
        	{
		 	  point_start=i;
	//		DEBUG(LOG_DEBUG,"gps rec start:%d  \n",point_start);
			break;
        	}
    }

       if((datv[point_start] == 'G') && (datv[3+point_start] == 'M') && (datv[point_start+4] == 'C'))//GPRMC/GNRMC

	if((datv[point_start] == 'G') && (datv[3+point_start] == 'M') && (datv[point_start+4] == 'C'))//确定是否收到"GPRMC/GNRMC"这一帧数据
	{
		 
	 //   DEBUG(LOG_DEBUG,"gps head analysis ok  \n");
		if(datv[point1] == '\n')									   
		{
		 //   DEBUG(LOG_DEBUG,"gps rec end:%d  \n",point1);
			memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
			memcpy(Save_Data.GPS_Buffer, datv+point_start, point1); 	//保存数据
			Save_Data.isGetData = true;
			point1 = 0;
			point_start =0;
			//memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空				
		}	
				
	}

     parseGpsBuffer();
    // printGpsBuffer();


}
void set_callback(Location gpsval,float h,float vel, void(*pfunc)(Location ,float,float))
{

	(*pfunc)(gpsval ,h,vel);
	return ;
}
int gps_task(void(*pfunc)(Location ,float,float))
{
	int fd = -1;
	int nread,nwrite;
	char sendbuff[] = "$123.42675,12344.23454\n";
	char readbuff[515];
	long cnt = 0;

	double lat,lon,lat_old=118.0,lon_old = 32.0;
	double lat_filt,lon_filt,lat_filt_old = 118.0,lon_filt_old=32.0;
	double HDOP = 3.0;
	int numSV = 0;
    float fusion_headingl;
	time_t now;
	struct tm *timenow;
	struct timeval  tick;
	long sec;
	Location loc_coor;

   
	// save the raw data and filtered data
	ofstream file1,file2;
	file1.open ("data/raw_data.txt", ios::out | ios::app);
	if(!file1.is_open()){
		cerr<<"open file raw_data.txt failed"<<endl;
		return 1;
	}
	
	file2.open("data/filtered_data.txt",ios::out | ios::app);
	if(!file2.is_open()){
		std::cerr<<"open file filtered_data.txt failed"<<std::endl;
		return 1;
	}
    
      char _buffer[0xff]={0};
	//   read_ID_fromSn(_buffer);
	//   memcpy(chargename,_buffer,16);
        //   chargename[16]='\0';
	
	// set precision
	file1.setf(ios::fixed);
	file1.precision(7);
	file2.setf(ios::fixed);
	file2.precision(7);

	if ((fd = open_port(fd, 2)) < 0) {
		perror("open_port error");
		return 1;
	}
	if (set_opt(fd, 9600, 8, 'N', 1) < 0) {
		perror("set_opt error");
		return 1;
	}
	printf("Starting! ......\n\n");

	//Test serial port
	nwrite = write(fd, sendbuff, sizeof(sendbuff));        //鍐欎覆鍙�  
    if(nwrite < 0){  
      	perror("write error");
    }
    // gps init
    gps_init(noise);

   	while(1){
   		nread = read(fd, readbuff, sizeof(readbuff));     //璇讳覆鍙ｆ暟鎹�  
   		if(nread > 0){
				gps_Data_Deal((unsigned char *)readbuff, nread);
    
				if (Save_Data.isParseData)
				{
					Save_Data.isParseData = false;
					
					if(Save_Data.isUsefull)
					{
						
						Save_Data.isUsefull = false;
						
						{
			      											
							lon = degree_minute2dec_degrees(atof(Save_Data.longitude));
							lat = degree_minute2dec_degrees(atof(Save_Data.latitude));

			      			gps_update(lat,lon,timestep);
			      			gps_read(&lat_filt,&lon_filt);
			      			time(&now);
			      			timenow = localtime(&now);
			      			gettimeofday(&tick,NULL);
			      			sec = tick.tv_usec/10000;
			      			double dist = get_distance(lat_old,lon_old,lat,lon);
			      			double dist1 = get_distance(lat_filt_old,lon_filt_old,lat_filt,lon_filt);
			      			lat_old = lat, lon_old = lon;
			      			lat_filt_old = lat_filt, lon_filt_old = lon_filt;
			      			printf("%2d:%2d:%2d:%2ld\tlat:  %f\tlon:  %f\t\tbias: %f\tbias2: %f\n",\
			      				timenow->tm_hour,timenow->tm_min,timenow->tm_sec,sec,lat,lon,dist,dist1);
			      			printf("        \tlatf: %f\tlonf: %f\t\tHDOP:%f\tnumSV:%d\n\n", lat_filt,lon_filt,HDOP,numSV);
			      			file1<<lat<<" "<<lon<<endl;
			      			file2<<lat_filt<<" "<<lon_filt<<endl;
							 //Location loc_coor_filter;
							 //(lon_filt,lat_filt);	
							 //myGps_filter(lat_filt,lon_filt, atof(Save_Data.earthSpeed)*1.852/3.6, \
							 atof(Save_Data.earthHeading),heading,&fusion_headingl);
							 //printf("        \tmylatf: %f\tmylonf: %f\t\tfusion_headingl:%f gpsheading:%f imuheading: %f\n\n", loc_coor_filter.lat,loc_coor_filter.lng
							// 	,fusion_headingl,atof(Save_Data.earthHeading)	,heading);

							nread = 0;

						//	loc_coor =  WGS84tobaidu(loc_coor_filter.lng,loc_coor_filter.lat);
							loc_coor =  WGS84tobaidu(lon_filt,lat_filt);
							//Creatstatejson(loc_coor,atof(Save_Data.earthHeading),atof(Save_Data.earthSpeed)*1.852/3.6,fusion_headingl);	
							set_callback( loc_coor,atof(Save_Data.earthHeading),atof(Save_Data.earthSpeed)*1.852/3.6,pfunc);
						}
						
							 
							
					}
			   }
		
		}
	
		
	}


   	file1.close();
   	file2.close();
	close(fd);

	exit(EXIT_SUCCESS);

	return 0;
}


