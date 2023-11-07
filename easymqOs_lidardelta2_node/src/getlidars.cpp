
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include <deque>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include <stdlib.h>
#include <unistd.h> 
#include <string.h>
#include "getlidars.h"
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"
//#include "queue.h"
#include "math.h"
#include "osp_syslog.h"

const unsigned short BUF_LEN = 20; // points per transmit packet
const unsigned short PKT_SIZE = 4; // bytes per point
const unsigned short BUF_SIZE = (360-1) ;//PKT_SIZE * BUF_LEN; // bytes per transmit packet
// Scan data initialization
unsigned short dist;
unsigned short ang;
bool startBit;
unsigned long long lidarFixed;
int beatDuration = 50; // Heartbeat timing loop (ms)
unsigned short ind = 0; // counter of number of scans in current revolution
char fixing = false; // Are we currently trying to fix the LIDAR?
bool runLIDAR = true; // Do stuff with LIDAR?
int driveError = 0; // positive error = left wheel gone too far
int bufferIndex = 360; // where in the software buffer are we?
unsigned short softBuffer_d[361] = {}; // store points to send in BUF_LEN-packet bursts

const float DFAC = 0.8; // distance resolution factor [1/mm]
const unsigned short AFAC = 8; // angle resolution factor [1/deg]
const unsigned short TARG = 360; // 360 points per revolution

const unsigned short DIST_MIN = 100*DFAC; // minimum distance, scaled to TX value
const unsigned short DIST_MAX = 3000; // maximum distance, scaled to TX value
const unsigned short ANG_MIN = 1; // minimum scan angle, scaled to TX value
const unsigned short ANG_MAX = 361; // maximum scan angle, scaled to TX value

using namespace std;
std::deque<lidar_t> ldar;//ȫ���״����ݶ���
#define DEG2RAD(x) ((x)*M_PI/180.)


typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;
unsigned long millis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long count = tv.tv_sec * 1000000 + tv.tv_usec;
    return count / 1000;
}
unsigned long micro_millis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long count = tv.tv_sec * 1000000 + tv.tv_usec;
    return count ;
}

int lidarSend_Queue(float *pbuffer_dis, float *pbuffer_ang,
	unsigned int frmsize   )
{	
#if 0
  int lRet = 0;	static u32 i=0;
  int s=	lidar_packet_queue_size();
 // printf("there is size:%d in queue \n",s);
	if(s >=1)//̫�಻д
		return 0;
	LidarDatasPacket *packet = NULL;
	packet = (LidarDatasPacket*)malloc(sizeof(LidarDatasPacket));
	if(packet == NULL){		
		DEBUG(LOG_ERR,"Packet memory allocate error!\n");
		return -1;	
		}	
		packet->dis= (float*)malloc(frmsize*sizeof(float));	
		if(packet->dis == NULL){		
			DEBUG(LOG_ERR,"dis packet->data memory allocate error!\n");
		return -1;	
		}	
		packet->ang= (float*)malloc(frmsize*sizeof(float));	
		if(packet->ang == NULL){		
			DEBUG(LOG_ERR,"ang packet->data memory allocate error!\n");
		return -1;	
		}
		/*copy current frame data and size into packet buffer*/	
		memcpy(packet->dis, pbuffer_dis, frmsize*sizeof(float));
		memcpy(packet->ang, pbuffer_ang, frmsize*sizeof(float));
		packet->size = frmsize;
	  packet->timeStamp= millis();
		lRet = lidar_packet_queue_push_head(packet);//�������Ƶ�������
	if(lRet < 0){			DEBUG(LOG_ERR,"[ send ] :queue build error!\n");
		return -1;	
		}
	i++;
//	if(packet->dis != NULL || packet->ang != NULL ){
	//		free(packet->dis);
	//		free(packet->ang);
	//	}
	//	free(packet);
  // DEBUG(LOG_DEBUG,"[ send ] :queue   push  %d!\n",i);
	return lRet;

#endif
}
void filterLidarScan()
{

	float t_softBuffer_d[360] = {}; 
	float t_softBuffer_a[360] = {};
	  


}
void checkScanRate() {
  if(startBit) {
    // maintain TARG readings per revolution to minimize SLAM error
   /* if(ind > TARG + 1 and motorspeed < MAXSPEED) {analogWrite(RPLIDAR_MOTOR, ++motorspeed);} // too many readings
    else if(ind < TARG - 1 and motorspeed > MINSPEED) {analogWrite(RPLIDAR_MOTOR, --motorspeed);} // too few readings
    */
	ind = 0; // reset counter
	
  }
  ind++;
}


char writeScanData(float distance,float anga) {
	
	//lidar_t single_ldar;
	char ret =0;
		//printf("distance from lidar :%f \n",distance);
	if(distance > DIST_MIN && distance < DIST_MAX && anga < ANG_MAX && ceil(anga) > ANG_MIN) { // only send real data
		softBuffer_d[(int)ceil(anga) ] = (unsigned short)round(distance); // least significant dist bits    
	//	printf("degeree display :%d ,%d \n",(int)ceil(anga) ,(unsigned short)round(distance));
		ret = -1;
	}
	if((int)ceil(anga) == 360) {
			ret =1;
	}

   return ret;  
}
int lidarSend_Init()
{	int lRet = 0;
	/*Initial queue*/	
	//lRet = queue_init();	
	if(lRet < 0){		
		DEBUG(LOG_ERR,"[ queue ] : Queue init error!\n");
		return -1;	
		}
	DEBUG(LOG_DEBUG,"[ queue ] :Queue_Init OK!\n\n");
	return lRet;
}

void set_callback(lidar_t lidar_msg, void(*pfunc)(lidar_t))
{

	(*pfunc)(lidar_msg);
	return ;
}
void lidar_task(void(*pfunc)(lidar_t))
{
    int    opt_com_baudrate = 115200;
	char opt_com_path[0xff]={0};
	strcpy(opt_com_path,USBDEVICE);
    //string opt_com_path = "/dev/ttyUSB0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path);
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
        printf("baud rate:%d\n",serial_connect.getBaud());
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path);
        return ;
    }

    printf("C3iroboticslidar connected\n");

    robotics_lidar.initilize(&serial_connect);


    while (1)
    {
		TLidarGrabResult result = robotics_lidar.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
                std::vector<RslidarDataComplete> send_lidar_scan_data;
                send_lidar_scan_data.resize(lidar_scan_size);
                RslidarDataComplete one_lidar_data;
                for(size_t i = 0; i < lidar_scan_size; i++)
                {
                    one_lidar_data.signal = lidar_scan.signal[i];
                    one_lidar_data.angle = lidar_scan.angle[i];
                    one_lidar_data.distance = lidar_scan.distance[i];//0.15-5.0m
                    send_lidar_scan_data[i] = one_lidar_data;
					if (writeScanData(lidar_scan.distance[i]*1000,ceil(lidar_scan.angle[i])) == 1) // 
					{
						
					}
                }
				lidar_t single_ldar;
									
				memcpy(single_ldar.dis,softBuffer_d,lidar_scan_size*sizeof(unsigned short));
				memset(softBuffer_d,0,360*sizeof(unsigned short));
					
				single_ldar.timeStamp = millis(); 
				
				set_callback(single_ldar,pfunc);

                printf("Lidar count %d!\n", lidar_scan_size);
				

                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(50);
    }

    return ;
	

}

