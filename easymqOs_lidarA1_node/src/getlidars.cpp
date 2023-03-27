
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

#include "getlidars.h"
#include "RPLidar.h"
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
	//	printf("distance from lidar :%f \n",distance);
	if(distance > DIST_MIN && distance < DIST_MAX && anga < ANG_MAX && ceil(anga) > ANG_MIN) { // only send real data
		softBuffer_d[(int)ceil(anga) ] = (unsigned short)round(distance); // least significant dist bits    
		//printf("degeree display :%d ,%d \n",(int)ceil(anga) ,(unsigned short)round(distance));
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
	//lidarSend_Init();
	RPLidar *lidar =new RPLidar();
	rplidar_response_device_health_t   healthinfo;
	
	lidar->begin("/dev/rplidar");
	cout << lidar->millis() << endl;

	lidar->getHealth(healthinfo,5000);
	printf("Info:heath:%d \n",healthinfo.status);
	printf("Info:RPLIDAR S/N: ");
   
	 rplidar_response_device_info_t info;
    if (IS_OK(lidar->getDeviceInfo(info, 100))) {
		
	    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", info.serialnum[pos]);
    }
		printf("\n");
		//lidar->stopMotor();
		printf("press any key start...\n");
		//int a;
	   // scanf("%d",&a);
		u_result ans= lidar->startScan();
		sleep(1);
		lidar->startMotor();
		
    
		if( ans == RESULT_OK)
			printf("Info:RPLIDAR start scan set success\n ");
	
		   usleep(500);
    }
	float prev_ang =0;
	int wait_cnt =0,wait_cnt_times =0;
	while(1){
	  	usleep(5000);
			  if(!fixing) {
				
				if(runLIDAR) {
				  if(IS_OK(lidar->waitPoint())) { // 90us, waits for new data point (500us call-to-call in theory...)
					  // 36 us
					float distance = lidar->getCurrentPoint().distance; //distance value in mm unit
					float angle    = lidar->getCurrentPoint().angle; //anglue value in degree
					bool  startBit = lidar->getCurrentPoint().startBit;//һ֡�ռ�����1
					unsigned int  quality = lidar->getCurrentPoint().quality;
					
					if(quality ==0)
						continue;
					if(distance < 300 || distance >3000)
						continue;
					if(ceil(angle) ==0 || ceil(angle) >360)
						continue;
					if(prev_ang !=0){
						if(prev_ang == 360 || prev_ang == 359)
							{
									if(ceil(angle) >15 && ceil(angle)!= 360)
										continue;
						  }
						else if(abs(ceil(angle) - prev_ang )>50){
								wait_cnt_times ++ ;
								//printf("eror lidar :%f,%f,%d \n",ceil(angle),distance,quality);
								if(wait_cnt_times < 5)
										continue;
								if(wait_cnt_times > 10)
								{
								  DEBUG(LOG_ERR,"LIDAR read datas over >15!pull out REPLUG lidar\n!");
							   //lidar->stopMotor();
								// sleep(1);
								// lidar->startScan();
								 //lidar->startMotor();
							  // wait_cnt_times =0;
								// prev_ang =0;
								}
						
							}
						
						else if(ceil(angle) < prev_ang ){								
							continue;
							}
						}
					wait_cnt_times = 0;
					prev_ang = ceil(angle);
					checkScanRate(); // 12 us // ensure we're getting TARG scans per revolution
					//printf("lidar :%f,%f,%d \n",ceil(angle),distance,quality);
					if (writeScanData(distance,ceil(angle)) == 1) // 72 us // send data to computer over serial0
					{
						lidar_t single_ldar;
									
						memcpy(single_ldar.dis,softBuffer_d,bufferIndex*sizeof(unsigned short));
					  
					 	 
					    single_ldar.timeStamp = millis(); 
					    
						set_callback(single_ldar,pfunc);
					}
					wait_cnt =0;
					} else {
						wait_cnt ++ ;
						if(wait_cnt > 10)
						{
						  DEBUG(LOG_ERR,"LIDAR read datas out of time!pull out REPLUG lidar\n!");
					   lidar->stopMotor();
					   exit(1);
						}
				  //fixLIDAR(); 
				  } // turn LIDAR on and make sure it's running fine
				} 
		  }

		
	  
	}
	lidar->stopMotor();
	//lidarSer.end();
	lidar->end();
	
	return ;
}


