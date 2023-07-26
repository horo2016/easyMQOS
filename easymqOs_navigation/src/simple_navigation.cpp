#include "simple_navigation.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include "sensor_msg.h"
#include "rtquaternion.h"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
 
#include <sys/types.h>
#include <sys/wait.h>
#include "kalman.h"
#include "cJSON.h"
#include "Mqtt/client_pub_sub.h"
#include "Mqtt/mosquitto.h"
#include "mydwa.h"
#include "osp_syslog.h"
using namespace std;
using namespace cv;
char process_init =0;
float act_heading;	
float init_heading =0;
Kalman xAccelFilter(0.225, 0.125, 1, 0);

Kalman gyrolFilter(0.025, 0.25, 1, 0);
float px=0,py=0;
Vector3 _gPosition ; 
float dis_odom =0;
float vx =0;
 
unsigned short lidar_dis[361];
 // data collection parameters
int num_data =100;
int imu_counter = 0;

// data storage elements
float sum_accel[3];
float sum_gyro[3];
float acc_min_x =999;
float acc_max_x =0;
float vt =0,s_d=0;
long time_prev_;
long time_prev_ms;
float gyro_ang =0;//单位时间内转过的角度
float last_heading =0;
float last_act_heading= -999;
float odom_theta =0 ;

char once_phase =0;
char odom_complete_flg =0;
 


int ticks_l_curr ;//msg->data[0]; // total ticks left wheel
int ticks_r_curr;
float _g_odom_ang_rad=0;
float _g_last_odom_ang_rad =0,_g_last_imu_ang_rad =0;
float delata_theta_thesh =0.0115;//need check  /500ms  0.0625

float statis =0;
float g_distance_acc =0;

 char navigation_run =0;
std::vector<Point> wayPoints;
float map_unit =10.0;//10mm/pix
int map_offset =200;//初始位置在地图的200 位置
Point current_waypoint;


long long getmillis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count = tv.tv_sec * 1000000 + tv.tv_usec;
	//printf("stamp:%ld \n",tv.tv_sec);
	//printf("us:%ld \n",tv.tv_usec);
    return (count / 1000);
}
long long getsecond()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count = tv.tv_sec ;
	
    return count ;
}
long long getmiilsecond()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long count =  tv.tv_usec;
	
    return count / 1000;
	
    return count ;
}
// encoders callback
float wheel_car_factor =1.1;//小车因时间问题滞后因子
void encoders_callback(    int l_encodes,  int r_encodes)
{		
	 ticks_l_curr = 0-l_encodes;//msg->data[0]; // total ticks left wheel
	 ticks_r_curr = 0-r_encodes;//msg->data[1]; // total ticks right wheel

	if(process_init ==0)
		return;
	printf("encoders :L:%d ,R:%d \n",l_encodes,r_encodes);
    if(last_act_heading == -999){
	  last_act_heading =act_heading;
	  return ;
	}
	// Compute distance moved by each wheel	
	float Dl = (ticks_l_curr)*0.1326 *wheel_car_factor;//mm
	float Dr = (ticks_r_curr)*0.1326 *wheel_car_factor;//mm
	float Dc = (Dl+Dr)/2.0;
	float theta  =(Dr-Dl)/200 ; // theta  车宽度为l=0.2m

	vx = Dc;
    float delta_theta_odom = theta - _g_last_odom_ang_rad ;
	_g_last_odom_ang_rad = theta;
	float delta_theta_imu = gyro_ang - _g_last_imu_ang_rad ;
	_g_last_imu_ang_rad = gyro_ang;


	//变成绝对值相减 例如 ，4-6=-2，（-4 - -6）->4-6 认为有效
	float delta_theta_g_o =  fabs(theta*180/3.1415) - fabs(delta_theta_imu);
	printf("degree,delta_theta_imu:%f,delta_theta_odom:%f ,odom -imu=%f \n",delta_theta_imu , theta*180/3.1415,delta_theta_g_o);
	//超过2度认为轮子打滑 一般是原地转弯时发生单轮子打滑
	if((delta_theta_g_o)   > 2){
	 	DEBUG(LOG_ERR,"error : Da Hua \n");
		once_phase =1;  //完成一个阶段
		odom_complete_flg =1;//完成一个接收
		return ;
	}

 	_g_odom_ang_rad += theta;
	if(_g_odom_ang_rad < -M_PI)                   //#Note:numpy and OpenCV X and Y reversed
			_g_odom_ang_rad = _g_odom_ang_rad +2*M_PI ;
	else if( _g_odom_ang_rad > M_PI)
		_g_odom_ang_rad = _g_odom_ang_rad - 2*M_PI ;
		

	//printf("odom_degree:(%f),\n",_g_odom_ang_rad*180/3.14);
 //	printf("imu_degree:%f \n",gyro_ang);

	dis_odom += Dc;
	g_distance_acc += s_d;
	printf("distance_acc :%f \n",s_d);
	printf("distance_odom :%f \n",Dc);
	float w = theta*2;//500ms 角速度
    
	



	_gPosition.z = gyro_ang*M_PI/180;//rad
	if(_gPosition.z  < -M_PI)                   //#Note:numpy and OpenCV X and Y reversed
			_gPosition.z  = _gPosition.z  + 2*M_PI;
	else if( _gPosition.z  > M_PI)
		_gPosition.z  = _gPosition.z  - 2*M_PI ;
	printf("imu heading:%f,odom heading:%f, \n",gyro_ang,_g_odom_ang_rad*180/3.1415);
    float   o_x =  Dc *cos(_gPosition.z);//转换成mm
	float   o_y =  Dc *sin(_gPosition.z);
	
	_gPosition.x = _gPosition.x +  o_x; 
	_gPosition.y = _gPosition.y +  o_y;
	 


	printf("robot pose (%f,%f,%f),\n",_gPosition.x,_gPosition.y,_gPosition.z*180/3.14);


	// _ekf.update( z );
    // Eigen::Matrix<double, 4, 1> x_now = _ekf.getStateX();
	//std::cout<<"X = "<<std::endl<<x_now<<std::endl;
	//printf("--->%f \n",x_now[2]*180/3.14);

	once_phase =1;  //完成一个阶段
	odom_complete_flg =1;//完成一个接收
 
}

// encoders callback
void odom_callback(sensors_msg_odom _msg)
{		
 
    
}

float acc_biases[3] ;
float gyro_biases[3];
float rpy_biases[3];

// imu callback
// updates orientation and position using quaternion math and physics kinematic equations
// Prediction Step/Time Propogation/State Update/Vehicle Kinematics step
//更新姿态位置通过运动学 预测时间步长
void imu_callback(sensors_msg_imu msg)
{
	
	
}
void imu_callback_085(sensors_msg_imu msg)
{
	
	
	// IMU data
	Vector3 rpy = msg.angular_velocity;
	Vector3 acc = msg.linear_acceleration;
	 
	const time_t t_s = msg.stamp_ss;
	
    

    // Calculate dt.
	if(process_init == 0)
	 {
		if (imu_counter < num_data)
		{
			printf("init collect imu frame:%d \n",imu_counter);
			sum_accel[0] += (acc.x);
			sum_accel[1] += (acc.y);
			sum_accel[2] += (acc.z);
			sum_gyro[0] += (rpy.x);
			sum_gyro[1] += (rpy.y);
			sum_gyro[2] += (rpy.z);
		 
			if(acc_max_x < acc.x )
				acc_max_x = acc.x;
			if(acc_min_x > acc.x )
				acc_min_x = acc.x;
			// increment counter
			imu_counter++;
			 
			
		} else {
				 acc_biases[0] = sum_accel[0] / num_data;
				 acc_biases[1] = sum_accel[1] / num_data;
				 acc_biases[2] = sum_accel[2] / num_data;
				 rpy_biases[0] = sum_gyro[0] / num_data;
				 rpy_biases[1] = sum_gyro[1] / num_data;
				 rpy_biases[2] = sum_gyro[2] / num_data;

				 
				printf("init accel bias:%f,%f,%f\n",acc_biases[0],acc_biases[1],acc_biases[2]);
				printf("init rpy bias:%f,%f,%f\n",rpy_biases[0],rpy_biases[1],rpy_biases[2]);
				process_init =1;
				printf("acc min,max (%f,%f) \n ",acc_min_x,acc_max_x);	
				 time_prev_ms =  getmillis();;
		}
	
		return ;
	}
		
	const time_t t_ms = getmillis();
    double dt;
 if(t_ms > time_prev_ms)
        dt = (double) (t_ms - time_prev_ms )/1000;//s
    else if(t_ms < time_prev_ms)  
        dt =   (double) (t_ms+1000 - time_prev_ms )/1000;//s
    time_prev_ms = t_ms;
    float acc_current_x ;
	//  if(acc.x > acc_max_x)
    //      acc_current_x = (acc.x -(acc_min_x+acc_max_x)/2);
	// else if(acc.x < acc_min_x)
    //      acc_current_x = (acc.x -(acc_min_x+acc_max_x)/2);
	// else  acc_current_x =0.0;

   
    acc_current_x = acc.x - acc_biases[0];
	if(abs(acc_current_x) < 0.005)//静止阈值
		acc_current_x =0;
	if(vx ==0)
        vt =0;


	//printf("x axix accx:%f \n",acc_current_x);
	s_d += vt*dt + dt*dt*acc_current_x*0.5;

	vt = vt+acc_current_x*dt;
//	printf("current acc=%f. \n",acc.x);
//	printf("IMU current vt=%f,dis=%f,fusionacc=%f \n",vt,s_d,acc_current_x);
	float current_gyro_ang = rpy.z-rpy_biases[2];
	//得到当前的航向角 gyro_ang
	gyro_ang = current_gyro_ang ;
	  


	if(once_phase ==1)
	{
	  	  
       s_d =0;	
	   once_phase =0;
	}
	
}

//发布话题的回调
void public_map_traj(vector<Point> traj,Point p,int ang)
{

   
  
    char  value_buf[4096*1024]={0};
    

    cJSON * root =  cJSON_CreateObject();
  if(!root) {
         printf("get root faild !\n");
     }
   // cJSON_AddItemToObject(root, "type",cJSON_CreateNumber(type));
	cJSON_AddItemToObject(root, "x",cJSON_CreateNumber(p.x));
	cJSON_AddItemToObject(root, "y",cJSON_CreateNumber(p.y));
	cJSON_AddItemToObject(root, "heading",cJSON_CreateNumber(ang));

	cJSON_AddItemToObject(root, "wx",cJSON_CreateNumber(current_waypoint.x));
	cJSON_AddItemToObject(root, "wy",cJSON_CreateNumber(current_waypoint.y));
	cJSON *ArrayObj3 =cJSON_CreateArray();

		for (unsigned int i = 0; i < traj.size(); i++)
		{      
			int a[2];
			a[0]=traj[i].x;
			a[1]=traj[i].y;

			cJSON_AddItemToArray(ArrayObj3,cJSON_CreateIntArray(a,2)); 
		}
		cJSON_AddItemToObject(root, "traj",ArrayObj3); 
		
		
 	
	

    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));

#if defined MQTT_REMOTE_SERVER 
	//	sprintf(send_buf,"mosquitto_pub -h www.woyilian.com -t %s  -m \"%s\"",topic_buf,value_buf);
#elif defined MQTT_TERMINAL_SERVER
	//	sprintf(send_buf,"mosquitto_pub -t %s  -m \"%s\"",topic_buf,value_buf);
#endif
	//system(send_buf);	
	//printf("length:%d \n :%s \n",strlen(value_buf),value_buf);
	if(strlen(value_buf)>0)
    publish_message(strlen(cJSON_Print(root)),value_buf);
    cJSON_Delete(root);
	
	
	//traj.clear();
    return ;
}
void public_map_raw_datas(int len, char *buf)
{
printf("len:%d \n",len);
publish_message(len,buf);

}
//发送命令
void public_cmd(float v,float ang)
{

   
  
    char  value_buf[1024]={0};
    char  send_buf[1024]={0};
	char ftostr[32]={0};
    cJSON * root =  cJSON_CreateObject();
  if(!root) {
         printf("get root faild !\n");
     }
   // cJSON_AddItemToObject(root, "type",cJSON_CreateNumber(type));
	cJSON_AddItemToObject(root, "\"control\"",cJSON_CreateString("\"1\""));
	sprintf(ftostr,"\"%.2f\"",v);
	cJSON_AddItemToObject(root, "\"vel\"",cJSON_CreateString(ftostr));
	memset(ftostr,32,0);
	sprintf(ftostr,"\"%.2f\"",ang);
	cJSON_AddItemToObject(root, "\"ang\"",cJSON_CreateString(ftostr));
		

    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));


	sprintf(send_buf,"mosquitto_pub -t /cmd/vel  -m \"%s\"",value_buf);

	system(send_buf);	
	printf("length:%d \n :%s \n",strlen(value_buf),value_buf);
	
    cJSON_Delete(root);
	
	
	//traj.clear();
    return ;
}
int  lidar2obstacle(vector<Point> *obs,char *isleftbig)
{
	int ret = 0;
    int cnt =0;
	int left_SonarDist_min =1999,right_SonarDist_min =1999;
	printf("LEFT: ");
	for(int i =160;i<180;i =i+2)
	{
		int dis = (unsigned int)lidar_dis[i];
		float SonarDist_t =  (float)dis * 100.0 /100.0;
		if(dis <left_SonarDist_min)
		  left_SonarDist_min =dis ;
	
		float rad = i *M_PI/180 + _gPosition.z;

		 if  (rad < -M_PI)                   
                rad = rad + 2 * M_PI ;
            else if(rad > M_PI)
                rad = rad - 2 * M_PI;

		int Xr =  ( int)(_gPosition.x  + SonarDist_t*cos(rad));
		int Yr = ( int)(_gPosition.y + SonarDist_t*sin(rad));
		Point p(Xr,Yr);
		obs->push_back(p);
		printf("%d:%d ",i,dis);
		if(dis < 450)
			ret++;
		if(dis < 600 && dis>450)
			cnt++;
	}
	printf("\n Right:");
		for(int i =180;i<200;i =i+2)
	{
		int dis = (unsigned int)lidar_dis[i];
		float SonarDist_t =  (float)dis * 100.0 /100.0;
		if(dis <right_SonarDist_min)
		  right_SonarDist_min =dis ;
		float rad = i *M_PI/180 ;
		//  if  (rad < -M_PI)                   
        //         rad = rad + 2 * M_PI ;
        //     else if(rad > M_PI)
        //         rad = rad - 2 * M_PI;
				
			rad += _gPosition.z;

		 if  (rad < -M_PI)                   
                rad = rad + 2 * M_PI ;
            else if(rad > M_PI)
                rad = rad - 2 * M_PI;

		int Xr =  ( int)(_gPosition.x  + SonarDist_t*cos(rad));
		int Yr = ( int)(_gPosition.y + SonarDist_t*sin(rad));
		//printf("%f ,point(%d,%d) \n",rad,Xr,Yr);
		Point p(Xr,Yr);
		obs->push_back(p);
		printf("%d:%d ",i,dis);
		if(dis < 450)
			ret ++;
		if(dis < 600 && dis >450)
			cnt++;
	}
	printf("\n");
	if(left_SonarDist_min < right_SonarDist_min){
		*isleftbig =1;//左侧有大障碍物
		
	}
		
	else {
		*isleftbig=0;
		//printf("right has obstacle \n");
	}
	if( ret!=0 )
		return 2;
	else if(cnt >0)
		return 1;
	else return 0;
 

}
int navigation_task(void(*pfunc)(sensors_msg_odom ))
{

	
	

	
	float prevError = 0.0f;
	float totalError = 0.0f;
	float P=9.0,I=0.0,D=0.0;
	float MAX_STEER = 0.5f;
	char need_read_waypoint =0;
	int limit_dis =150;//mm
	int step_check_waypoints =50;

    mydwa *mydwa_demo = new mydwa();
	int serious_obstacle =0;
	while(1)
	{
		//vector <Point> _tmpobs;
		//int ret =lidar2obstacle(&_tmpobs);
		while(navigation_run)
		{
			if(need_read_waypoint == 0 && !wayPoints.empty() && wayPoints.size()>0)
			{
					current_waypoint = wayPoints[wayPoints.size()-1];
					printf("length:%d ,curernt waypoint:(%d,%d) \n",wayPoints.size(),current_waypoint.x,current_waypoint.y);
					wayPoints.pop_back();
					need_read_waypoint =1;
			}
			float dx = (current_waypoint.x-map_offset)  * map_unit -_gPosition.x;//10mm/pix
			float dy = (current_waypoint.y -map_offset) * map_unit -_gPosition.y;
			float rad = atan2( dy,dx);//#atan2(y,x) =y/x  距离目标点的弧度s
			float dis = sqrt(dx*dx + dy*dy);//mm unit 
            printf("cpt:(%.1f,%.1f),->wpt:(%.1f,%.1f) ,[deg:%.1f,dis:%.1f mm] \n",
			_gPosition.x,_gPosition.y, (current_waypoint.x-map_offset)  * map_unit,(current_waypoint.y -map_offset) * map_unit,rad*180/3.14,dis);

			//150mm的定位精度
			if(dis < limit_dis){//航点已经到达 读取下一个航点
				if(!wayPoints.empty() && wayPoints.size()>0)
				{
					need_read_waypoint =0;
					printf("curernt:(%d,%d) have been arrived \n",current_waypoint.x,current_waypoint.y);
				}else {
					navigation_run =0;
					printf("waypoint lenth is 0 ,navigation exit \n");
					break;
				}
				continue;
			}else if(dis > (limit_dis+step_check_waypoints))//第一次检验航点
			{
				Point t_current_waypoint;
				int _tdis =140;
				int finnal_waypoint_cnt =0;
				DEBUG(LOG_DEBUG,"waypoints over everyone \n");
				for(int i =0;i<(step_check_waypoints-30)/10;i++){

				
					t_current_waypoint = wayPoints[wayPoints.size()-1-i];
					float dx = (t_current_waypoint.x-map_offset)  * map_unit -_gPosition.x;//10mm/pix
					float dy = (t_current_waypoint.y -map_offset) * map_unit -_gPosition.y;			
					float dis = sqrt(dx*dx + dy*dy);//mm unit
					if(dis < limit_dis)
					{
						if(dis<_tdis){
							_tdis =dis;
				            finnal_waypoint_cnt =i;

						}
					}
				}
				if(finnal_waypoint_cnt){
					for(int j=0;j<finnal_waypoint_cnt;j++)
						wayPoints.pop_back();//删除已达到的航点
					need_read_waypoint =0;
					step_check_waypoints =50;
					continue;
				}
				else
				step_check_waypoints +=20;

			}
			//以下是 pure suit 的算法
 			float alpha = rad - _gPosition.z;
            if  (alpha < -M_PI)                   
                alpha = alpha + 2 * M_PI ;
            else if(alpha > M_PI)
                alpha = alpha - 2 * M_PI;
            printf("heading:%.1f,target:%.1f,alpha:%f \n",_gPosition.z*180/3.14,rad*180/3.1415,alpha*180/3.14);
			float v =0.1;
			float w = P * 2*v*sin(alpha)/sqrt(dis);
            printf("turn w:%f\n",w);
			v =0.05;

			//以下使用PID 算法
			//totalError += alpha;	
			//float output_w = P*alpha + I*totalError + D*(alpha - prevError);    
            //prevError = alpha;
			vector <Point> _tmpobs;
			char isleft;
			int level =lidar2obstacle(&_tmpobs,&isleft);
			if(level ){
				mydwa_demo->update_obstacle(_tmpobs);//刷新障碍物地图
				//刷新机器人位置和目标点
				mydwa_demo->update_robot_and_goal(_gPosition.x,_gPosition.y,_gPosition.z,Point((current_waypoint.x-map_offset)  * map_unit,(current_waypoint.y -map_offset) * map_unit));
				Control u_res;
				Traj ltraj = mydwa_demo->calc_final_cost(u_res);//计算最后结果
				printf("dwa result: %f,%f \n",u_res.v_,u_res.w_);

				// cv::Mat bg(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
				// cv::circle(bg, Point((current_waypoint.x-map_offset)  * map_unit+50, (current_waypoint.y -map_offset) * map_unit+50),
				// 		3, cv::Scalar(255,0,0), 5);
						
				// for(unsigned int j=0; j<_tmpobs.size(); j++){
				// 	cv::circle(bg, Point(_tmpobs[j].x+50, _tmpobs[j].y+50),
				// 			5, cv::Scalar(0,0,0), -1);
				// }
				// for(unsigned int j=0; j<ltraj.size(); j++){
				// 	cv::circle(bg, Point(ltraj[j].x_+50, ltraj[j].y_+50),
				// 			2, cv::Scalar(0,255,0), -1);
				// }
				// imwrite("dwa2.jpg", bg);
				// sleep(1);
				if(level == 2){//等级不同  速度不同
					v = 0.0;
					if(isleft ==1)
					{   
						w =0.2;//向右转
						DEBUG(LOG_ERR,"left has obstacle \n");
					}else if(isleft ==0)
					{   
						w =-0.2;
						DEBUG(LOG_ERR,"right has obstacle \n");
					}
				}
					
				 if(level ==1)
				 {
					v = 0.05;//
					if(isleft ==1)
					{   
						w =0.1;//向右转
						DEBUG(LOG_ERR,"WARNING:left has obstacle \n");
					}else if(isleft ==0)
					{   
						w =-0.1;
						DEBUG(LOG_ERR,"WARNING:right has obstacle \n");
					}
				 }
					
				serious_obstacle ++;
				if(serious_obstacle>10)
				{	for(int i=0;i<3;i++)
					{
						
						w =0.0;
						v =0.1;
						public_cmd(v,w);//下发指令控制运动
						usleep(300000);
						continue;
					}
					serious_obstacle =0;
				}
				
				//w = u_res.w_;
				
			}else if(serious_obstacle){//具备一定的障碍物堵路程度
				if(serious_obstacle>0)serious_obstacle --;
				w =0.0;
				v =0.1;
				public_cmd(v,w);//下发指令控制运动
				usleep(200000);
				continue;
			}
			
			if (w > MAX_STEER)
                    w = MAX_STEER;
			if(dis  < 100)//当距离小于100mm时降低速度。
				v = 0.05;
			if(fabs(alpha *180/3.1415) >20 )//当转角过大时 线速度为0  实现原地转弯
				v = 0.0;
			if(w>0 )
				printf("turn right \n");
			else printf("turn left \n");
			public_cmd(v,w);//下发指令控制运动
			usleep(200000);

		}
		need_read_waypoint =0;

		usleep(500000);



	}
	delete mydwa_demo;
	return 0;
}
//发布任务数据获取
void *data_CollectTask (void *)
{
 
 
    const char *file = "1my.dat";  

	FILE* fp;	 
	if ((fp = fopen(file, "a")) == NULL)    //���ļ�
		return NULL;
	unsigned int	scanCnt =0,lefttacho,righttacho;
	
	while (1/* condition */)
	{
		/* code */
		if(odom_complete_flg ==1)
		{
				struct timeval tv;
				gettimeofday(&tv, 0);
			
				fprintf(fp, "%ld%03d000,%d,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,0,0,0,0,0,0,0,0,0,0,0,0,0,0",tv.tv_sec,tv.tv_usec/1000,scanCnt++,ticks_l_curr,ticks_r_curr,
				(unsigned int)_gPosition.x,(unsigned int)_gPosition.y,_gPosition.z*180/M_PI,
				0.0,0.0,_g_odom_ang_rad)*180/M_PI;	
				for(int i=0;i<360;i++){
					fprintf(fp,",%d",(unsigned int)lidar_dis[i]);				 
				}
				fprintf(fp, "\n");
				fflush(fp); 
				odom_complete_flg =0;
				
		}
		usleep(100000);
	
	}				
	
		if (fclose(fp) != 0)                //�ر��ļ�
		 return NULL;
		
}