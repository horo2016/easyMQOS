#include "mydwa.h"
#include<cmath>
#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include <limits>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
float v_reso = 100;//mm 每个像素10mm  100mm/10
float yawrate_reso = 0.1;//0.5 * PI / 180.0;
//预测时间和单步执行时间
float dt = 0.5;
float predict_time = 2.0;
//障碍物最小距离
float obstacle_min_dis = 0.05;
//距离目标最小距离
float goal_min_dis = 1.0;
float robot_radius = 0.6;//1 bylide 实际中注意这个机器人的半径 太大导致原地打转
float to_goal_cost_gain = 1.0;// 权重调节 认为航向角的比重为
float speed_cost_gain = 1.0;
mydwa::mydwa()
{
    _vMin =0.0;//线速度
    _vMax =500;//mm/s
   //角速度
    _wMin =-0.3;
    _wMax =0.3;
}
int mydwa::update_obstacle(Obstacle obs)
{
    obs_ =obs;
}

void mydwa::update_robot_and_goal(float x,float y,float theta,Point goal)
{
    cur_x_.x_ = x ;
    cur_x_.y_ = y ;
    cur_x_.theta_ = theta ;
    goal_ = goal;
    
}

State mydwa::motion(State x, Control u)
{
    x.theta_ += u.w_ * dt;
    x.x_ += u.v_ * std::cos(x.theta_) * dt;
    x.y_ += u.v_ * std::sin(x.theta_) * dt;
    x.v_ = u.v_;
    x.w_ = u.w_;
    return x;
}

Traj mydwa::calc_trajectory(State x, float v, float w)
{
    Traj traj;
   // traj.push_back(x);
    float time = 0.0;
   //这里实现预测：预测时间为predicattime 3，每一次执行为dt 时间 ，保留的轨迹点waypoint_cnts = predicat /dt
  //实际中 dt 应该与地盘执行时间返回时间保持一致
    while (time <= predict_time){
        x = motion(x, Control{v, w});
        traj.push_back(x);
        time += dt;
    }
    return traj;

}
float mydwa::calc_obstacle_cost(Traj traj)
{
    // calc obstacle cost inf: collistion, 0:free
    int skip_n = 2;
    float minr = std::numeric_limits<float>::max();

    for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
        for (unsigned int i=0; i< obs_.size(); i++){
            float ox = obs_[i].x;
            float oy = obs_[i].y;
            float dx = traj[ii].x_ - ox;
            float dy = traj[ii].y_ - oy;

            float r = std::sqrt(dx*dx + dy*dy);
            if (r <= robot_radius){//小于障碍物距离
                return std::numeric_limits<float>::max();//有障碍物时返回最大损耗 
            }

            if (minr >= r){
                minr = r;
            }
        }
    }

    return 1.0 / minr;// 距离障碍物越近 返回值越大
}
float mydwa::calc_degree_cost(Traj traj)
{
    float base_dx = traj[0].x_ - cur_x_.x_ ;
    float base_dy = traj[0].y_ - cur_x_.y_ ;
    float base_oritation = atan2(base_dy,base_dx);

    float goal_dx = goal_.x - cur_x_.x_ ;
    float goal_dy = goal_.y - cur_x_.y_ ;
    float _oritation = atan2(goal_dy,goal_dx);

    float dtheta = _oritation - base_oritation;
    
    if(dtheta < -M_PI)                   //#Note:numpy and OpenCV X and Y reversed
			dtheta = dtheta +2*M_PI ;
	else if( dtheta > M_PI)
		dtheta = dtheta - 2*M_PI ;

        return fabs(dtheta) ;
}
Traj mydwa::calc_final_cost(Control& u)
{
    float min_cost = 10000.0;
    Control min_u = u;
    min_u.v_ = 0.0;
    Traj best_traj;
    
    // capture the start time
    clock_t         start, stop;
    start = clock();

    // evalucate all trajectory with sampled input in dynamic window
    int traj_cnt = 0;
    for (float v=_vMin; v<= _vMax; v += v_reso){
        for (float w= _wMin; w<= _wMax; w+= yawrate_reso){

            Traj traj = calc_trajectory(cur_x_, v, w );//模拟计算出 轨迹来
            
            float ob_cost = calc_obstacle_cost(traj ); //越大 障碍物越近 越小越好
			float degree_cost = calc_degree_cost(traj);//越小 角度越小 合适

			// by lide 增加比重
		    // 航向得分的比重、速度得分的比重、障碍物距离得分的比重 

            float final_cost =   ob_cost + degree_cost;
            //计算总的损耗  损耗越小path better
            if (final_cost  < min_cost){
                min_cost = final_cost;
                min_u = Control{v, w};
                best_traj = traj;
            }
            traj_cnt ++;
        }
    }

    stop = clock();
    float   elapsedTime = (float)(stop - start) /
                          (float)CLOCKS_PER_SEC * 1000.0f;
    printf( "Time to generate:  %3.1f ms\n", elapsedTime );

    u = min_u;
    return best_traj;

}