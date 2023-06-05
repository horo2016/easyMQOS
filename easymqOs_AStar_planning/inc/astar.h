#ifndef _ASTAR_H
#define _ASTAR_H

	//起始点定义
extern int start_point_x;//在这改动图像和二维数组的关系
extern int start_point_y ;
//目标点定义
extern int goal_point_x;
extern int goal_point_y;
extern char start_find;
extern void *findPathAstar_task(void*);
extern void path_public_callback(std::vector<cv::Point> msg);
#endif