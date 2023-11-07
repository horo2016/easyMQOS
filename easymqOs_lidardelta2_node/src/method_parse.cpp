#include "method_parse.h"

#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include <unistd.h>
#include<vector>

#include <fstream>
#include <sstream>
using namespace std;

typedef struct
{
  double lnt;
	double lat;
}segment_t;


std::vector<segment_t> waypoint_list;

void push_waypoint(double lon ,double lat)
{
	segment_t tmp_wayp;
	
	tmp_wayp.lat =lat;
	tmp_wayp.lnt =lon;

	waypoint_list.push_back(tmp_wayp);

}
void save_waypoint()
{
  FILE* fp2;	 
	if ((fp2 = fopen("waypoints.csv", "a")) == NULL)    //���ļ�
		return ;
	  vector<segment_t>::iterator it;
	printf("segment_line_list total:%d\n",waypoint_list.size());
	for(it=waypoint_list.begin();it!= waypoint_list.end();it++)
	{
		
		char buffer[1024] = {0};
		sprintf(buffer,"%f,%f\r\n",it->lat,it->lnt);
		
		int numberOfBuffer = fprintf(fp2, buffer);         //д�����ݵ�log.txt
		fflush(fp2);  
   }
	if (fclose(fp2) != 0) 							 //�ر��ļ�
		 return ;

}
void clear_waypoint()
{
	if(!waypoint_list.empty())
		   waypoint_list.clear();

}

