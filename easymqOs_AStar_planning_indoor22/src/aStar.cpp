#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <stack>
#include <opencv2/opencv.hpp>
#include "astar.h"
using namespace std;
using namespace cv;
#define WIDTH  1000
#define HEIGHT 1000

	//起始点定义
int start_point_x= 935;//在这改动图像和二维数组的关系
int start_point_y= 690 ;
//目标点定义
int goal_point_x= 468;
int goal_point_y= 650;
char start_find =0;
class CPoint
{
public:
    CPoint(int x,int y):X(x),Y(y),G(0),H(0),F(0),m_parentPoint(NULL){ };
    ~CPoint();
    int X,Y,G,H,F;
    CPoint * m_parentPoint;
    void CalF(){
        F=G+H;
    };
};
//二维数组中，二值化后， 0代表可以到达。1代表不能到达。
class  CAStar
{
    private:
	int m_array[WIDTH][HEIGHT];
	static const int STEP = 3;
    static const int OBLIQUE = 14;
	typedef std::vector<CPoint*> POINTVEC;
	POINTVEC m_openVec;
	POINTVEC m_closeVec;
    public:
        CAStar(unsigned char array[WIDTH][HEIGHT])
        {
            for (int i=0;i<WIDTH;i++)
                for(int j=0;j<HEIGHT;j++)
                    m_array[i][j]=array[i][j];
        }
		//在当前开环中,获取所有的点,找到F值最小的点 
        CPoint* GetMinFPoint()
        {
            int idx=0,valueF=9999;
			//printf("1 m_openVec size : %d \n",m_openVec.size());
            for(int i=0; i < m_openVec.size(); i++)//再开环中查找有几个坐标
                {
					//printf("2 m_openVec[i]->F (%d,%d) %d \n",m_openVec[i]->X,m_openVec[i]->Y,m_openVec[i]->F);
                    if(m_openVec[i]->F < valueF)
                    {
                        valueF = m_openVec[i]->F;
                        idx = i;
                    }
                }
		//printf("3 m_openVec selected idx: %d \n",idx);
		return m_openVec[idx];
        }
	bool RemoveFromOpenVec(CPoint* point)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if((*it)->X == point->X && (*it)->Y == point->Y)
			{
				m_openVec.erase(it);
				return true;
			}
		}
		return false;
	}
	//这里规定 0代表可以到达
	bool canReach(int x, int y)
	{
		 //优化 保证该点的上下左右的9个点都可以到达 
		 for(int i =-15;i<16;i =i+2)
		 	for(int j= -15;j<16;j=j+2)
		   {
			if(0 != m_array[x+i][y+j])
			
				return false;
		   }
		
			return true;
	}
	//	判断是否可以访问，来验证是否在障碍物中，或者在闭环中
	bool IsAccessiblePoint(CPoint* point, int x, int y, bool isIgnoreCorner)
	{
		if(!canReach(x, y) || isInCloseVec(x, y))
			return false;
		else
		{
			//可到达的点
			if(abs(x - point->X) + abs(y - point->Y) == 1)    // 左右上下点
				return true;
			else
			{
				if(canReach(abs(x - 1), y) && canReach(x, abs(y - 1)))   // 对角点
					return true;
				else
					return isIgnoreCorner;   //墙的边角
			}
		}
	}
    //获取临近的点，即当前点的四周的8个点
	//判断是否可以访问，来验证是否在障碍物中
	std::vector<CPoint*> GetAdjacentPoints(CPoint* point, bool isIgnoreCorner)
	{
		POINTVEC adjacentPoints;
		//printf("4 current center point : (%d,%d) \n",point->X,point->Y);
		for(int x = point->X-1; x <= point->X+1; x++)
			for(int y = point->Y-1; y <= point->Y+1;  y++)
			{
				if(IsAccessiblePoint(point, x, y, isIgnoreCorner))
				{
					CPoint* tmpPoint = new CPoint(x, y);
					adjacentPoints.push_back(tmpPoint);
					
				}
			}
		return adjacentPoints;
	}
    //是否在开环中
	bool isInOpenVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); it++)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}
    //是否在闭环中
	bool isInCloseVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_closeVec.begin(); it != m_closeVec.end(); ++it)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}
	/** *****************************************************************
	* @remarks    RefreshPoint
	* @brief       RefreshPoint
	* @param      add g h 的 sum 和,判断 add end point
	* @return	   none
	* @author		  
	*********************************************************************/
	void RefreshPoint(CPoint* tmpStart, CPoint* point,CPoint* end)
	{
		int valueG = CalcG(tmpStart, point);
		int valueH = CalcH(end, point);
		point->G = valueG;
	    point->H = valueH;
		point->CalF();
		//printf("7 point (%d,%d) finanlH:%d \n",point->X,point->Y,valueG +valueH);
		usleep(50);
		
	}
	/** *****************************************************************
	* @remarks    NotFoundPoint
	* @brief       没发现点
	* @param      tmpStart :当前的临时点；end :结束点;point:临近中的点
					临近的所有点计算出来F 并加入开环中
	* @return	   none
	* @author		  
	*********************************************************************/
	void NotFoundPoint(CPoint* tmpStart, CPoint* end, CPoint* point)
	{
		point->m_parentPoint = tmpStart;
		point->G = CalcG(tmpStart, point);
		point->G = CalcH(end, point);
		point->CalF();
		m_openVec.push_back(point);
	}
	int CalcG(CPoint* start, CPoint* point)//要么10 要么14
	{
		int G = (abs(point->X - start->X) + abs(point->Y - start->Y)) == 2 ? OBLIQUE : STEP ;
		int parentG = point->m_parentPoint != NULL ? point->m_parentPoint->G : 0;
		return G + parentG;
	}
	/** *****************************************************************
	* @remarks    CalcH
	* @brief       计算H
	* @param      end:结束点；point:临时点；
	* @return	   none
	* @author		  
	*********************************************************************/
	int CalcH(CPoint* end, CPoint* point)//distance 
	{
		int step = abs(point->X - end->X)+ abs(point->Y - end->Y);
		return (STEP * step);
	}
 
	// 搜索路径，路径上的每一个点都要经过开环判断拉入闭环的过程。
	//从开环开始处理
	//从开环开始处理-获取F值最小的一个点-获取临近值-将临近值压入开环-
	/** *****************************************************************
	* @remarks    NotFoundPoint
	* @brief       没发现点
	* @param      tmpStart :当前的临时点；end :结束点;point:临近中的点
					临近的所有点计算出来F 并加入开环中
	* @return	   none
	* @author		  
	*********************************************************************/
	CPoint* FindPath(CPoint* start, CPoint* end, bool isIgnoreCorner)
	{
		m_openVec.push_back(start);//加入开环中
		while(0 != m_openVec.size())
		{
			CPoint* tmpStart = GetMinFPoint();   // 获取F值最小的点
			RemoveFromOpenVec(tmpStart);//从开环中移除
			m_closeVec.push_back(tmpStart);//加入闭环
			POINTVEC adjacentPoints = GetAdjacentPoints(tmpStart, isIgnoreCorner);
			//printf("5 new adjacentPoints point push openvec : %d \n",adjacentPoints.size());
			for(POINTVEC::iterator it=adjacentPoints.begin(); it != adjacentPoints.end(); it++)
			{
				CPoint* point = *it;
				if(isInOpenVec(point->X, point->Y))// 如果在开启列表 说明已经判断过，就不再处理 
				   {
					// printf("6 point isInOpenVec (%d,%d) \n",point->X, point->Y);
				   }else // 将不在开环中的的点计算出 并压入开环中 同时将F值最小的点压入开环的 父节点，父节点就是路径
					NotFoundPoint(tmpStart, end, point);
			}
			if(isInOpenVec(end->X, end->Y)) // 目标点已经在开启列表中
			{
				for(int i=0; i < m_openVec.size(); ++i)
				{
					if(end->X == m_openVec[i]->X && end->Y == m_openVec[i]->Y)
						return m_openVec[i];
				}
			}
		}
		return end;
	}
};


void clear_dque(vector<Point>& q) {
	vector<Point> empty;
	swap(empty, q);
}

 void set_callback_function(vector<Point> _msg ,void(*pfunc)(vector<Point>))
{
    (*pfunc)(_msg);
   return ;
}
void *findPathAstar_task(void *)
{
		//输入图像加载、读取50*50
	unsigned char array_img[WIDTH][HEIGHT];
   
	Mat src = imread("word.png");
	Mat grayMaskSmallThresh;
	 if(src.empty())  
    {   
        if (!src.data) { 
            printf(" 读取图片文件错误~！使用默认图片 \n"); 
			Mat img(WIDTH,WIDTH,CV_8UC3,Scalar(255,255,255));
			src = img;
        } 
    }
	printf("channels:%d \n",src.channels());
	 // 对图像灰度化操作
    cvtColor(src, src, COLOR_BGR2GRAY);//#if opencv version>4.5 ,modify to COLOR_BGR2GRAY
	// imshow("src",src);
	//waitKey(0);
	//图像阈值化操作
	threshold(src, grayMaskSmallThresh, 128, 255, THRESH_BINARY_INV);//反相选择 >230 =0,else= 1 ，二值化的地图中，0代表可以到达。if opencv version>4.5 ,modify to_
    //获取 mat 的行和列
 	//imshow("grayMaskSmallThresh",grayMaskSmallThresh);
	 //threshold(src, grayMaskSmallThresh, 128, 255, COLOR_THRESH_BINARY);//反相选择 >230 =0,else= 1 ，二值化的地图中，0代表可以到达。
	 medianBlur(grayMaskSmallThresh,grayMaskSmallThresh,3);

    //获取 mat 的行和列
 	// imshow("grayMaskSmallThresh",grayMaskSmallThresh);
	//waitKey(0);
    int row = src.rows;//320
    int col = src.cols;
	   
    cout << "  src.cols : " << src.cols << endl;//50 
    cout << "  src.rows : " << src.rows << endl;//50
    // 循环读取图形mat的值，并将mat对应值赋给二维数组对应值
    for (int i = 0; i < row   ; i ++){
        for (int j = 0; j < col; j ++){
            array_img[i][j] = grayMaskSmallThresh.at<uchar>(i, j);
        }
    }
  while(1){
	usleep(800000);
	if(start_find ==0)
	continue;
	printf("from(%d,%d)-->(%d,%d) \n",start_point_x,start_point_y,goal_point_x,goal_point_y);
	start_find =0;

	 
	
	 
	//Mat Aimg(WIDTH, HEIGHT, CV_8UC1,cv::Scalar(0));
	//memcpy(Aimg.data, array_img, WIDTH*HEIGHT*sizeof(unsigned char));
	// imshow("img",Aimg);
	//waitKey(0);
	//定义A *类
	CAStar* pAStar = new CAStar(array_img);
    if(array_img[start_point_x][start_point_y])
    {
        cout<<"start point set error!!!"<<endl;
       
    }
	 if(array_img[goal_point_x][goal_point_y])
    {
        cout<<" goal point set error!!!"<<endl;
        
    }
	//定义起始、结束、开始搜寻路径
    CPoint* start = new CPoint(start_point_x,start_point_y);
    CPoint* end = new CPoint(goal_point_x,goal_point_y);
	if( !pAStar->canReach(goal_point_x,goal_point_y))
	{
        cout<<" goal point set error!!!"<<endl;
        
    }
    CPoint* point = pAStar->FindPath(start, end, false);
    Rect rect;
    Point left_up,right_bottom;
	//最终图像尺寸500x500
   // Mat img(800,800,CV_8UC3,Scalar(255,255,255));
    std::cout << "最终的路径输出： "   << std::endl;
	vector<Point> pahtPoints;
	char skip =1;
    while(point != NULL)
    {

		//画矩形
       // rectangle(img,left_up,right_bottom,Scalar(0,255,255),CV_FILLED,8,0);//路径green颜色
		//circle(img,Point(point->Y,point->X),3,Scalar(0,255,0),-1);
        std::cout << "(" << point->X << "," << point->Y << ");" << std::endl;
		Point _tmpPoint;
		_tmpPoint.x = point->X;
		_tmpPoint.y = point->Y;
		//跳1 筛选路径点
		if(skip ==1){
			pahtPoints.push_back(_tmpPoint);
		}
		skip =!skip;
        point = point->m_parentPoint;
		

    }
 	printf("path len:%d \n",pahtPoints.size());
	set_callback_function(pahtPoints,path_public_callback);
 	clear_dque(pahtPoints);
   /* for(int i=0;i<WIDTH;i++)
    {
        for(int j=0;j<HEIGHT;j++)
        {   
            left_up.x = j*1; //存储数组的列(j)对应矩形的x轴
            left_up.y = i*1;  
            right_bottom.x = left_up.x+10;  
            right_bottom.y = left_up.y+10;
            if(array_img[i][j])
            {
				circle(img,Point(j,i),3,Scalar(0,0,0),-1);
               // rectangle(img,left_up,right_bottom,Scalar(0,0,0),CV_FILLED,8,0);//障碍物为黑色
            }
            else
            {
                if(i==start_point_x&&j==start_point_y)
                     rectangle(img,left_up,right_bottom,Scalar(255,0,0),CV_FILLED,8,0);//起始点为蓝色
                 else if(i==goal_point_x&&j==goal_point_y)
                     rectangle(img,left_up,right_bottom,Scalar(0,0,255),CV_FILLED,8,0);//目标点为红色
                 //else
                    // rectangle(img,left_up,right_bottom,Scalar(180,180,180),2,8,0);//空闲区为灰色
            }    
        }
    }
    //窗口中显示图像 
    imshow("astar",img);
	waitKey(0);*/
   // imwrite("astar.jpg",img);   
	 }
}
