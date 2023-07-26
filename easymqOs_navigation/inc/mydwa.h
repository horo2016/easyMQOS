

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
struct State {
    float x_;
    float y_;
    float theta_;
    float v_;
    float w_;
};

struct Control {
    float v_;
    float w_;
};

using Traj = std::vector<State>;
using Obstacle = std::vector<Point>;;
class mydwa {
public:
    mydwa();

    // bool stepOnceToGoal(std::vector<State>* best_trajectry, State* cur_state,Obstacle *cur_obs);
	 int update_obstacle(Obstacle obs);
     void  update_robot_and_goal(float x,float y,float theta,Point goal);
     State motion(State x, Control u);
    Traj calc_final_cost(Control& u);

	// int  state_error_check(float head_d);
     Point goal_;//目标点
     Obstacle obs_;//障碍物
     State cur_x_;//机器人的状态
private:
    

     Traj calc_trajectory(State x, float v, float w);
     float calc_obstacle_cost(Traj traj);
     float calc_degree_cost(Traj traj);
	
    


    float _vMin;//线速度
    float _vMax;
    //角速度
    float _wMin;
    float _wMax;

};
