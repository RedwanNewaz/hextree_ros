#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "../header.h"
#include "../../hexTree/datalogger.h"

class trajectory
{
public:
    trajectory();
    bool find_waypoints(float *setpoints);
    void evolve_waypoints(int);

private:
    ros::NodeHandle nh;
    ros::Publisher debugger_cntrl;
    ros::Subscriber traj_sub;
    ros::ServiceServer mode;

    datalogger *waypoints_log;
    int traj_length,index_travese,local_index;
    bool goal_converage;
    vector<vector<float> > Traj_array2D;

    enum target{
        EXECUTING=1,
        IDLE=0
    }controller_Status;

    std::string str;




protected:
    void trajCallback(const geometry_msgs::PoseArrayConstPtr msg);
    void update_setpoint(float *setpoints);
    void modeSelction(int);
    void debugger(std::string);
    void memory_allocate(int);
    void update_parameters(float x, float y);

    bool talk(hextree::plannertalk::Request  &req,
             hextree::plannertalk::Response &res);



};

#endif // TRAJECTORY_H
