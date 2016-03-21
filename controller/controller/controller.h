#ifndef CONTROLLER_H
#define CONTROLLER_H

#define STOP 0.3
#define GARBAGE_MES 1000
#define UNITSTEP 0.1
#define STATE_SIZE 8
#define SIGNAL_SIZE 4
#define CONTROL_TIME 0.03


#include "../header.h"
#include "trajectory.h"
#include "seeker.h"
#include "../../hexTree/datalogger.h"


using namespace Eigen;
class datalogger;
class trajectory;
class seeker;

class controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    controller();
    void stateUpdate(const vector<double>& msg);
    void debugger(std::string);
    void readGain();



private:
    Eigen::Matrix4f kp,kd,dx_du;
    float state[STATE_SIZE],desired_location[SIGNAL_SIZE],
    input_u[SIGNAL_SIZE],state_err[STATE_SIZE];
    bool nocomm_vslam,land_cmd,stablizing,state_update,inputApply;
    ros::Timer timer;
    ros::NodeHandle nh;
    ros::Publisher debugger_cntrl,land_pub,vel_pub;
    ros::Subscriber xbee,camPose_sub;
    ros::ServiceClient obs_srv;
    ros::ServiceServer service,attribute,threshold,mode;

    int count,resetController,obstacleStatus;
    double error,vslam_count,gain[STATE_SIZE];

    //QT
    QMutex mutex;


//Trajectory
    trajectory *traj_cntrl;
    seeker *hotspot_seeker;


//datalogger
            datalogger *log,*cntrl_per,*gain_write;
            int global_index;
            vector<float> converage;

protected:

    void cmdPublish(ControlCommand);
    bool goalConverage();


    void dataWrite();
    void updateGain();
    void wrtieGain();



    //service
    bool gainchange(hextree::pidgain::Request  &req,
             hextree::pidgain::Response &res);

    void camCallback(const geometry_msgs::PoseConstPtr cam);

    bool compute_X_error();

//    controller design
    void run(const ros::TimerEvent& e);
    Eigen::Vector4f Ar_drone_input();
};

//PD default parameters
static const double KpX=0.11;         //0.5727  0.11  1.27;
    static const double KdX=0.11;		//1.53STATE_SIZE   0.22  0.25STATE_SIZESTATE_SIZE
static const double KpY=0.11; 		//0.51 0.2144 0.46STATE_SIZE5 0.2144;
    static const double KdY=0.11;		//0.323 0.4225 1.314 0.4225;
static const double KpZ=0.6;
    static const double KdZ=0.1;
static const double KpS=0.05;
    static const double KdS=0.0;





#endif // CONTROLLER_H
