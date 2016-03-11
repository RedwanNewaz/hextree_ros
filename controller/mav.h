#ifndef MAV_H
#define MAV_H

#include "header.h"



class controller;
class display;
class stateEstimation;


class mav
{

public:
     mav();
    ~mav();

     stateEstimation *stateSpace;
     controller *Cntrl;
     display *visualize;

     void run();

     void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
     void imudataCb(const sensor_msgs::ImuConstPtr msg);
     void ukf_localization(const nav_msgs::OdometryConstPtr Odom_msg);

     void camCallback(const geometry_msgs::PoseConstPtr);



private:

     QMutex mutex;

     ros::NodeHandle nh_;
     ros::Subscriber navdata_sub,imu_sub,ukf_sub,camPose_sub;
     ros::ServiceServer service;
     ros::Timer timer;



protected:
     void timerCallback(const ros::TimerEvent& e);
};

#endif // MAV_H
