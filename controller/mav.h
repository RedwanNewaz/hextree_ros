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

     void pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
     void takerCallback(const std_msgs::String &msg);
     void camCallback(const geometry_msgs::PoseConstPtr);
     void goalCallback(const geometry_msgs::PoseStampedConstPtr);



private:

     QMutex mutex;

     ros::NodeHandle nh_;


     ros::Subscriber navdata_sub;
     ros::Subscriber imu_sub;
     ros::Subscriber ukf_sub;
     ros::ServiceServer service;

     std::vector<double> roboVect;
     bool scaleFix;


     //OCTOMAP
//     ros::Subscriber octomap_sub;
//     ros::Subscriber projectedmap_sub;

     ros::Subscriber orbTraker;
     ros::Subscriber cloud_sub;
     ros::Subscriber camPose_sub;
     ros::Subscriber dronepose_sub;
     ros::Subscriber traj_sub;


     ros::Subscriber rviz_goal;
     ros::Timer timer;
     vector<double>stateDis;
     bool controlPose;

protected:
     void timerCallback(const ros::TimerEvent& e);
};

#endif // MAV_H
