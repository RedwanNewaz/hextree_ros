#include "mav.h"

using namespace std;

mav::mav()
{
    stateSpace =new stateEstimation();
    visualize=new display();
    Cntrl=new controller();
    //timer = nh_.createTimer(ros::Duration(0.25), &mav::timerCallback,this);


}

mav::~mav()
{

}



void mav::run(){
        ROS_INFO("MOTION SUBSCRIBTION ENABLE");
        sleep(1);
        Cntrl->debugger("controller started vs 3.04");
        Cntrl->readGain();


        //SENSOR_FUSION DEPENDENCIES
        navdata_sub	   = nh_.subscribe(nh_.resolveName("ardrone/navdata"),50, &mav::navdataCb, this);
        imu_sub    = nh_.subscribe("/ardrone/imu",10, &mav::imudataCb, this);
        ukf_sub	   = nh_.subscribe("sensor/fusion",10, &mav::ukf_localization, this);

        //ORB_SLAM DEPENDECIES
        camPose_sub=nh_.subscribe("/slam/camera",10,&mav::camCallback, this);


}

void mav::timerCallback(const ros::TimerEvent& e){
     mutex.lock();
          visualize->ukf_transformer(stateSpace->stateDisplay());
     mutex.unlock();

 }

        //SENSOR_FUSION DEPENDENCIES
void mav::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
    mutex.lock();
    stateSpace->navdataCb(navdataPtr);
    mutex.unlock();


}

void mav::camCallback(const geometry_msgs::PoseConstPtr cam){
     mutex.lock();
     stateSpace->slamCb(cam);
     mutex.unlock();
}

void mav::imudataCb(const sensor_msgs::ImuConstPtr msg){
     mutex.lock();
     stateSpace->imudataCb(msg);
     mutex.unlock();


}

void mav::ukf_localization(const nav_msgs::OdometryConstPtr Odom_msg)
{
    mutex.lock();
        stateSpace->ukf_feedback(Odom_msg);
        Cntrl->stateUpdate(stateSpace->stateMSG());
    mutex.unlock();


}







