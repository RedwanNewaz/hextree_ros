#include "mav.h"

using namespace std;

mav::mav()
{
    stateSpace =new stateEstimation();
    visualize=new display();
    Cntrl=new controller();
    timer = nh_.createTimer(ros::Duration(0.25), &mav::timerCallback,this);
    controlPose=false;

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
        orbTraker=nh_.subscribe("ORB_SLAM/Debug",10,&mav::takerCallback, this);
//        cloud_sub=nh_.subscribe("/ORB_SLAM/PointCloud_raw",10,&mav::pointcloudCallback, this);
        camPose_sub=nh_.subscribe("/slam/camera",10,&mav::camCallback, this);
        //dronepose_sub=nh_.subscribe(nh_.resolveName("ardrone/predictedPose"),10,&mav::tumSatecallback, this);


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
        //ORB_SLAM DEPENDECIES
void mav::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
//        mapPublisher

}

void mav::takerCallback(const std_msgs::String &msg){
    string s;
    std::stringstream ss;
    ss<<msg;
    QStringList f;
        while (getline(ss, s, '\n')){
            QString str = QString::fromUtf8(s.c_str());
            f<<str;
        }
}



void mav::goalCallback(const geometry_msgs::PoseStampedConstPtr msg){
    mutex.lock();
    vector<double>p;
    p.push_back(msg->pose.position.x);
    p.push_back(msg->pose.position.y);
    p.push_back(1.5);
    p.push_back(0.0);

    controlPose=true;
    mutex.unlock();

}



