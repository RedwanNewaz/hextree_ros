#ifndef STATEESTIMATION_H
#define STATEESTIMATION_H

#include "../header.h"
#include "../../hexTree/datalogger.h"
#include "scale.h"

#define mm  1000.00
#define cm 1000.00
#define g 9.81
#define deg 0.0174532925
#define ALT_TH 2.5
#define zero_eq 0.001
#define STATE_UPDATE_TIME 0.5




class scale;
class datalogger;
using namespace Eigen;
using namespace std;



enum SENSOR_DATA{
    NAVDATA=10,
    IMUDATA=9,
    SLAMDATA=6,
 };

class stateEstimation
{
public:
    stateEstimation();

    void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
    void slamCb(const geometry_msgs::PoseConstPtr cam);
    void imudataCb(const sensor_msgs::ImuConstPtr msg);


    double battery();
    void ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg);
    std::vector<double> stateDisplay();

    vector<double> stateMSG();



private:
    float ukf_state[13],est_altd,VOSCALE,
          batteryStatus,zHeightoffset,
    track_vo_mu[SLAMDATA],track_vo_sig[SLAMDATA],
    track_nav_ori_mu[3],track_nav_ori_var[3],
    velocity_mean[SLAMDATA],velocity_var[SLAMDATA],
    angular_velocity[3],angular_velocity_var[3]
    ;

    int movingState,publish_data;
    bool VOSTART,initialized_scale,zOFFSET;
    QMutex mutex;


    scale *nav_raw,*slam_raw,*imu_raw;

    ros::NodeHandle nh;
    ros::Publisher debugger_cntrl,slam_pub,nav_pub,imu_pub;
    ros::Timer state_update_timer;
    nav_msgs::Odometry  odom_slam,odom_imu,odom_nav;

    //path planner localization
    ros::ServiceServer robot_srv,calib_srv;
    datalogger *ukf_log,*slam_log,*imu_log,*nav_log;


protected:

    void visual_odometry(float *position_mean, float *position_var, float *position_covar);

    void estimate_velocity(const float *mean, const float *track, float *velocity,int size);

    void imu_odometry(const float *position_mean,const float *position_var);

    void nav_odometry(const float *position_mean,const float *position_var);


    double timeDiff(ros::Time start);

    void sensor_log(const float *position_mean, const float *velocity_mean, SENSOR_DATA opt);

    nav_msgs::Odometry state_publish(const float *position_mean,const float *position_covar,
                                     const float *velocity_mean,const float *velocity_covar);
    void imu_linear_vel(float *linear_velocity,float *data_ori,
                        float *data_acc, float dt=0.5 );

    void logfile_Init();

    void state_update_execution(const ros::TimerEvent& e);

    bool localization(hextree::obstacle::Request  &req,
                      hextree::obstacle::Response &res);

    bool calibration(hextree::measurement::Request  &req,
                     hextree::measurement::Response &res);

    void debugger(std::string ss);
    void covariance(float *cov, const float *data);
    void dot_euler_angle(float *angular_velocity,float *data );
    void nav_linear_vel(float *linear_velocity,float *data );


    };

#endif // STATEESTIMATION_H
