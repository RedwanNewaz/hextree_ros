/*SCALE estimation problems
 * VO and IMU has different scale
 * we need to calibrate each of them
 * From the 1meter real world distance
 * TODO make a gui button for calibration
 */


#include "stateestimation.h"


 stateEstimation::stateEstimation()
{
    nav_raw=new scale();
    slam_raw=new scale();
    imu_raw=new scale();
    nav_raw->initialize(NAVDATA);
    imu_raw->initialize(IMUDATA);
    slam_raw->initialize(SLAMDATA);

    //VALUE INITIALIZATION
    VOSTART=zOFFSET=initialized_scale=false;
    batteryStatus=VOSCALE=zHeightoffset=publish_data=zero_eq;
    for(int i(0);i<SLAMDATA;i++){
        track_vo_mu[i]=track_vo_sig[i]=zero_eq;
        velocity_mean[i]=velocity_var[i]=zero_eq;
        if(i<3){
            track_nav_ori_mu[i]=track_nav_ori_var[i]=zero_eq;
             angular_velocity[i]=angular_velocity_var[i]=zero_eq;
        }
    }



    state_update_timer= nh.createTimer(ros::Duration(STATE_UPDATE_TIME), &stateEstimation::state_update_execution,this);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);

    //path planner localization
    robot_srv = nh.advertiseService("localization",&stateEstimation::localization,this);
    calib_srv = nh.advertiseService("calibration",&stateEstimation::calibration,this);


    //PUBLISHER TOPICS
     slam_pub= nh.advertise<nav_msgs::Odometry>("ORB_SLAM", 5);
     nav_pub= nh.advertise<nav_msgs::Odometry>("navData", 5);
     imu_pub= nh.advertise<nav_msgs::Odometry>("imu", 5);

     //writing files
     logfile_Init();

}

  //SENSOR CALLBACK

 void stateEstimation::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr){

    batteryStatus=navdataPtr->batteryPercent;
    est_altd=navdataPtr->altd/cm;
    movingState=navdataPtr->state;
//SLAM [Roll Pitch Yaw]-->TRANSFORM--> [-Pitch Roll Yaw]
    float nav_raw_state[NAVDATA]={
            navdataPtr->vx/mm,navdataPtr->vy/mm,navdataPtr->vz/mm,
            navdataPtr->ax*g,navdataPtr->ay*g,g-navdataPtr->az*g,
           -navdataPtr->rotY*deg,navdataPtr->rotX*deg,navdataPtr->rotZ*deg,
            navdataPtr->altd/cm
    };

    if(VOSTART)
        nav_raw->update_parameters(nav_raw_state);

}

 void stateEstimation::slamCb(const geometry_msgs::PoseConstPtr cam){

   //FUSION [X Y Z Roll Pitch Yaw]-->TRANSFORM--> [X Z Y Roll Yaw -Pitch]
    float slam_raw_state[SLAMDATA]={
       VOSCALE*cam->position.x,VOSCALE*cam->position.z,VOSCALE*cam->position.y,
       cam->orientation.x,cam->orientation.z,-cam->orientation.y
    };


    if(VOSTART)
    slam_raw->update_parameters(slam_raw_state);


}

 void stateEstimation::imudataCb(const sensor_msgs::ImuConstPtr msg){
    geometry_msgs::Quaternion orientation=msg->orientation;
    geometry_msgs::Vector3 angular_velocity=msg->angular_velocity;
    geometry_msgs::Vector3 linear_acceleration=msg->linear_acceleration;


    double roll, pitch, yaw;
    tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//SLAM [Roll Pitch Yaw]-->TRANSFORM--> [-Pitch Roll Yaw]
    float imu_raw_state[IMUDATA]={
        -pitch,roll,yaw,
        linear_acceleration.x,linear_acceleration.y,g-linear_acceleration.z,
        angular_velocity.x,angular_velocity.y,angular_velocity.z

    };

    if(VOSTART)
        imu_raw->update_parameters(imu_raw_state);

}

 //TIMER

 void stateEstimation::visual_odometry(float *position_mean, float *position_var, float *position_covar)
 {

     float velocity_covar[36];
     slam_raw->get_parameters(position_mean,position_var);

          //ESTIMATE  VELOCITY
     estimate_velocity(position_mean,track_vo_mu,velocity_mean,SLAMDATA);
     estimate_velocity(position_var,track_vo_sig,velocity_var,SLAMDATA);
     memcpy(track_vo_mu,position_mean,SLAMDATA*sizeof *position_mean);
     memcpy(track_vo_sig,position_var,SLAMDATA*sizeof *position_var);
     covariance(position_covar,position_var);
    if(nan_array(position_covar,36) )
    {
        debugger("slam pos covs are nan");
        return;
    }
     covariance(velocity_covar,velocity_var);

   if(nan_array(velocity_covar,36))
   {
       debugger("slam vel covs are nan");
       return;
   }
     odom_slam=state_publish(position_mean,position_covar,velocity_mean,velocity_covar);
     slam_pub.publish(odom_slam);
     sensor_log(position_mean,velocity_mean,SLAMDATA);

 }

 void stateEstimation::imu_odometry(const float *position_mean,const float *position_var){
     float IMU_mean[IMUDATA],IMU_var[IMUDATA];
     float angular_velocity[3],angular_velocity_var[3];
     float imu_position_mean[6],imu_position_var[6],imu_position_covar[36];
     float imu_linear_velocity[3], imu_linear_velocity_var[3],
             imu_velocity_mean[6],imu_velocity_var[6],imu_velocity_covar[36],
             data_ori[2],data_acc[3],data_ori_var[2],data_acc_var[3];


     //GET SENOSR READING
     imu_raw->get_parameters(IMU_mean,IMU_var);

     //POSITION &ORIENTATION :ORGANIZE VARIABLES
     dot_euler_angle(angular_velocity,IMU_mean);
     dot_euler_angle(angular_velocity_var,IMU_var);

     for(int i(0);i<3;i++)
     {
         imu_position_mean[i]=position_mean[i];
         imu_position_mean[3+i]=IMU_mean[i];
         imu_position_var[i]=position_var[i];
         imu_position_var[3+i]=IMU_var[i];
     }

     covariance(imu_position_covar,imu_position_var);

     if(nan_array(imu_position_covar,36))
     {
         debugger("imu position are nan");
         return;
     }

     //COMPUTE LINEAR VELOCITIES

     for(int i(0);i<3;i++)
     {
         data_acc[i]=IMU_mean[3+i];
         data_acc_var[i]=IMU_var[i+3];
      if(i<2){
             data_ori[i]=IMU_mean[i];
             data_ori_var[i]=IMU_var[i];
         }

     }

     imu_linear_vel(imu_linear_velocity,data_ori,data_acc);
     imu_linear_vel(imu_linear_velocity_var,data_ori_var,data_acc_var);

     for(int i(0);i<3;i++){
         imu_velocity_mean[i]=imu_linear_velocity[i];
         imu_velocity_mean[3+i]=IMU_mean[6+i];
         imu_velocity_var[i]=imu_linear_velocity_var[i];
         imu_velocity_var[3+i]=IMU_var[6+i];

     }

     covariance(imu_velocity_covar,imu_velocity_var);

     if(nan_array(imu_velocity_covar,36))
        {
            debugger("imu covs are nan");
            return;
        }

    odom_imu=state_publish(imu_position_mean,imu_position_covar,imu_velocity_mean,imu_velocity_covar);
    imu_pub.publish(odom_imu);
    sensor_log(imu_position_mean,imu_velocity_mean,IMUDATA);




 }

 void stateEstimation::nav_odometry(const float *position_mean,const float *position_var)
 {
     float NAV_mean[NAVDATA],NAV_var[NAVDATA];
     float linear_velocity[3],linear_velocity_var[3];
     float nav_velocity_mean[SLAMDATA],nav_velocity_var[SLAMDATA],
           nav_velocity_covar[36],nav_position_covar[36];
     float nav_position_mean[6],nav_position_var[6],
             euler_angle_mu[3],euler_angle_sig[3];

     //GET SENOSR READING
     nav_raw->get_parameters(NAV_mean,NAV_var);

     //POSITION &ORIENTATION :ORGANIZE VARIABLES
     for(int i(0);i<3;i++)
     {
         euler_angle_mu[i]=NAV_mean[i+6];
         euler_angle_sig[i]=NAV_var[i+6];
         nav_position_mean[i]=position_mean[i];
         nav_position_mean[3+i]=euler_angle_mu[i];
         nav_position_var[i]=position_var[i];
         nav_position_var[3+i]=euler_angle_sig[i];
     }
     covariance(nav_position_covar,nav_position_var);

     //COMPUTE LINEAR VELOCITIES
     nav_linear_vel(linear_velocity,NAV_var);
     nav_linear_vel(linear_velocity_var,NAV_var);


     if(nan_array(nav_position_covar,36) )
     {
         debugger("nav covs are nan");
         return;
     }


     //ESTIMATE ANGULAR VELOCITIES
     estimate_velocity(nav_position_mean,track_nav_ori_mu,angular_velocity,3);
     estimate_velocity(nav_position_var,track_nav_ori_var,angular_velocity_var,3);
     memcpy(track_nav_ori_mu,euler_angle_mu,3*sizeof *euler_angle_mu);
     memcpy(track_nav_ori_var,euler_angle_sig,3*sizeof *euler_angle_sig);

     //ORIENTATION :ORGANIZE VARIABLES
     for(int i=0;i<3;i++){
         nav_velocity_mean[i]=linear_velocity[i];
         nav_velocity_mean[3+i]=angular_velocity[i];
         nav_velocity_var[i]=linear_velocity_var[i];
         nav_velocity_var[3+i]=angular_velocity_var[i];
     }
     covariance(nav_velocity_covar,nav_velocity_var);

     if(nan_array(nav_velocity_covar,36))
     {
         debugger("nav covs are nan");
         return;
     }

     odom_nav=state_publish(nav_position_mean,nav_position_covar,nav_velocity_mean,nav_velocity_covar);
     nav_pub.publish(odom_nav);

     sensor_log(nav_position_mean,nav_velocity_mean,NAVDATA);
 }

 void stateEstimation::state_update_execution(const ros::TimerEvent& e){
     if(!VOSTART)return;
     mutex.lock();
    float position_mean[SLAMDATA],position_var[SLAMDATA],position_covar[36];
    visual_odometry(position_mean,position_var,position_covar);
    imu_odometry(position_mean,position_var);
    nav_odometry(position_mean,position_var);
    publish_data++;
    mutex.unlock();

 }


// PUBLISH

 nav_msgs::Odometry stateEstimation::state_publish(const float *position_mean,const float *position_covar,
                                                   const float *velocity_mean,const float *velocity_covar){
     nav_msgs::Odometry  odom;

     odom.header.frame_id="slam";
     odom.header.stamp=ros::Time::now();

//    position update

     //update orientation first
     odom.pose.pose=euler2quaternion(position_mean[3],position_mean[4],position_mean[5]);

     odom.pose.pose.position.x=position_mean[0];
     odom.pose.pose.position.y=position_mean[1];
     odom.pose.pose.position.z=position_mean[2];


     odom.twist.twist.linear.x=velocity_mean[0];
     odom.twist.twist.linear.y=velocity_mean[1];
     odom.twist.twist.linear.z=velocity_mean[2];

     odom.twist.twist.angular.x=velocity_mean[3];
     odom.twist.twist.angular.y=velocity_mean[4];
     odom.twist.twist.angular.z=velocity_mean[5];
     for(int i=0;i<36;i++){
      odom.pose.covariance[i]=position_covar[i];
      odom.twist.covariance[i]=velocity_covar[i];
     }
    return odom;
 }

 //SERVICES

 bool stateEstimation::calibration(hextree::measurement::Request  &req,
                   hextree::measurement::Response &res)
 {
     stringstream ss;

     VOSCALE=req.state;
     VOSTART =true;
     initialized_scale=false;

     ss<<"VO SCALE "<<VOSCALE;
     debugger("calibration started "+ss.str());
     sleep(1);
     return initialized_scale=true;
 }

 bool stateEstimation::localization(hextree::obstacle::Request  &req,
         hextree::obstacle::Response &res)
{
    ROS_INFO_STREAM("localization message received");


    for(int i(0);i<3;i++){// populate x,y,z
        res.state[i]=ukf_state[1+i];
        if(i<2){//populate roll pitch
           res.state[4+i]=ukf_state[10+i];
        }
    }
    //populate yaw
    res.state[3]=ukf_state[4];
    return true;

}



// INFORMATION

 double stateEstimation::battery(){
    return batteryStatus;
}

 void stateEstimation::ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg){

 if(publish_data<2)return;
     float x=Odom_msg->pose.pose.position.x,
           y=Odom_msg->pose.pose.position.y,
           z=Odom_msg->pose.pose.position.z;
     float vx=Odom_msg->twist.twist.linear.x,
           vy=Odom_msg->twist.twist.linear.y,
           vz=Odom_msg->twist.twist.linear.z;
     float phi_dot=Odom_msg->twist.twist.angular.x,
           theta_dot=Odom_msg->twist.twist.angular.y,
           psi_dot=Odom_msg->twist.twist.angular.z;
     geometry_msgs::Quaternion orientation=Odom_msg->pose.pose.orientation;
     double phi, theta, psi;//roll,pitch,yaw
     tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
     tf::Matrix3x3(q).getRPY(phi, theta, psi);




    mutex.lock();
    float arr[13]={getMS(), x,y,-z+zHeightoffset,psi,
                   vx,vy,vz,psi_dot,phi,
                   theta,phi_dot,theta_dot};
    memcpy(ukf_state, arr, 13 * sizeof *arr);
    mutex.unlock();
    ukf_log ->dataWrite(ukf_state,13);



 }

 vector<double>  stateEstimation::stateMSG(){

     vector<double>msg;
     mutex.lock();
     for(int i=1;i<9;i++)
         msg.push_back(ukf_state[i]);
     mutex.unlock();
     return msg;

 }

//NOTIFICATIONS & VIZUALIZATION

 void stateEstimation::logfile_Init(){

     ROS_WARN("initialization of datalogger");


     ukf_log =new datalogger;
     slam_log   =new datalogger;
     imu_log   =new datalogger;
     nav_log   =new datalogger;

     ukf_log->fileName("ukf_log");
     slam_log->fileName("slam_log");
     imu_log->fileName("imu_log");
     nav_log->fileName("nav_log");



     string headername[] = {"MS","x","y","z","psi","vx","vy","vz","psi_dot","phi","theta","phi_dot","theta_dot"   };
     string est_headername[] = {"MS","x","y","z","vx","vy","vz","rx","py","yz","rx_dot","py_dot","yz_dot"   };

     sleep(1);

     ukf_log    ->addHeader(headername,13);
     slam_log   ->addHeader(est_headername,13);
     imu_log    ->addHeader(est_headername,13);
     nav_log    ->addHeader(est_headername,13);

 }

 void stateEstimation::sensor_log(const float *position_mean, const float *velocity_mean, SENSOR_DATA opt){
     float a[13]={getMS(),position_mean[0],position_mean[1],position_mean[2],
                  velocity_mean[0],velocity_mean[1],velocity_mean[2],
                  position_mean[3],position_mean[4],position_mean[5],
                  velocity_mean[3],velocity_mean[4],velocity_mean[5]};
     switch (opt){
        case SLAMDATA:slam_log ->dataWrite(a,13);break;
        case IMUDATA:imu_log ->dataWrite(a,13);break;
        case NAVDATA:nav_log ->dataWrite(a,13);break;
     }
 }

 void stateEstimation::debugger(std::string ss){
     std_msgs::String debug_cntr;
     debug_cntr.data=ss;
     debugger_cntrl.publish(debug_cntr);
    // ROS_INFO_STREAM(ss);
 }

 std::vector<double> stateEstimation:: stateDisplay(){
    std::vector<double> robot;
     for(int i=1;i<5;i++)
       robot.push_back(ukf_state[i]);
//roll_pitch update
      for(int i=10;i<12;i++)
         robot.push_back(ukf_state[i]);
       return robot;
  }

// MATHS AND USFUL FUNCTION
 void stateEstimation::covariance(float *cov, const float *data){
    Eigen::VectorXd vat(6);
    Eigen::MatrixXd mat(6,6);
    for(int i=0;i<6;i++)
        vat[i]=data[i];
    mat=vat*vat.transpose();
    int count=0;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            if(i==j)
            cov[count]=data[i];
            else
                cov[count]=0;

            count++;

        }
    }
}

 void stateEstimation::dot_euler_angle(float *angular_velocity,float *data ){
     for(int i=0;i<3;i++)
         angular_velocity[i]=data[i];
 }

 void stateEstimation::nav_linear_vel(float *linear_velocity,float *data ){
      //FUSION [vX vY vZ Roll Pitch Yaw]-->TRANSFORM--> [-vY -vX vZ Roll Pitch Yaw]
     double yawRad = data[9];
     linear_velocity[1] =- (sin(yawRad) * data[0] + cos(yawRad) * data[1]) ;
     linear_velocity[0] =- (cos(yawRad) * data[0] - sin(yawRad) * data[1]) ;
     linear_velocity[2] =data[2];
 }

 void stateEstimation::imu_linear_vel(float *linear_velocity,float *data_ori,float *data_acc){
     float dt=STATE_UPDATE_TIME;
     //FUSION [vX vY vZ Roll Pitch Yaw]-->TRANSFORM--> [vZ vY vX -Pitch roll Yaw]
     linear_velocity[2] =  (-data_acc[1]*dt    +g*sin(data_ori[0]));
     linear_velocity[1] =  (data_acc[0]*dt    +g*cos(data_ori[0])*sin(-data_ori[1])) ;
     linear_velocity[0] =  (data_acc[2]*dt);
 }

 void stateEstimation::estimate_velocity(const float *mean, const float *track, float *velocity, int size){
     for(int i(0);i<size;i++)
     {
        float vo= mean[i]-track[i];
        if(vo==0)continue;
        velocity[i]=vo/STATE_UPDATE_TIME;

     }
 }
