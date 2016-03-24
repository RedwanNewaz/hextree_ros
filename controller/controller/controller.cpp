/*STATEMENT OF PURPOSE:
 * 1) Dependencies trajectory controller for updating setpoint
 * 2) controller frequecy is 30 HZ
 * 3) For controlling XYZ we use PD controller and for Yaw only P controller
 * 4) Prediction based on controller gain is not implemented yet.
 * 5) DATA LOG: Time stamp, position and control command
 * 6) PID GAIN can be recorded to txt file
 */

#include "controller.h"

#include "lyap_control/controller_msg.h"
#include "lyap_control/plant_msg.h"

controller::controller()
{

    traj_cntrl=new trajectory;
    hotspot_seeker=new seeker;
    //PUBLISHER
    lyap_pub	   = nh.advertise<lyap_control::plant_msg>("state",1);

    land_pub	   = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);

    vel_pub	   = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);
    //SERVICE
    service = nh.advertiseService("pid_gains",&controller::gainchange,this);
    global_index= error=0;
    stablizing =false;


    //datalogger
    log=new datalogger;
    log->fileName("contoller_log");
    string a[2]={"waypoint_index", "uncertainty"};
    log->addHeader(a,2);

    gain_write=new datalogger;
    cntrl_per=new datalogger;
    cntrl_per->fileName("contoller_performance_log");   
    string b[15]={"MS","Set_x","Set_y","Set_z","Set_Yaw","x","y","z","Yaw","inX","inY","inZ","inYaw","state_update","sig_flag"};

    cntrl_per->addHeader(b,15);

//  controller intialize
    count=resetController=0;
    timer = nh.createTimer(ros::Duration(CONTROL_TIME), &controller::run,this);
    inputApply=false;
    readGain();


}

//----------------------------------ITERATIONS----------------------------------------------

void controller::run(const ros::TimerEvent& e){

    bool found_waypoint=traj_cntrl->find_waypoints(desired_location);

    /* TESTING CODE
        if(found_waypoint)
            resetController++;
        if(resetController>30){
            traj_cntrl->evolve_waypoints(1);
            resetController=0;
            hotspot_seeker->run(false);
        }
        if(resetController>15)
        hotspot_seeker->run(true);
        return;
    */


    if(!found_waypoint||!compute_X_error()){

        ControlCommand c;
        c.roll =0;   //moving X direction
        c.pitch=0; //moving Y direction
        c.gaz  =0;    //moving Z direction
        c.yaw  =0; //change Yaw
        cmdPublish(c);}
    else
    {


    double dt=(getMS()-vslam_count)/1000;
    if(dt>1)
        nocomm_vslam=true;
    else
        nocomm_vslam=false;

//lyap controller
    lyap_control::plant_msg plant_state;
    plant_state.t=getMS()/1000.00;
    for (int i(0);i<SIGNAL_SIZE;i++){
        if(stablizing||nocomm_vslam)
            plant_state.setpoint[i]=plant_state.x[i]=state[i];
        else{
        plant_state.setpoint[i]=desired_location[i];
        plant_state.x[i]=state[i];}
    }

    lyap_pub.publish(plant_state);


    if(resetController>30){
        global_index++;
        float a[2]={global_index,min_ele_vec(converage)};
        traj_cntrl->evolve_waypoints(1);
        log->dataWrite(a,2);
        ROS_INFO_STREAM("Target "<< a[0] <<" Achieved "<< a[1]);
        stablizing=false;
        resetController=0;
        debugger("goal converged "+num2str(min_ele_vec(converage)));
        converage.clear();
    }
    Eigen::Vector4f u_cmd;
    mutex.lock();
            u_cmd = Ar_drone_input();

    mutex.unlock();


    ControlCommand c;
    c.roll=u_cmd(0);   //moving X direction
    c.pitch=u_cmd(1); //moving Y direction
    c.gaz=u_cmd(2);    //moving Z direction
    c.yaw=u_cmd(3); //change Yaw
    cmdPublish(c);
   // controller_gain_update();
    inputApply=true;
    state_update=false;

    hotspot_seeker->run(stablizing);

    }
 }

void controller::stateUpdate(const vector<double>& tmsg){

mutex.lock();
     if(tmsg.size()<STATE_SIZE)
        debugger("controller Error in state update");
     std::copy(tmsg.begin(), tmsg.end(), state);

     state_update=true;
mutex.unlock();

 }

void controller::sensor_sub(){
    //SENSOR
    camPose_sub=nh.subscribe("/slam/camera",10,&controller::camCallback, this);
    lyap_sub=nh.subscribe("control_effort",10,&controller::lyapCallback, this);
    ukf_sub	   = nh.subscribe("sensor/fusion",10, &controller::ukf_feedback, this);


}

//----------------------------------CMD PUBLISH----------------------------------------------
 void controller::cmdPublish(ControlCommand cmd){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = cmd.yaw;
    cmdT.linear.z = cmd.gaz;
    cmdT.linear.x = cmd.pitch;
    cmdT.linear.y = -cmd.roll;
    vel_pub.publish(cmdT);


}

 Eigen::Vector4f controller::Ar_drone_input(){


     //UPDATE CMD BASED ON PD GAIN
     Eigen::Vector4f X_error_eig,X_dot_error_eig,cmd;

     /* PD CONTROLLER IMPLEMENTATION
     for(int i=0;i<SIGNAL_SIZE;i++)
     {
         X_error_eig(i)=state_err[i];
         X_dot_error_eig(i)=state_err[4+i];
     }
     cmd=kp*(X_error_eig)+kd*X_dot_error_eig;
     ////ROS_INFO_STREAM("sig1 "<<cmd.transpose());

     // ROUNDING INPUTS
     float max=cmd.maxCoeff();
     for(int i=0;i<SIGNAL_SIZE;i++)
         if(abs(cmd(i))<0.33*abs(max))// TOO SMALL!
             cmd(i)=0;
         else if (abs(cmd(i))>0)
             cmd(i)=cmd(i)/abs(max)*UNITSTEP;
         ////ROS_INFO_STREAM("sig2 "<<cmd.transpose());

     // MAKE YAW CONTROLLER SLAGGISH
     double max_yaw=10*deg;

     if (abs(state_err[3])<max_yaw)
         cmd(3)=0;
     else if (abs(state_err[3])>0)
          cmd(3)=state_err[3]/abs(state_err[3])*UNITSTEP/2;

     ////ROS_INFO_STREAM("sig3 "<<cmd.transpose());

         // MAKE ALT CONTROLLER SLAGGISH
     cmd(2)*=UNITSTEP;

     */


     for(int i(0);i<SIGNAL_SIZE;i++)
         cmd(i)= input_u[i];

     double max_yaw=10*deg;
     if (abs(state_err[3])<max_yaw)
         cmd(3)=0;

    ROS_INFO("lyap (%f %f %f %f)",input_u[0],input_u[1],input_u[2],input_u[3]);

 //    GOAL CONVERGE
         if( goalConverage()||nocomm_vslam)
             for(int i=0;i<SIGNAL_SIZE;i++)
                 cmd(i)=0;

     // RECORD STATE & CMD TO A TXT FILE
         for(int i(0);i<SIGNAL_SIZE;i++)
             input_u[i]=cmd(i);
     //memcpy(input_u, cmd.data(), SIGNAL_SIZE * sizeof *cmd.data());
//

  ROS_INFO_STREAM("sig5 "<<cmd.transpose());

     dataWrite();

     return cmd;

 }

 bool controller::compute_X_error(){
      if(!state_update)return false;

      mutex.lock();
      static float init_yaw=state[3];
      desired_location[3]=init_yaw;
      float desired_state[STATE_SIZE];
      for(int i=0;i<STATE_SIZE;i++)
      {
          if(i<SIGNAL_SIZE)
              desired_state[i]=desired_location[i]-state[i];
          else
              desired_state[i]=-state[i];
      }
      memcpy(state_err, desired_state, STATE_SIZE * sizeof *desired_state);
      inputApply=false;
      mutex.unlock();
      return true;
  }

 bool controller::goalConverage(){
     error=0;
     for (int i=0;i<2;i++)
         error+=pow(desired_location[i]-state[i],2);

 //    ROS_INFO_STREAM("setpoint "<<desired_location[0]<<"\t"<<desired_location[1]);
      //check goal radius
     if(sqrt(error)<STOP ||stablizing)
     {
         if(!stablizing)
             debugger("goal stablizing "+num2str(sqrt(error)));

         converage.push_back(sqrt(error));
         stablizing=true;

         resetController+=1;
         return true;
     }
         else
         return false;
 }

//----------------------------------STORE PID GAIN-----------------------------------------------------------------
void controller::wrtieGain(){

    float a[STATE_SIZE];
    for (int i(0);i<STATE_SIZE;i++)
        a[i]=gain[i];
    gain_write->fileName("pidtune",1);
    gain_write->dataWrite(a,STATE_SIZE);
}


void controller::readGain(){

//    if (!gain_write->read_pid_gain(gain,"pidtune"))
//    {
//        gain[0]=100*KpX;gain[2]=100*KpY;gain[4]=100*KpZ;gain[6]=100*KpS;
//        gain[1]=100*KdX;gain[3]=100*KdY;gain[5]=100*KdZ;gain[7]=100*KdS;
//        wrtieGain();
//    }
    gain[0]=KpX;gain[1]=KpY;gain[2]=KpZ;gain[3]=KpS;
    gain[4]=KdX;gain[5]=KdY;gain[6]=KdZ;gain[7]=KdS;

    //updateGain();
    for(int i(0);i<4;i++)
        for(int j(0);j<4;j++)
            if(i==j){
                kp(i,j)=gain[i];
                kd(i,j)=gain[4+i];
            }
            else{
                kp(i,j)=0;
                kd(i,j)=0;
            }

    ROS_INFO_STREAM("Kp\n"<<kp);
    ROS_INFO_STREAM("Kd\n"<<kd);


}

void controller::updateGain(){

    kp<<gain[0]/100.00,0,0,0,0,gain[2]/100.00,0,0,0,0,gain[4]/100.00,0,0,0,0,gain[6]/100.00;
    kd<<gain[1]/100.00,0,0,0,0,gain[3]/100.00,0,0,0,0,gain[5]/100.00,0,0,0,0,gain[7]/100.00;
    debugger("pidtune gain update successfully");
}

//----------------------------------GUI COMMUNICATION and DATA LOG------------------------------------------------------------
void controller::debugger(std::string ss){
    std_msgs::String debug_cntr;
    debug_cntr.data=ss;
    debugger_cntrl.publish(debug_cntr);
    //ROS_INFO_STREAM(ss);
}

void controller::dataWrite(){
  //  string b[15]={"MS","Set_x","Set_y","Set_z","Set_Yaw","x","y","z","Yaw","inX","inY","inZ","inYaw","state_update","sig_flag"};

    float data[15];
    data[0]=getMS();
    for(int i=0;i<SIGNAL_SIZE;i++){
        data[i+1]=desired_location[i];
        data[i+5]=state[i];
        data[i+9]=input_u[i];
    }

    data[9+4]=int(state_update);
    data[10+4]=int(nocomm_vslam);

    cntrl_per->dataWrite(data,15);


}

//----------------------------------SERVICES----------------------------------------------

bool controller::gainchange(hextree::pidgain::Request  &req,
         hextree::pidgain::Response &res)
{
    res.result=true;
    switch(req.id){
    case 1:gain[0]=req.P;gain[1]=req.D;debugger("roll gain updated");break;
    case 2:gain[2]=req.P;gain[3]=req.D;debugger("pitch gain updated");break;
    case 3:gain[4]=req.P;gain[5]=req.D;debugger("altd gain updated");break;
    case 4:gain[6]=req.P;gain[7]=req.D;debugger("yaw gain updated");break;
    case 5:wrtieGain();break;
    }
    sleep(1);
    if(req.id<=4)
    updateGain();
    return true;
}

void controller::camCallback(const geometry_msgs::PoseConstPtr cam){
    vslam_count=getMS();
}

void controller::lyapCallback(const lyap_control::controller_msgConstPtr msg){
    float factor[]={100,100,1000,1000};
    for(int i(0);i<SIGNAL_SIZE;i++)
        input_u[i]=msg->u[i]/factor[i];

}

void controller::ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg){

            state[0]=Odom_msg->pose.pose.position.x;
            state[1]=Odom_msg->pose.pose.position.y;
            state[2]=Odom_msg->pose.pose.position.z;

            geometry_msgs::Quaternion orientation=Odom_msg->pose.pose.orientation;
            double phi, theta, psi;//roll,pitch,yaw
            tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
            tf::Matrix3x3(q).getRPY(phi, theta, psi);
            state[3]=psi;
            state[4]=Odom_msg->twist.twist.linear.x;
            state[5]=Odom_msg->twist.twist.linear.y;
            state[6]=Odom_msg->twist.twist.linear.z;
            state[7]=Odom_msg->twist.twist.angular.z;
            state_update=true;

}
