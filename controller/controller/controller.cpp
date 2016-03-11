/*STATEMENT OF PURPOSE:
 * 1) Dependencies trajectory controller for updating setpoint
 * 2) controller frequecy is 30 HZ
 * 3) For controlling XYZ we use PD controller and for Yaw only P controller
 * 4) Prediction based on controller gain is not implemented yet.
 * 5) DATA LOG: Time stamp, position and control command
 * 6) PID GAIN can be recorded to txt file
 */

#include "controller.h"

controller::controller()
{

    traj_cntrl=new trajectory;
    hotspot_seeker=new seeker;
    //PUBLISHER
    land_pub	   = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
    vel_pub	   = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);
    //SENSOR
    camPose_sub=nh.subscribe("/slam/camera",10,&controller::camCallback, this);
    //SERVICE
    service = nh.advertiseService("pid_gains",&controller::gainchange,this);
    global_index= error=0;


    //datalogger
    log=new datalogger;
    log->fileName("contoller_log");
    string a[2]={"waypoint_index", "uncertainty"};
    log->addHeader(a,2);

    gain_write=new datalogger;
    cntrl_per=new datalogger;
    cntrl_per->fileName("contoller_performance_log");   
    string b[11]={"xErr","yErr", "zErr","oErr","inX","inY","inZ","inO","MS","v_time","flag"};
    cntrl_per->addHeader(b,11);

//  controller intialize
    count=resetController=0;
    timer = nh.createTimer(ros::Duration(CONTROL_TIME), &controller::run,this);
    inputApply=false;


}

//----------------------------------ITERATIONS----------------------------------------------

void controller::run(const ros::TimerEvent& e){

    bool found_waypoint=traj_cntrl->find_waypoints(desired_location);

    if(!state_update ||!found_waypoint||!compute_X_error())
        return;
    else
    {

    double dt=(getMS()-vslam_count)/1000;
    if(dt>1)
        nocomm_vslam=true;
    else
        nocomm_vslam=false;


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
    Eigen::Vector4d u_cmd;
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


//----------------------------------CMD PUBLISH----------------------------------------------
 void controller::cmdPublish(ControlCommand cmd){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = cmd.yaw;
    cmdT.linear.z = cmd.gaz;
    cmdT.linear.x = cmd.pitch;
    cmdT.linear.y = -cmd.roll;
    vel_pub.publish(cmdT);


}

 Eigen::Vector4d controller::Ar_drone_input(){


     //UPDATE CMD BASED ON PD GAIN
     Eigen::Vector4d X_error_eig,X_dot_error_eig,cmd;
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
          cmd(3)=state_err[3]/abs(state_err[3])*UNITSTEP;

     ////ROS_INFO_STREAM("sig3 "<<cmd.transpose());

         // MAKE ALT CONTROLLER SLAGGISH
     cmd(2)*=UNITSTEP;

 //    GOAL CONVERGE
         if( goalConverage()||obstacleStatus||nocomm_vslam)
             for(int i=0;i<SIGNAL_SIZE;i++)
                 cmd(i)=0;

     // RECORD STATE & CMD TO A TXT FILE
     memcpy(input_u, cmd.data(), SIGNAL_SIZE * sizeof *cmd.data());

  ////ROS_INFO_STREAM("sig5 "<<cmd.transpose());

     dataWrite();

     return cmd;

 }

 bool controller::compute_X_error(){
      if(!state_update)return false;

      mutex.lock();
      float desired_state[STATE_SIZE];
      for(int i=0;i<STATE_SIZE;i++)
          if(i<SIGNAL_SIZE)
              desired_state[i]=desired_location[i]-state[i];
          else
              desired_state[i]=-state[i];
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

    if (!gain_write->read_pid_gain(gain,"pidtune"))
    {
        gain[0]=100*KpX;gain[2]=100*KpY;gain[4]=100*KpZ;gain[6]=100*KpS;
        gain[1]=100*KdX;gain[3]=100*KdY;gain[5]=100*KdZ;gain[7]=100*KdS;
        wrtieGain();
    }
    updateGain();


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

    float data[11];
    data[0]=getMS();
    for(int i=1;i<SIGNAL_SIZE;i++){
        data[i]=state_err[i-1];
        data[4+i]=input_u[i-1];
    }

    data[9]=vslam_count;
    data[10]=int(nocomm_vslam);

    cntrl_per->dataWrite(data,11);


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



