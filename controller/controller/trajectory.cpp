#include "trajectory.h"

trajectory::trajectory()
{
    traj_sub=nh.subscribe("traj",5,&trajectory::trajCallback,this);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);
    mode = nh.advertiseService("motionplan",&trajectory::talk,this);
    waypoints_log =new datalogger;
    controller_Status=IDLE;
    goal_converage=true;

}

bool trajectory::find_waypoints(float *setpoints){

        update_setpoint(setpoints);
    return controller_Status;

}

void trajectory::update_setpoint(float *setpoints){

    if(controller_Status==IDLE)
        return;

    if(index_travese>traj_length-1)
    {
        debugger("Target achieved");
        controller_Status=IDLE;
        sleep(1);
        return;
    }
    if(goal_converage){
        ROS_INFO_STREAM("index "<<index_travese);
        for(int i=0;i<SIGNAL_SIZE;i++)
            setpoints[i]=Traj_array2D[index_travese][i];
        debugger("moving to ["+num2str(setpoints[0])+" ,"+num2str(setpoints[1])+"]");
        ROS_INFO_STREAM("moving to ["+num2str(setpoints[0])+" ,"+num2str(setpoints[1])+"]");
        goal_converage=false;
    }

 }

void trajectory::evolve_waypoints(int i){
        index_travese+=i;
    goal_converage=true;
}

void trajectory::modeSelction(int i){

   switch (i)
   {
       case 1:{//P2p
          memory_allocate(1);
          update_parameters(0, 1);
       break;}

       case 2:{ //square box
           memory_allocate(4);
           float x1[4]={ 0,  -0.5, -0.5,   0};
           float y1[4]={0.5, 0.5,  0,    0};
           for (int i=0;i<4;i++)
              update_parameters(x1[i], y1[i]);

       break;}
        case 3:{//read txt log
            float x2[100],y2[100];
            int n=waypoints_log->read_traj(x2, y2);
            memory_allocate(n);
            for (int i=0;i<n;i++)
               update_parameters(x2[i], y2[i]);
       break;}


   }
   if(i<4)
   controller_Status=EXECUTING;

}

void trajectory::trajCallback(const geometry_msgs::PoseArrayConstPtr msg){

     //Traj_array2D
     memory_allocate(msg->poses.size());
     foreach (geometry_msgs::Pose pose,msg->poses)
        update_parameters(pose.position.x, pose.position.y);
                 controller_Status=EXECUTING;

}

void trajectory::debugger(std::string ss){
    std_msgs::String debug_cntr;
    debug_cntr.data=ss;
    debugger_cntrl.publish(debug_cntr);
}

bool trajectory::talk(hextree::plannertalk::Request  &req,
         hextree::plannertalk::Response &res)
{
   modeSelction(req.option);
   res.result=true;
   return true;
}

void trajectory::memory_allocate(int HEIGHT){

    traj_length=HEIGHT;
    local_index=index_travese=0;
    Traj_array2D.clear();
    int WIDTH=SIGNAL_SIZE;
    Traj_array2D.resize(HEIGHT);
     for (int i = 0; i < HEIGHT; ++i)
       Traj_array2D[i].resize(WIDTH);

    debugger("new traj received "+num2str(HEIGHT));
    sleep(1);



}

void trajectory::update_parameters(float x, float y){
    if(local_index<traj_length){
        Traj_array2D[local_index][0]=x;
        Traj_array2D[local_index][1]=y;
        Traj_array2D[local_index][2]=1.2;
        Traj_array2D[local_index][3]=0;
        local_index++;


    }
}
