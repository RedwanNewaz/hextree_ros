/* STATEMENT OF PURPOSE:
 * 1) Seeker need activation from GUI
 * 2) The local lightmap is generated from the HexTree trajectory
 * 3) Light intensity is published by arduino serial node
 * 4) The MAIN program 'run' gathers measurement while
 *    UAV is in stablizing mode
 * 5) After stablization average measurements are computed for
 *    the feedback of HexTree node
 */

#include "seeker.h"

seeker::seeker()
{
    xbee	   = nh.subscribe("xbeeReading",50, &seeker::xbeeRead, this);
    traj_sub=nh.subscribe("traj",5,&seeker::trajCallback,this);

    obs_srv=nh.serviceClient<hextree::sensor>("seeker/measurements");
    attribute = nh.advertiseService("measurement",&seeker::measurements_attribute,this);
    threshold = nh.advertiseService("threshold",&seeker::measurements_threshold,this);

    active_sensing=active=false;
    for (int i=0;i<3;i++)
        lightSensor[i]=0;
}

void seeker::xbeeRead(const geometry_msgs::QuaternionConstPtr msg){
   lightSensor[0]=msg->x;
   lightSensor[1]=msg->y;
   lightSensor[2]=msg->z;
   obstacleStatus=msg->w;
}

void seeker::SendMeasurementPacket(){

    active_sensing=false;
    mesurements.clear();
    hextree::sensor lightIntensity;

    lightIntensity.request.count=terminate_exploration();
    for( int i=0;i<traj_length;i++)
            lightIntensity.request.reading[i]=light_map[2][i];
    if(obs_srv.call(lightIntensity))
        ROS_WARN("Data send");

    //wait a bit

    sleep(1);


}

int seeker::terminate_exploration(){
    bool terminate=false;
    for( int i=0;i<traj_length;i++)
    {
        float goal[2]={0,7},power=0;
        for (unsigned int j(0);j<2;i++)
            power+=pow(light_map[i][j]-goal[j],2);
        if(sqrt(power)<1)
           return terminate=true;
    }
    return terminate;
}

void seeker::trajCallback(const geometry_msgs::PoseArrayConstPtr msg){

     //light_map
    if(!active)return;
     memory_allocate(msg->poses.size());
     foreach (geometry_msgs::Pose pose,msg->poses)
        update_positions(pose.position.x, pose.position.y);
     active_sensing=true;

}

void seeker::memory_allocate(int HEIGHT){

    traj_length=HEIGHT;
    local_index=index_travese=0;
    light_map.clear();
    int WIDTH=SIGNAL_SIZE;
    light_map.resize(HEIGHT);
     for (int i = 0; i < HEIGHT; ++i)
       light_map[i].resize(WIDTH);

}

void seeker::update_positions(float x, float y){
    if(local_index<traj_length){
        light_map[local_index][0]=x;
        light_map[local_index][1]=y;
        local_index++;
    }
}

void seeker::update_measurements(){

    if(mesurements.empty())return;
    float x=mean(mesurements),
          y=var(mean(mesurements),mesurements);
    if(index_travese<traj_length){
        light_map[index_travese][2]=x;
        light_map[index_travese][3]=y;

        float a[5]={getMS(),
                    light_map[index_travese][0],
                    light_map[index_travese][1],
                    light_map[index_travese][2],
                    light_map[index_travese][3]};
        log->dataWrite(a,5);


        index_travese++;
    }
    else
      SendMeasurementPacket();



}


bool seeker::measurements_attribute(hextree::measurement::Request  &req,
                                   hextree::measurement::Response &res)
{
    float mes=0;
    for (int i=0;i<3;i++)
        mes+=lightSensor[i];
    res.result=mes;
    return true;
}

bool seeker::measurements_threshold(hextree::measurement::Request  &req,
                                   hextree::measurement::Response &res)
{
    log =new datalogger;
    log->fileName("hexTree_measurement");
    string a[5]={"MS","x","y","mu","sigma"};
    log->addHeader(a,5);

    return active=true;
}
void seeker::gather(){
    float mes=0;
    for (int i=0;i<3;i++)
        mes+=lightSensor[i];
    mesurements.push_back(mes);
    ROS_INFO_STREAM("mesurements size "<<mesurements.size());
}

void seeker::run(bool state){
    if(active_sensing && active)
    return(state?gather():update_measurements() );

}
