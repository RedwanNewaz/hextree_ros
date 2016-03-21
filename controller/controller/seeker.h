#ifndef SEEKER_H
#define SEEKER_H

#include "../header.h"
#include "../../hexTree/datalogger.h"

class datalogger;
class seeker
{
public:
    seeker();
    void run(bool state);
private:
    ros::NodeHandle nh;
    ros::Publisher debugger_cntrl;
    ros::Subscriber xbee,traj_sub;
    ros::ServiceClient obs_srv;
    ros::ServiceServer attribute,threshold;

    //XBEE READING
    int obstacleStatus,index_travese,local_index,traj_length;
    float lightSensor[3];
    vector<vector<float> > light_map;
    vector<float>mesurements;
    bool active,active_sensing;
    datalogger *log;



protected:
    void xbeeRead(const geometry_msgs::QuaternionConstPtr);
    void trajCallback(const geometry_msgs::PoseArrayConstPtr msg);

    void SendMeasurementPacket();
    int terminate_exploration();

    void memory_allocate(int HEIGHT);
    void update_positions(float x, float y);
    void gather();
    void update_measurements();

    bool measurements_attribute(hextree::measurement::Request  &req,
                                hextree::measurement::Response &res);

    bool talk(hextree::plannertalk::Request  &req,
              hextree::plannertalk::Response &res);


};

#endif // SEEKER_H
