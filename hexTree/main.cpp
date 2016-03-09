#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hextree/plannertalk.h"
#include "geometry_msgs/PoseStamped.h"
#include "hotspot.h"




float robot[2]={0,0};


bool talk(hextree::plannertalk::Request  &req,
         hextree::plannertalk::Response &res)
{
   if(req.option>3){
       hotspot HP;


       switch(req.option){
        case 4:
           HP.debugger("SEEKER_1.20 ACTIVATED");
           HP.findHotspot();
           break;
        case 5:
           HP.debugger("SEEKER_1.20 SIMULATION");
           HP.simulation();
           break;
       }
       return true;
   }
     return (res.result=false);



}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"HotspotSeeking");
    ros::NodeHandle nh_;
    ros::ServiceServer mode = nh_.advertiseService("motionplan",talk);
    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();

    return 0;
}
