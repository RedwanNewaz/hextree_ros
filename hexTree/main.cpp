#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "hotspot.h"




int main(int argc, char *argv[])
{
    ros::init(argc,argv,"HotspotSeeking");
    hotspot HP;
    HP.run();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
