#include "header.h"

unsigned int ros_header_timestamp_base = 0;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ActiveMotion");


    stateEstimation uav;
    controller cntrl;
    uav.run();
    cntrl.sensor_sub();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
