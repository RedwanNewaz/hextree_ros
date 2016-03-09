#ifndef DISPLAY_H
#define DISPLAY_H
#define VISFACTOR 1
#include "../header.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include <visualization_msgs/Marker.h>

class display
{
public:
    display();
    void ukf_transformer( std::vector<double> pose);


private:
    ros::Publisher robot_pub,obstacle_pub;
    ros::Publisher pcl_pub2;

    ros::NodeHandle nh_;
    struct track{
        std::vector<double> x,y,z;
    }positionTracker;

    visualization_msgs::Marker robot,track_list,plan,obstacle;
    uint32_t shape;

    ros::Publisher debugger_pub;
    std::stringstream debug_pose;
    std_msgs::String debug_posi;
    double robo_x,robo_y,robo_z;

    enum orientation{
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD

    };


};

#endif // DISPLAY_H
