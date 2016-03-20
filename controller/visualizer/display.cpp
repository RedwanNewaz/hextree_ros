#include "display.h"

display::display()
{
    robot_pub =obstacle_pub= nh_.advertise<visualization_msgs::Marker>("Quad/state/map", 1);
//    obstacle_pub = nh_.advertise<visualization_msgs::Marker>("Quad/state/obs", 1);
    pcl_pub2=nh_.advertise<sensor_msgs::PointCloud2>("Quad/map",10);
    debugger_pub=nh_.advertise<std_msgs::String>("jaistquad/debug",1);

    robot.header.frame_id=obstacle.header.frame_id=plan.header.frame_id =track_list.header.frame_id = "Quad";
    robot.header.stamp=obstacle.header.stamp=plan.header.stamp =track_list.header.stamp = ros::Time::now();
    robot.ns=obstacle.ns=plan.ns =track_list.ns = "stateEstimation";

    robot.id = 0;obstacle.id=1;
    robot.type =obstacle.type = visualization_msgs::Marker::ARROW;
    robot.action =obstacle.action=track_list.action = visualization_msgs::Marker::ADD;

    robot.scale.x  = 0.5;
    robot.scale.y  = 0.5;
    robot.scale.z  = 0.1;

    obstacle.scale.x  = 0.25;
    obstacle.scale.y  = 0.25;
    obstacle.scale.z  = 0.1;
    // Robot is red
    robot.color.r = 1.0f; obstacle.color.b = 1.0f;
    robot.color.a =obstacle.color.a= 1.0;
    robot.lifetime = ros::Duration();

    //configure pathPlan
    plan.pose.orientation.w = 1.0;
    plan.id = 2;
    plan.type = visualization_msgs::Marker::LINE_STRIP;

    plan.scale.x = 0.05;
    plan.scale.y = 0.05;
    plan.scale.z = 0.05;

    // track list is blue
    plan.color.r = 1.0;
    plan.color.a = 1.0;


    //configure track list
    track_list.pose.orientation.w = 1.0;
    track_list.id = 2;
    track_list.type = visualization_msgs::Marker::POINTS;

    track_list.scale.x = 0.05;
    track_list.scale.y = 0.05;
    track_list.scale.z = 0.05;
    // track list is blue
    track_list.color.b = 1.0;
    track_list.color.a = 1.0;


}

//PUBLISHER
void display::ukf_transformer( std::vector<double>robot_pose){
    geometry_msgs::Pose pose;


    debug_pose.str("");
    debug_pose<<"robot\t"<<robot_pose.at(0)<<"\t"<<robot_pose.at(1)<<"\t"<<robot_pose.at(2);
    debug_pose<<"\t"<<robot_pose.at(3)<<"\t"<<robot_pose.at(4)<<"\t"<<robot_pose.at(5);



//    ORIENTATION
    double  ro=robot_pose.at(3),
            po=robot_pose.at(4),
            yo=robot_pose.at(5);
//    cout<< setprecision(3)<<ro<<"\t"<<po<<"\t"<<yo;

    pose=euler2quaternion(ro,po,yo+M_PI/2);

//POSITION
    robo_x=pose.position.x=robot_pose.at(0)*VISFACTOR;
    robo_y=pose.position.y=robot_pose.at(1)*VISFACTOR;
    robo_z=pose.position.z=robot_pose.at(2)*VISFACTOR;


    robot.pose=pose;

//    Track Position
    positionTracker.x.push_back(robot_pose.at(0)*VISFACTOR);
    positionTracker.y.push_back(robot_pose.at(1)*VISFACTOR);
    positionTracker.z.push_back(robot_pose.at(2)*VISFACTOR);


    for (uint32_t i = 0; i < positionTracker.x.size(); ++i)
    {
      geometry_msgs::Point p;
      p.x = positionTracker.x.at(i);
      p.y = positionTracker.y.at(i);
      p.z = positionTracker.z.at(i);
      track_list.points.push_back(p);
    }



    robot_pub.publish(robot);
   // robot_pub.publish(track_list);


    debug_posi.data=debug_pose.str();
    //debugger_pub.publish(debug_posi);


}


