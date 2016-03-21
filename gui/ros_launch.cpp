#include "ros_launch.h"
#define GUI_UPDATE_TIME 0.3

ros_launch::ros_launch(QObject *parent) :
    QThread(parent)
{

}

void ros_launch::run()
{
    std::cout<< "topic in string: "  <<image_topic<<std::endl;

    sensor_subs=new ros_launch(this);
    sensor_subs->update_image=false;

    gui_update_timer= nh_.createTimer(ros::Duration(GUI_UPDATE_TIME), &ros_launch::gui_update_execution,this);

    image_sub=nh_.subscribe(image_topic, 1, &ros_launch::callback_Image,sensor_subs);
    navdata_sub	   = nh_.subscribe(nh_.resolveName("ardrone/navdata"),50, &ros_launch::navdataCb, sensor_subs);
    debugger_sub=nh_.subscribe("jaistquad/debug", 1, &ros_launch::debugger_callback,sensor_subs);
    measurement_client=nh_.serviceClient<hextree::measurement>("measurement");
    traj_sub=nh_.subscribe("traj",5,&ros_launch::trajCallback,sensor_subs);



    ros::MultiThreadedSpinner();
}

void ros_launch::callback_Image(const sensor_msgs::Image::ConstPtr& msg)
{

   if(gui_comm)return;
   cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
   update_image=true;

}

void ros_launch::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
 if(gui_comm)return;
    battery_level=navdataPtr->batteryPercent;
    battery_state_update=true;

}

void ros_launch::debugger_callback(const std_msgs::StringConstPtr msg){

    if(gui_comm)return;
    QString str = QString::fromUtf8(msg->data.c_str());
    terminal_msg+=str+"\n";
    msg_state=true;

}

void ros_launch::trajCallback(const geometry_msgs::PoseArrayConstPtr msg){

     //Traj_array2D=[x;y]
     gui_traj.clear();
     gui_traj.resize(2);
     foreach (geometry_msgs::Pose pose,msg->poses){
     gui_traj[0].push_back(pose.position.x);
     gui_traj[1].push_back(pose.position.y);
     }
     ROS_INFO_STREAM("gui_traj size "<<gui_traj[0].size());
     gui_traj_recv=true;



}


// update timer
void ros_launch::gui_update_execution(const ros::TimerEvent& e)
{

    sensor_subs->gui_comm=true;
    publish_msg();
    publish_image();
    publish_battery_level();

    if(sensor_subs->gui_traj_recv){
        ROS_WARN("TRAJ sending ..");
    emit sig_trajectory(sensor_subs->gui_traj);
    sensor_subs->gui_traj_recv=false;}

    sensor_subs->gui_comm=false;

}

void ros_launch::publish_image()
{
    if(sensor_subs->update_image){
        cv::Mat conversion_mat_ = sensor_subs->cv_ptr->image;
        QImage _img(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
        emit sig_image_pub(_img.rgbSwapped());
        sensor_subs->update_image=false;
    }
}

void ros_launch::publish_battery_level()
{
    if(sensor_subs->battery_state_update)
    {
        emit sig_ardrone_battery(sensor_subs->battery_level);
        hextree::measurement attribute;
        double backgroundMeasurement=0;
        attribute.request.state=1;
        if(measurement_client.call(attribute))
            backgroundMeasurement=attribute.response.result;
        emit sig_light_intensity(backgroundMeasurement);
        sensor_subs->battery_state_update=false;
    }
}

void ros_launch::publish_msg()
{
    if(sensor_subs->msg_state)
    {
        emit sig_main_debugger(sensor_subs->terminal_msg);
        sensor_subs->terminal_msg="";
        sensor_subs->msg_state=false;

    }

}

