#ifndef ROS_LAUNCH_H
#define ROS_LAUNCH_H
#include "header.h"


class robotViz;
using namespace std;
class ros_launch : public QThread
{
    Q_OBJECT

public:
    explicit ros_launch(QObject *parent = 0);
    void run();
    void callback_Image(const sensor_msgs::Image::ConstPtr& );
    void debugger_callback(const std_msgs::StringConstPtr);
    void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);

    std::string image_topic;
    ros_launch *sensor_subs;

    bool update_image,gui_comm,battery_state_update,msg_state;
    double battery_level;
    QString terminal_msg;
    cv_bridge::CvImageConstPtr cv_ptr ;


signals:
     void sig_image_pub(const QImage&);
     void sig_ardrone_battery(double);
     void sig_main_debugger(QString);
     void sig_light_intensity(double);


    
public slots:


private:
     ros::NodeHandle nh_;
     ros::Subscriber image_sub;
     ros::Subscriber navdata_sub;
     ros::Subscriber debugger_sub;
     ros::ServiceClient measurement_client;
     ros::Timer gui_update_timer;

     unsigned int navdataCount;
     QMutex mutex;
     QImage _image;



protected:
     void publish_image();
     void publish_battery_level();
     void publish_msg();
     void gui_update_execution(const ros::TimerEvent& e);






};

#endif // ROS_LAUNCH_H
