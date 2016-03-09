//FILE HEADER
#include "ui_mainwindow.h"
#include "mainwindow.h"
#include "ros_launch.h"
#include "ros_thread.h"

// CUSTOM HEADER
#include "hextree/pidgain.h"
#include "hextree/plannertalk.h"
#include "hextree/obstacle.h"
#include "hextree/measurement.h"





//ROS MESSAGES
#include "ros/ros.h"
#include <ros/macros.h>
#include <ros/master.h>

#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


//OPEN CV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



//STD LIB
#include <stdio.h>
#include <unistd.h>




//QT LIB

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <QLabel>
#include <QMainWindow>
#include <QMetaType>
#include <QMutex>
#include <QPainter>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QThread>
#include <QThreadPool>
#include <QTime>
#include <QTimer>
#include <QVector>
#include <QtAlgorithms>
#include <QtCore>
#include <QtGui>
#include <QtGlobal>











