#ifndef ROBOTVIZ_H
#define ROBOTVIZ_H

#include "qcustomplot.h"
#include "../../gui/header.h"
#include "visualization_msgs/Marker.h"

namespace Ui {
class robotViz;
}

class robotViz : public QWidget
{
    Q_OBJECT

public:
    explicit robotViz(QWidget *parent = 0);
    ~robotViz();

    void robot_pos(float x, float y);

private:
    Ui::robotViz *ui;
    ros::NodeHandle nh_;
    ros::Subscriber robot_sub;


};

#endif // ROBOTVIZ_H
