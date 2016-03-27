#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"HrxTreeSlam");
    QApplication a(argc, argv);
    qRegisterMetaType<doubleVect >();
    MainWindow w;
    w.show();

    return a.exec();
}
