#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define BUF_MAX 1024
#define MAX_CPU 128
#define RAD 57.2958

#include "header.h"
#include "../controller/visualizer/robotviz.h"

typedef std::vector<std::vector<float> >doubleVect;
Q_DECLARE_METATYPE(doubleVect);


class ros_launch;
class robotViz;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_buttonMoveit_clicked();

    void on_buttonStart_clicked();

    void on_buttonStop_clicked();

    void on_buttonTakeOff_clicked();

    void on_buttonLand_clicked();



    void timeChanged();
    void sub_write(const QImage &frame);
    void updateBattery(double status);
    void updateIntensity(double status);
    void sub_debug(QString);
    void traj_display(const doubleVect& traj);




    void on_button_motion_clicked();

    void on_btn_test_clicked();

    void on_btn_calibration_clicked();

private:
    Ui::MainWindow *ui;
    struct pid_gain{
        double kp,kd;
        bool change;
    }rollGain,pitchGain,altdGain,yawGain;
    QTimer *timer;
    QMutex mutex;

    //process start variables
    bool on_fly_status,moveit_enable;

    long displayCount,battery;

    QList<QString> pathFile,action;
    QString notification,raw_notification;
    ros_thread *drone_thread;
    ros_launch *sensor_subs;
    QStringList topicList,executionList;
    QProcess *process,*process_action;
    ros::ServiceClient robot_client,test_obs_clinet,calibration_client;
    ros::ServiceClient plannerclient;
    ros::NodeHandle n;


    //-------------------------

    //cpu usage variabale
        bool cpu_usage_active;
        QVector<double>robot_x, robot_y, map_y;
        FILE *fp;
        unsigned long long int fields[10], total_tick[MAX_CPU], total_tick_old[MAX_CPU], idle[MAX_CPU], idle_old[MAX_CPU], del_total_tick[MAX_CPU], del_idle[MAX_CPU];
        int update_cycle, i, cpus, count;
        double percent_usage;
     //------------------------------

        //subordinate program
    QProcess *process_cntrl,*process_plan, *process_lyap;

    bool subprograms;
    int lightIntensity;

//    vizualization motion
    robotViz *viz;





protected:
    void cpuInit();
    int read_fields (FILE *fp, unsigned long long int *fields);
    void cpuUsages();

    void pdLabel();
    void pdInitializer();


    void processInit();
    void processStarup();
    void notificationDisplay(QString msg);

    void cameraInit();
    void EnableSubscriber();
    void posi_display();
    void readWriteRes(QString read);


};

#endif // MAINWINDOW_H
