#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDir>
#include <QFile>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displayBattery->setPalette(Qt::red);
    ui->intensity->setPalette(Qt::blue);

    ui->robot_x->setPalette(Qt::red);
    ui->robot_y->setPalette(Qt::green);
    ui->robot_z->setPalette(Qt::blue);

    ui->robot_x_3->setPalette(Qt::black);
    ui->robot_y_3->setPalette(Qt::black);
    ui->robot_z_3->setPalette(Qt::black);


    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timeChanged()));

    notificationDisplay("Welcome to JAIST QUAD gui!");


    //cpu usage initializer
    cpuInit();

    //process initialization
    processInit();

    //cameara initialize
    cameraInit();
    lightIntensity=0;



    //start timer
    timer->start(250);

    ui->tc->setChecked(true);
    ui->Cntrl->setChecked(true);


}

MainWindow::~MainWindow()
{
    delete ui;
}

//System Button

void MainWindow::on_buttonMoveit_clicked()
{

    QStringList cntrl, planner;
    cntrl<<"hextree"<<"motion1";
    planner<<"hextree"<<"seeking1";

    process_cntrl =new QProcess(this);
    process_cntrl->start("rosrun",cntrl);

    if(ui->experiment->isChecked()){
         process_plan =new QProcess(this);
         process_plan->start("rosrun",planner);
     }


    subprograms=true;



}

void MainWindow::on_buttonStart_clicked()
{
    raw_notification="Ros thread is started";
    MainWindow::EnableSubscriber();
    drone_thread->start();

    //viz init

    viz =new robotViz(this);
    viz->showNormal();




}

void MainWindow::on_buttonStop_clicked()
{

       qDebug()<<"Application closing request accepted";
       notificationDisplay("Application closing request accepted");
       QFile("/home/redwan/Desktop/data/log.csv").remove();

       if (timer->isActive()){
           timer->stop();
           sleep(1);
           notificationDisplay("timer is stopped");
       }

       if(cpu_usage_active){
           fclose (fp);
           sleep(1);
           MainWindow::notificationDisplay("cpu Usage is closed");
           qDebug()<<"cpu Usage closed";
           cpu_usage_active=false;
       }


       if(subprograms){
           subprograms=false;
           process_cntrl->terminate();
           if(ui->experiment->isChecked())
           process_plan->terminate();
           if(moveit_enable)
           process_slam->terminate();
       }

       notificationDisplay(raw_notification);
        moveit_enable=false;
    QProcess::startDetached(QApplication::applicationFilePath());
    exit(1);
}

// UAV MOTION BUTTON

void MainWindow::on_buttonTakeOff_clicked()
{
    if (!on_fly_status){
        notificationDisplay("Jaist Quadrotor is taking Off");
        on_fly_status=true;
        drone_thread->sendTakeoff();
    }
    else
        notificationDisplay("Quad is already in flying mode");
}

void MainWindow::on_buttonLand_clicked()
{
    if(on_fly_status){
        raw_notification="Landing...";
        on_fly_status=false;
         drone_thread->sendLand();
    }
    else
         raw_notification="Quad is already in Landing mode";
      MainWindow::notificationDisplay(raw_notification);
}



// PROCESSES
void MainWindow::processInit()
{

    //thread intialization
    drone_thread=new ros_thread(this);
    sensor_subs=new ros_launch(this);

    battery=0;
    on_fly_status=false;
    moveit_enable=false;

    //enable debugging
    client=n.serviceClient<hextree::pidgain>("pid_gains");
    test_obs_clinet=n.serviceClient<hextree::measurement>("threshold");
    calibration_client=n.serviceClient<hextree::measurement>("calibration");
    robot_client=n.serviceClient<hextree::obstacle>("localization");
    plannerclient=n.serviceClient<hextree::plannertalk>("motionplan");
    connect(sensor_subs,SIGNAL(sig_main_debugger(QString)),this,SLOT(sub_debug(QString)));

    //enable image visualization
    connect(sensor_subs,SIGNAL(sig_image_pub(QImage)),this,SLOT(sub_write(QImage)));

}

void MainWindow::EnableSubscriber()
{
    raw_notification+="\n\t\t/*image subscription is enabled";
    QString en= topicList[ui->topics->currentIndex()];
    std::string img_sub_topic=en.toUtf8().constData();
    sensor_subs->image_topic=img_sub_topic;
    sleep(1);
    MainWindow::notificationDisplay(raw_notification);
    sensor_subs->start();

}

void MainWindow::sub_write(const QImage &frame)
{
    ui->image_view->setPixmap(QPixmap::fromImage(frame).scaled(frame.width(), frame.height(),Qt::KeepAspectRatio));

}

void MainWindow::updateBattery(double status)
{
    ui->displayBattery->display(status);
}

void MainWindow::updateIntensity(double status)
{
     ui->intensity->display(int(status));
     lightIntensity=status;
}
// UTILITIES

void MainWindow::processStarup(){
    if(!moveit_enable)
       {
           moveit_enable=true;
           process_slam =new QProcess;
           qDebug()<<pathFile;
           process_slam->start("roslaunch",pathFile);
           raw_notification="Processes are configured for operation...";
       }
       else
            raw_notification="Did you shutdown the processes?";


       MainWindow::notificationDisplay(raw_notification);
       qDebug()<<"processes are "<<process_slam->state();
}

void MainWindow::cameraInit(){
    connect(sensor_subs,SIGNAL(sig_ardrone_battery(double)),this,SLOT(updateBattery(double)));
    connect(sensor_subs,SIGNAL(sig_light_intensity(double)),this,SLOT(updateIntensity(double)));

    topicList<<"/ORB_SLAM/Frame"<<"/camera/image_raw"<<"/ardrone/image_raw";
     ui->topics->addItems(topicList);
}

void MainWindow::notificationDisplay(QString msg)
{
    displayCount++;
    if(displayCount%20==0)
        notification="";

    if(displayCount<2)
        notification+=QString::number(displayCount)+": "+ msg;
    else
        notification+="\n"+QString::number(displayCount)+": "+msg;

    ui->textStatus->setPlainText(notification);
}

void MainWindow::timeChanged(){

    posi_display();

    QDateTime display_time;
    //display current time
    ui->runTime->setDateTime(display_time.currentDateTime());
    //show cpu usages
    cpuUsages();



}

int MainWindow::read_fields (FILE *fp, unsigned long long int *fields)
{
  int retval;
  char buffer[BUF_MAX];


  if (!fgets (buffer, BUF_MAX, fp))
  { perror ("Error"); }
  /* line starts with c and a string. This is to handle cpu, cpu[0-9]+ */
  retval = sscanf (buffer, "c%*s %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu",
                            &fields[0],
                            &fields[1],
                            &fields[2],
                            &fields[3],
                            &fields[4],
                            &fields[5],
                            &fields[6],
                            &fields[7],
                            &fields[8],
                            &fields[9]);
  if (retval == 0)
  { return -1; }
  if (retval < 4) /* Atleast 4 fields is to be read */
  {
    fprintf (stderr, "Error reading /proc/stat cpu field\n");
    return 0;
  }
  return 1;
}

void MainWindow::cpuInit(){
    update_cycle = 0;
    cpus = 0;
    fp = fopen ("/proc/stat", "r");
    if (fp == NULL)
     {
       qDebug()<<"Error!!! cpu file can't read";
        cpu_usage_active=false;
     }
    else{
         timer->start(1000);
      cpu_usage_active=true;
     }
    while (MainWindow::read_fields (fp, fields) != -1)
     {
       for (i=0, total_tick[cpus] = 0; i<10; i++)
       { total_tick[cpus] += fields[i]; }
       idle[cpus] = fields[3]; /* idle ticks index */
       cpus++;
     }
}

void MainWindow::cpuUsages(){
    fseek (fp, 0, SEEK_SET);
             fflush (fp);
             for (count = 0; count < cpus; count++)
             {
               total_tick_old[count] = total_tick[count];
               idle_old[count] = idle[count];

               if (!MainWindow::read_fields (fp, fields))
               { return ; }

               for (i=0, total_tick[count] = 0; i<10; i++)
               { total_tick[count] += fields[i]; }
               idle[count] = fields[3];

               del_total_tick[count] = total_tick[count] - total_tick_old[count];
               del_idle[count] = idle[count] - idle_old[count];

               percent_usage = ((del_total_tick[count] - del_idle[count]) / (double) del_total_tick[count]) * 100;
               if (count == 0)
               {
                ui->cpu_usage->setValue(percent_usage);
               }
             }
             update_cycle++;



}

//Debuging
void MainWindow::posi_display(){

       hextree::obstacle srv;
       if(!robot_client.call(srv))
           return;
    //update position
       ui->robot_x->display(srv.response.state[0]);
       ui->robot_y->display(srv.response.state[1]);
       ui->robot_z->display(srv.response.state[2]);
    //update orientation
       ui->robot_x_3->display(int(srv.response.state[3]*RAD));
       ui->robot_y_3->display(int(srv.response.state[4]*RAD));
       ui->robot_z_3->display(int(srv.response.state[5]*RAD));

       viz->robot_pos(srv.response.state[0],srv.response.state[1]);

}

void MainWindow::sub_debug(QString msg){

     notificationDisplay(msg);

 }


void MainWindow::on_button_motion_clicked()
{
    hextree::plannertalk srv;
    if(ui->p2p->isChecked())
        srv.request.option=1;
    else if (ui->sb->isChecked())
        srv.request.option=2;
    else if (ui->ca->isChecked())
        srv.request.option=3;
    else if (ui->tc->isChecked()&&ui->experiment->isChecked())
        srv.request.option=4;
    else if (ui->tc->isChecked()&&ui->planning->isChecked())
        srv.request.option=5;

    if (plannerclient.call(srv))
        ROS_INFO_STREAM("plannerMotion is changed "<<srv.response);

}

void MainWindow::on_btn_test_clicked()
{
    int x=lightIntensity-ui->threshold_box->currentIndex()-1;
        hextree::measurement srv;
        srv.request.state=x;
        if (test_obs_clinet.call(srv))
            ROS_INFO_STREAM(" reponseded  "<<x);
}

void MainWindow::on_btn_calibration_clicked()
{
    hextree::measurement srv;
    int option =ui->scale_cal->currentIndex();
    switch(option){
        case 0:srv.request.state=50;break;
        case 1:srv.request.state=5;break;
    }

    if (calibration_client.call(srv))
        ROS_INFO_STREAM(" reponseded  ");
}
