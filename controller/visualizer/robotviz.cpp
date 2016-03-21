#include "robotviz.h"
#include "ui_robotviz.h"
#define MAX_AREA 15


robotViz::robotViz(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::robotViz)
{
    ui->setupUi(this);
   // ui->customPlot=new Qui->customPlot;
    QRect rec = QApplication::desktop()->screenGeometry();
    int x[1],y[1],w[1],h[1];
    rec.getRect(x,y,w,h);
    setGeometry(x[0]+150,y[0]+25,640,480);



    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-MAX_AREA/2, MAX_AREA);
    ui->customPlot->yAxis->setRange(-MAX_AREA/2, MAX_AREA);
    ui->customPlot->setBackground(Qt::transparent);
    ui->customPlot->setAttribute(Qt::WA_OpaquePaintEvent, false);

    ui->customPlot->addGraph(); // red line for traj
    ui->customPlot->graph(0)->setPen(QPen(Qt::red,2));
    ui->customPlot->addGraph(); // blue dot robot position
    ui->customPlot->graph(1)->setPen(QPen(Qt::blue,2));
    ui->customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->customPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->customPlot->addGraph(); // blue dot robot position
    ui->customPlot->graph(2)->setPen(QPen(QColor(255, 100, 0),2));
    ui->customPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->customPlot->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);


}

void robotViz::robot_pos(float x1, float y1)
{
    ui->customPlot->graph(1)->clearData();
    ui->customPlot->graph(1)->addData(x1, y1);
    ui->customPlot->replot();
if(abs(x1)>MAX_AREA||abs(y1)>MAX_AREA||abs(y1)<=0.001||abs(x1)<=0.001)return;

    ui->customPlot->graph(0)->addData(x1, y1);

    ui->customPlot->replot();

}

void robotViz::robot_traj(float x1, float y1)
{

    ui->customPlot->graph(2)->addData(x1, y1);
    ui->customPlot->replot();


}

robotViz::~robotViz()
{
    delete ui;
}
