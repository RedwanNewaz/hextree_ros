#include "datalogger.h"
#include "ros/ros.h"
#include <QDebug>

datalogger::datalogger()
{
    root_path=path="/home/redwan/Desktop/data/";
    QString filename=path+"log.csv";

    if(QFile(filename).exists()){

        //read log file
        QString proposed=read_log_file(filename);
        //add folder name to path
        path=proposed;

    }
    else{
    //create new log file
        fileName("log");
    //create new folder
        QString proposed="EX_";
        QString folder_name=path+proposed;
        for(int i=1;QDir(folder_name).exists();i++)
            folder_name=path+proposed+QString::number(i);
        QDir().mkdir(folder_name);

    //write folder name to log file
        folder_name+="/";

        string a[1]={folder_name.toUtf8().constData()};
        addHeader(a,1);
        path=folder_name;
    }


}

void datalogger::fileName(QString proposed,bool root){


    if(root)
    {
        name=root_path+proposed+".csv";
        initialization=true;
        return;
    }

    QString filename=path+proposed;
   if(QFile(filename).exists())
    for(int i=0;QFile(filename).exists();i++)
       filename=filename+QString::number(i)+".csv";
   else
       filename+=".csv";
    name=filename;
    initialization=true;



}

void datalogger::dataWrite(float *a, float size)
{
    if(!initialization)return;
    QFile file(name);
    file.open(QIODevice::Append | QIODevice::Text);

    if(file.isOpen()){
        QTextStream outStream(&file);
        for(int i=0;i<size;i++)
            outStream<<a[i]<<"\t";
        outStream<<"\n";

    }
    file.close();

}

void datalogger::addHeader(string *v,int sizearr){
    if(!initialization)return;
    QFile file(name);
    file.open(QIODevice::Append | QIODevice::Text);

    if(file.isOpen()){
        QTextStream outStream(&file);
        for(int i=0;i<sizearr;i++)
            outStream<<QString::fromUtf8( v[i].c_str())<<"\t";
        outStream<<"\n";

    }
    file.close();

}

void datalogger::changePath(QString new_path){
    path=new_path;

}

QString datalogger::read_log_file(QString log){
    QFile file(log);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QString data="EX";
    while(!file.isOpen())
        file.close();

        data=file.readLine();
    return data;
}

int datalogger::read_traj(float* x, float* y){
       QString log="/home/redwan/Desktop/data/trajectory.csv";
    read_traj_file_protect( log);
    for (unsigned int i=0;i<traj_x.size();i++){
        x[i]=traj_x.at(i);
        y[i]=traj_y.at(i);
    }
    return traj_x.size();
}

void datalogger::read_traj_file_protect(QString log){
    QFile file(log);
    if(!file.exists())return;
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    traj_x.clear();traj_y.clear();

    while(!file.isOpen())
        file.close();
    while(!file.atEnd()){
        QString data=file.readLine();
        QStringList list=data.split("\t",QString::SkipEmptyParts);

        if(list.size()>1){
        traj_x.push_back(list[0].toDouble());
        traj_y.push_back(list[1].toDouble());
         qDebug()<<list[0].toDouble()<<"\t"<<list[1].toDouble();
        }
    }

}

bool datalogger::read_pid_gain(double *gain,QString proposed){


    QFile file(root_path+proposed+".csv");
    if(!file.exists())return false;
    if (file.open(QIODevice::ReadOnly))
    {
        QStringList list;
        int i(0);
        while (!file.atEnd()&& i<8){
              list.append(file.readLine());
              gain[i]=double(list[i].toDouble());
        }


    }

        return true;

}
