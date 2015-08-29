/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
**/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qtgui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
MainWindow *MainWindow::inst=NULL;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
      : QMainWindow(parent)
, qnode(argc,argv)
{
    inst = this;
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    printf("Main window init\n");
    //    if(!qnode.init("http://127.0.0.1:11311/","127.0.0.1"))
    if(!qnode.init())
        showNoMasterMessage();
    // wire up the UI elements here.
    connect(ui.pushButton_speak,SIGNAL(clicked()),this,SLOT(onClickButtonSpeak()));
    connect(ui.pushButton_1,SIGNAL(clicked()),this,SLOT(onClickButton1()));
    connect(ui.pushButton_2,SIGNAL(clicked()),this,SLOT(onClickButton2()));
    connect(ui.pushButton_3,SIGNAL(clicked()),this,SLOT(onClickButton3()));
    connect(ui.pushButton_4,SIGNAL(clicked()),this,SLOT(onClickButton4()));
    connect(ui.pushButton_5,SIGNAL(clicked()),this,SLOT(onClickButton5()));
}

MainWindow::~MainWindow() {}

void MainWindow::setHLevel(double level){
    ui.hlevel->setText(QString::number(level));
}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::onClickButtonSpeak(){
    qnode.triggerService("qtgui/dispatcher/randomgreet");
}
void MainWindow::onClickButton1(){
    qnode.triggerService("qtgui/dispatcher/1");
}
void MainWindow::onClickButton2(){
    qnode.triggerService("qtgui/dispatcher/2");
}
void MainWindow::onClickButton3(){
    qnode.triggerService("qtgui/dispatcher/3");
}
void MainWindow::onClickButton4(){
    qnode.triggerService("qtgui/dispatcher/4");
}
void MainWindow::onClickButton5(){
    qnode.triggerService("qtgui/dispatcher/5");
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace qtgui



