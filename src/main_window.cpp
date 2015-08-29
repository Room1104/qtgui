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
    connect(ui.pushButton,SIGNAL(clicked()),this,SLOT(onClickButton()));
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

void MainWindow::onClickButton(){
    std::cout << "Button\n";
    if(qnode.triggerService("qtgui/dispatcher/randomgreet"))
        std::cout << "Success\n";
    else
        std::cout << "Failure\n";
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



