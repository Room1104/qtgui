/**
 * @file /include/qtgui/main_window.hpp
 *
 * @brief Qt based gui for qtgui.
 *
 * @date November 2010
**/
#ifndef qtgui_MAIN_WINDOW_H
#define qtgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
          
      public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    
    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
    
    static MainWindow *inst;
public:
    static MainWindow *getInstance() {
        return inst;
    }
    void setHLevel(double level);
    void setSmiles(int n);
    void setLog(const std::string s);
Q_SIGNALS:
    void colchange();
public Q_SLOTS:
    void onClickButtonSpeak();
    void onClickButtonFind();
    void onClickButtonHome();
    void onClickButtonPet();
    void onClickButtonPunish();
    void onClickButton1();
    void onClickButton2();
    void onClickButton3();
    void onClickButton4();
    void onClickButton5();
    void updateColour();
private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    double hlevel;
};

}  // namespace qtgui

#endif // qtgui_MAIN_WINDOW_H
