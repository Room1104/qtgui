/**
 * @file /include/qtgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
**/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtgui_QNODE_HPP_
#define qtgui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
      public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    bool triggerService(const std::string &s);
    /*********************
    ** Logging
    **********************/
    enum LogLevel {
        Debug,
              Info,
              Warn,
              Error,
              Fatal
          };
    
    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);
    void hlevel_callback(std_msgs::Float64 message);
    void smiles_callback(std_msgs::Int8 message);
    
Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    
private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    ros::Subscriber hlevel_subscriber;
    ros::Subscriber smiles_subscriber;
    QStringListModel logging_model;
};

}  // namespace qtgui

#endif /* qtgui_QNODE_HPP_ */
