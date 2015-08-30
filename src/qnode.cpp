/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <sstream>
#include "../include/qtgui/qnode.hpp"
#include "../include/qtgui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
init_argc(argc),
init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"qtgui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    hlevel_subscriber = n.subscribe<std_msgs::Float64>("hlevel",1000,
                                                       &QNode::hlevel_callback,this);
    smiles_subscriber = n.subscribe<std_msgs::Int8>("smile_detector",1000,
                                                       &QNode::smiles_callback,this);
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"qtgui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    
    start();
    printf("All init ok\n");
    return true;
}

void QNode::hlevel_callback(std_msgs::Float64 message){
    MainWindow::getInstance()->setHLevel(message.data);
}
void QNode::smiles_callback(std_msgs::Int8 message){
    MainWindow::getInstance()->setSmiles(message.data);
}

bool QNode::triggerService(const std::string &s){
    log(Info,"calling service " + s);
    ros::NodeHandle n;
    ros::ServiceClient c = n.serviceClient<std_srvs::Trigger>(s);
    std_srvs::Trigger t;
    if(c.call(t)){
        log(Info,"call succeeded");
        std::cout << "service called OK: " << t.response.message << std::endl;
        return true;
    } else {
        log(Error,"SERVICE CALL FAILED");
        return false;
    }
}         


void QNode::run() {
    ros::Rate loop_rate(5);
    int count = 0;
    while ( ros::ok() ) {
        
        // jcf - we're not sending these messages.        
        /*
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        chatter_publisher.publish(msg);
        log(Info,std::string("I sent: ")+msg.data);
        ++count;
*/
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qtgui
