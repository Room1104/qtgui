#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
import mary_tts.msg

f = open('jokes.txt','r')
jokes = [x.strip() for x in f.read().split('%%')]

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from strands_executive_msgs.srv import AddTasks, DemandTask, SetExecutionStatus

def say(text):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    if(not c.wait_for_server(timeout=rospy.Duration(10))):
        print "Failed to connect to server"
        return std_srvs.srv.TriggerResponse(False,"failed to connect to server");
    goal = mary_tts.msg.maryttsGoal(text)
    c.send_goal(goal)
    if(not c.wait_for_result(timeout=rospy.Duration(10))):
        print "Failed to get result"
        return std_srvs.srv.TriggerResponse(False,"failed to get result");
    return std_srvs.srv.TriggerResponse(True,text);

def mksay(name,text):
    rospy.Service('qtgui/dispatcher/'+name,std_srvs.srv.Trigger,
        lambda req: say(text))

def randomjoke(req):
    t = random.choice(jokes)
    return say(t)
    
# Waypoint scheduling stuff

def get_service(service_name, service_type):    
    rospy.loginfo('Waiting for %s service...' % service_name)
    rospy.wait_for_service(service_name)
    rospy.loginfo("Done")        
    return rospy.ServiceProxy(service_name, service_type)
    


def get_execution_status_service():
    return get_service('/task_executor/set_execution_status', SetExecutionStatus)

def get_demand_task_service():
    return get_service('/task_executor/demand_task', DemandTask)

def get_add_tasks_service():
    return get_service('/task_executor/add_tasks', AddTasks)

def wait_waypoint1(req):
    wait_waypoint(1)
    return std_srvs.srv.TriggerResponse(True,'waypoint 1');
def wait_waypoint2(req):
    wait_waypoint(2)
    return std_srvs.srv.TriggerResponse(True,'waypoint 2');
def wait_waypoint3(req):
    wait_waypoint(3)
    return std_srvs.srv.TriggerResponse(True,'waypoint 3');
def wait_waypoint4(req):
    wait_waypoint(4)
    return std_srvs.srv.TriggerResponse(True,'waypoint 4');
def wait_waypoint5(req):
    wait_waypoint(5)
    return std_srvs.srv.TriggerResponse(True,'waypoint 5');

def wait_waypoint(wp_num):
    task = Task()
    task.action = '/wait_action'

    max_wait_secs = 20
    task.max_duration = rospy.Duration(max_wait_secs)

    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(10))

    task.start_after = rospy.get_rostime() + rospy.Duration(10)
    task.end_before = task.start_after + rospy.Duration(400)

    task.start_node_id = 'WayPoint' + str(wp_num)
    task.end_node_id = 'WayPoint' + str(wp_num)

    set_execution_status = get_execution_status_service()
    set_execution_status(True)

    demand_task = get_demand_task_service()
    demand_task(task) 
    

# Interface startup stuff

def start_services():
    rospy.Service('qtgui/dispatcher/1',std_srvs.srv.Trigger,
        wait_waypoint1 )    
    rospy.Service('qtgui/dispatcher/2',std_srvs.srv.Trigger,
        wait_waypoint2 )    
    rospy.Service('qtgui/dispatcher/3',std_srvs.srv.Trigger,
        wait_waypoint3 )    
    rospy.Service('qtgui/dispatcher/4',std_srvs.srv.Trigger,
        wait_waypoint4 )    
    rospy.Service('qtgui/dispatcher/5',std_srvs.srv.Trigger,
        wait_waypoint5 )    
    rospy.Service('qtgui/dispatcher/joke',std_srvs.srv.Trigger,
        randomjoke)
    print "services up"
    
def startup():
    rospy.init_node('dispatcher')
    start_services()
    rospy.spin()
    
if __name__ == "__main__":
    startup()
