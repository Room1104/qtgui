#!/usr/bin/python

import sys
import rospy
import actionlib
import std_srvs.srv
import mary_tts.msg

def pushbutton_callback(req):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    c.wait_for_server()
    goal = mary_tts.msg.maryttsGoal("wibble")
    c.send_goal(goal)
    c.wait_for_result()
    
    return std_srvs.srv.TriggerResponse(True,"boo");

def add_pushbutton_service():
    s = rospy.Service('pushbutton/one',std_srvs.srv.Trigger,
        pushbutton_callback)
    print "pushbutton service up"
    rospy.spin()
    
if __name__ == "__main__":
    rospy.init_node('pushbutton')
    pushbutton_callback("foo")
    add_pushbutton_service()
