#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
import mary_tts.msg

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
    

greets = ["hello","good day","hi there"]

def randomgreet(req):
    t = random.choice(greets)
    return say(t)
    
    
def add_pushbutton_service():
    rospy.init_node('dispatcher')
    rospy.Service('qtgui/dispatcher/randomgreet',std_srvs.srv.Trigger,
        randomgreet)    

    print "pushbutton service up"
    rospy.spin()
    
if __name__ == "__main__":
    add_pushbutton_service()
