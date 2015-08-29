#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
import mary_tts.msg

def say(text):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    c.wait_for_server()
    goal = mary_tts.msg.maryttsGoal(text)
    c.send_goal(goal)
    c.wait_for_result()
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
