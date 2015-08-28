#!/usr/bin/python

import sys
import rospy
import actionlib
import std_srvs.srv
import mary_tts.msg

def pushbutton_callback(text):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    c.wait_for_server()
    goal = mary_tts.msg.maryttsGoal(text)
    c.send_goal(goal)
    c.wait_for_result()
    return std_srvs.srv.TriggerResponse(True,text);


def mksay(name,text):
    rospy.Service('pushbutton/'+name,std_srvs.srv.Trigger,
        lambda req: pushbutton_callback(text))
    
def add_pushbutton_service():
    rospy.init_node('pushbutton')
    mksay("1","wibble")
    mksay("2","wobble")
    mksay("3","wabble")
    print "pushbutton service up"
    rospy.spin()
    
if __name__ == "__main__":
    add_pushbutton_service()
