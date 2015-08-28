#!/usr/bin/python

import sys
import rospy
import std_srvs.srv


def pushbutton_callback(req):
    return std_srvs.srv.TriggerResponse(True,"boo");

def add_pushbutton_service():
    rospy.init_node('pushbutton')
    s = rospy.Service('pushbutton/one',std_srvs.srv.Trigger,pushbutton_callback)
    print "pushbutton service up"
    rospy.spin()
    
if __name__ == "__main__":
    add_pushbutton_service()
