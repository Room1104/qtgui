#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
from std_msgs.msg import Float64
from math import exp

hlevel=0
centre = 0
width = 0.1
decayrate=0.95 # geometric decay
releaserate=0
tempreleaserate=0

def sigmoid(x):
    x=(x-centre)/width
    # input centred at 0
    return 1 - (1/(1+exp(x)))

# print the sigmoid

for x in [x/10.0 for x in range(-10,10)]:
    print str(x)+" "+str(sigmoid(x))



def update():
    global hlevel,decayrate,releaserate,tempreleaserate
    r = (releaserate+tempreleaserate)*0.1*(0.95-hlevel)
    tempreleaserate=0
    hlevel = (hlevel+r)*decayrate
    return sigmoid(hlevel)

def startpublisherandwait():
    pub = rospy.Publisher('hlevel',Float64,queue_size=100)
    rate = rospy.Rate(3) # 3Hz
    while not rospy.is_shutdown():
        h=update()
        print(h)
        pub.publish(h)
        rate.sleep()
        
def setrelease(r):
    global releaserate
    releaserate = r
    
def bumprelease(r):
    global tempreleaserate
    tempreleaserate = r
    return std_srvs.srv.TriggerResponse(True,'');

def startsubscriber():
    rospy.Subscriber('hrelease',Float64,
        lambda x: setrelease(x.data))
        
def startservices():
    rospy.Service('aes/nice',std_srvs.srv.Trigger,
        lambda req: bumprelease(0.5))
    rospy.Service('aes/nasty',std_srvs.srv.Trigger,
        lambda req: bumprelease(-0.5))

def start():
    rospy.init_node("aes",anonymous=True)
    startsubscriber()
    startservices()
    startpublisherandwait()
    


if __name__ == '__main__':
    start()
        
