#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from math import exp

hconc=0
centre = 0
width = 0.1

# geometric decay rate
decayrate=rospy.get_param('/aes/decayrate',0.99)
# release multiplied by this
releasefactor=rospy.get_param('/aes/release',0.02)
# amount released by nice/nasty services
tempreleasefactor=rospy.get_param('/aes/nudge',1)
# amount released by smiles
smilereleasefactor=rospy.get_param('/aes/smile',0.2)

releaserate=0
smilecount=0
tempreleaserate=0

def sigmoid(x):
    x=(x-centre)/width
    # input centred at 0
    return 1 - (1/(1+exp(x)))

# print the sigmoid

for x in [x/10.0 for x in range(-10,10)]:
    print str(x)+" "+str(sigmoid(x))

def update():
    global hconc,decayrate,releaserate,tempreleaserate
    r = releaserate+tempreleaserate*tempreleasefactor+smilecount*smilereleasefactor
    r = r*releasefactor*(0.95-hconc)
    tempreleaserate=0
    hconc = (hconc+r)*decayrate

def startpublisherandwait():
    global hconc
    pubc = rospy.Publisher('hconc',Float64,queue_size=100)
    publ = rospy.Publisher('hlevel',Float64,queue_size=100)
    rate = rospy.Rate(3) # 3Hz
    while not rospy.is_shutdown():
        update()
        hlevel = sigmoid(hconc)
        print(str(hconc)+" -> "+str(hlevel))
        pubc.publish(hconc)
        publ.publish(hlevel)
        rate.sleep()
        
def setrelease(r):
    global releaserate
    releaserate = r
    
def setsmile(s):
    global smilecount
    smilecount = s
    
def bumprelease(r):
    global tempreleaserate
    tempreleaserate = r
    return std_srvs.srv.TriggerResponse(True,'');

def startsubscribers():
    subsmile = rospy.Subscriber('smile_detector',Int8,
        lambda x: setsmile(x.data))
    rospy.Subscriber('hrelease',Float64,
        lambda x: setrelease(x.data))
        
def startservices():
    rospy.Service('aes/nice',std_srvs.srv.Trigger,
        lambda req: bumprelease(0.5))
    rospy.Service('aes/nasty',std_srvs.srv.Trigger,
        lambda req: bumprelease(-0.5))

def start():
    rospy.init_node("aes",anonymous=True)
    startsubscribers()
    startservices()
    startpublisherandwait()
    


if __name__ == '__main__':
    start()
        
