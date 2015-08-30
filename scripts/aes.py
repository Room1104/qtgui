#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
import mary_tts.msg
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from math import exp

hconc=0
centre = 0
width = 0.1
hlevel = 0.5

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

f = open('utterances.txt','r')
utterances = [x.strip() for x in f.read().split('%%')]

def say(text):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    if(not c.wait_for_server(timeout=rospy.Duration(10))):
        print "Failed to connect to server"
    goal = mary_tts.msg.maryttsGoal(text)
    c.send_goal(goal)
    if(not c.wait_for_result(timeout=rospy.Duration(10))):
        print "Failed to get result"
        
def utterance():
    global hlevel
    q = random.gauss(hlevel,0.1)
    q = min(max(q,0),1)
    q = int(q*len(utterances))
    t = utterances[q]
    say(t)
    return t
    
def speakserv(req):
    return std_srvs.srv.TriggerResponse(True,utterance());

def update():
    global hconc,hlevel,decayrate,releaserate,tempreleaserate
    r = releaserate+tempreleaserate*tempreleasefactor+smilecount*smilereleasefactor
    r = r*releasefactor*(0.95-hconc)
    tempreleaserate=0
    hconc = (hconc+r)*decayrate
    hlevel = sigmoid(hconc)

def startpublisherandwait():
    global hconc,hlevel
    pubc = rospy.Publisher('hconc',Float64,queue_size=100)
    publ = rospy.Publisher('hlevel',Float64,queue_size=100)
    rate = rospy.Rate(3) # 3Hz
    while not rospy.is_shutdown():
        update()
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
    rospy.Service('aes/speak',std_srvs.srv.Trigger,speakserv)

def start():
    update()
    rospy.init_node("aes",anonymous=True)
    startsubscribers()
    startservices()
    startpublisherandwait()
    


if __name__ == '__main__':
    start()
        
