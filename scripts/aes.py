#!/usr/bin/python

import sys
import rospy
import actionlib
import random
import std_srvs.srv
import mary_tts.msg
from subprocess import Popen
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from strands_executive_msgs.srv import AddTasks, DemandTask, SetExecutionStatus
import time
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import String
from bayes_people_tracker.msg import PeopleTracker
from math import exp

nextannounce = time.time() + 20

state = 'NORMAL'


hconc=0
centre = 0
width = 0.1
hlevel = 0.5

# geometric decay rate
decayrate=rospy.get_param('/aes/decayrate',0.99)
# release multiplied by this
releasefactor=rospy.get_param('/aes/release',0.02)
# amount released by nice/nasty services
tempreleasefactor=rospy.get_param('/aes/nudge',2)
# amount released by smiles
smilereleasefactor=rospy.get_param('/aes/smile',0.4)
# amount released by people
peoplereleasefactor=rospy.get_param('/aes/people',0.1)
# amount released by home node
homereleasefactor=rospy.get_param('/aes/home',0.2)


releaserate=0
smilecount=0
tempreleaserate=0
peoplerelease=0
homerelease=0
goingHome = False
publog=0

def log(x):
    rospy.logwarn(x)
        
def snd(x):
    Popen(['aplay',x])
    

def sigmoid(x):
    x=(x-centre)/width
    # input centred at 0
    return 1 - (1/(1+exp(x)))

# print the sigmoid

for x in [x/10.0 for x in range(-10,10)]:
    print str(x)+" "+str(sigmoid(x))

f = open('utterances.txt','r')
utterances = [x.strip() for x in f.read().split('%%')]

def calltrig(name):
    rospy.wait_for_service(name)
    try:
        s = rospy.ServiceProxy(name,std_srvs.srv.Trigger)
        resp = s()
    except rospy.ServiceException,e:
        print "Failed: %s" % e

def say(text):
    c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
    if(not c.wait_for_server(timeout=rospy.Duration(10))):
        print "Failed to connect to server"
    goal = mary_tts.msg.maryttsGoal(text)
    c.send_goal(goal)
    if(not c.wait_for_result(timeout=rospy.Duration(10))):
        print "Failed to get result"
        
def utterance():
    global hlevel,state
    if hlevel>0.6 and random.random()<(hlevel*0.3):
        calltrig('/qtgui/dispatcher/joke')
        t='joke'
    else:      
        q = random.gauss(hlevel,0.1)
        q = min(max(q,0),1)
        q = int(q*(len(utterances)-1))
        t = utterances[q]
        say(t)
    return t
    
def speakserv(req):
    return std_srvs.srv.TriggerResponse(True,utterance());

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

def gowp(name):
    task = Task()
    task.action = '/wait_action'
    max_wait_secs = 20
    task.max_duration = rospy.Duration(max_wait_secs)
    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(10))
    task.start_after = rospy.get_rostime() + rospy.Duration(10)
    task.end_before = task.start_after + rospy.Duration(400)
    task.start_node_id = name
    task.end_node_id = task.start_node_id
    set_execution_status = get_execution_status_service()
    set_execution_status(True)
    demand_task = get_demand_task_service()
    demand_task(task) 
    log('Heading for %s - too miserable' % name)

def update():
    global hconc,hlevel,decayrate,releaserate,tempreleaserate,peoplerelease
    global peoplereleasefactor,smilereleasefactor,tempreleasefactor
    global smilecount,nextannounce
    global homerelease,homereleasefactor
    global state,publog
    
    r = releaserate
    r = r+tempreleaserate*tempreleasefactor
    r = r+smilecount*smilereleasefactor
    r = r+peoplerelease*peoplereleasefactor
    r = r+homerelease*homereleasefactor

    deb = 'TMP:{0} SM:{1} HOME:{2} PPL:{3} --> {4} ({5})'.format(
        tempreleaserate,smilecount,homerelease,peoplerelease,r,state)
    
    if publog!=0:
        publog.publish(deb)
    log(deb)

    r = r*releasefactor*(0.95-hconc)
    tempreleaserate=0
    hconc = (hconc+r)*decayrate
    hlevel = sigmoid(hconc)
    if hlevel<0.1 and state=='NORMAL':
        say("I have had enough. I am going home.")
        gowp('Station')
        state='HOMING'
    if time.time() > nextannounce:
        nextannounce = nextannounce+random.uniform(20,120)
        utterance()
    if hlevel>0.8:
        state='HAPPY'
    elif hlevel>0.3:
        state='NORMAL'        
    if state=='HAPPY' and homerelease>0.5:
        gowp('WayPoint1')
        

def startpublisherandwait():
    global hconc,hlevel,publog
    publog = rospy.Publisher('aeslog',String,queue_size=100)
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

# fuzzification
def ramp(x,a,b):
    if x<a:
        return 0
    elif x>b:
        return 1
    else:
        return (x-a)/(b-a)
def peak(x,v,w):
    return min(ramp(x,v-w,v),1.0-ramp(x,v,v+w))

def setpeopleOLD(p):
    global peoplerelease
    t=0
    good=0
    bad=0
    closeppl=0
    # fuzzy logic on each person
    for d,a in zip(p.distances,p.angles):
        # close is up to 4m away
        close = 1.0-ramp(d,1.0,4.0)
        closeppl = closeppl+close
        # very close
        veryclose = 1.0-ramp(d,0.0,2.0)
        # behind
        behind = peak(a,3.1415927,0.5)
        # "good" is close and not very close and not behind
        good = max(good,min(close,1.0-veryclose,1.0-behind))
        # bad is (close and behind) or veryclose
        bad = max(bad,min(close,behind),veryclose)
    # we end up with "good/bad", but lots of close people is very bad.
    bad = max(bad,ramp(closeppl,2,4))
    peoplerelease = good-bad
#    sys.stderr.write(str(peoplerelease)+"\n") 
        

def setpeople(p):
    global peoplerelease
    nearct=0
    ct=0
    for d,a in zip(p.distances,p.angles):
        if d<4:
            ct=ct+1
        if d<2:
            nearct=nearct+1
    log('near ppl:{0}, ppl:{1}'.format(nearct,ct))
    
    peoplerelease=0

    # lots of people or none make me unhappy
    if ct>3 or ct<1:
        peoplerelease=peoplerelease-1

    # less than 3 close people make me happier
    if nearct<3 and nearct>1:
        peoplerelease=peoplerelease+1


def setnode(x):
    global homerelease,goingHome,hlevel
    if x=='Station' or x=='ChargingPoint':
        homerelease = 1
    else:
        homerelease = 0
    log('hr {0}'.format(homerelease))
    

def startsubscribers():
    rospy.Subscriber('smile_detector',Int8,
        lambda x: setsmile(x.data))
    rospy.Subscriber('hrelease',Float64,
        lambda x: setrelease(x.data))
    rospy.Subscriber('/people_tracker/positions',PeopleTracker,
        lambda x: setpeople(x))
    rospy.Subscriber('/current_node',String,
        lambda x: setnode(x.data))
    
def nice(req):
    snd('purr.wav')
    return bumprelease(0.5)
def nasty(req):
    snd('slap.wav')
    return bumprelease(-0.5)
    

        
def startservices():
    rospy.Service('aes/nice',std_srvs.srv.Trigger,nice)
    rospy.Service('aes/nasty',std_srvs.srv.Trigger,nasty)
    rospy.Service('aes/speak',std_srvs.srv.Trigger,speakserv)
    
def start():
    update()
    rospy.init_node("aes",anonymous=True)
    startsubscribers()
    startservices()
    startpublisherandwait()
    


if __name__ == '__main__':
    start()
        
