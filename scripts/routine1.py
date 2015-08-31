#!/usr/bin/env python

import rospy
import random

from routine_behaviours.robot_routine import RobotRoutine

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from mongodb_store.message_store import MessageStoreProxy
msg_store = MessageStoreProxy()

f = open('jokes.txt','r')
jokes = [x.strip() for x in f.read().split('%%')]

def move_to(pose):
    task = Task()
    task.action = '/move_base'
    xin = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    x = xin - 1
    poseout = pose
    poseout.pose.position.x = x
    object_id = msg_store.insert(poseout)
    task_utils.add_object_id_argument(task, object_id, PoseStamped)
    return task
    

def tell_joke():
    task = Task()
    task.action = '/speak'

    joke = random.choice(jokes)
#    joke_text = "I will tell a joke." + joke + "Please smile or I will be sad."
    joke_text = joke
    task_utils.add_string_argument(task, joke_text)

    max_wait_secs = 60
    task.max_duration = rospy.Duration(max_wait_secs)

    task.start_after = rospy.get_rostime() + rospy.Duration(10)
    task.end_before = task.start_after + rospy.Duration(200)

    return task

class ExampleRoutine(RobotRoutine):
    """ Creates a routine which simply visits nodes. """

    def __init__(self, daily_start, daily_end, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        # super(PatrolRoutine, self).__init__(daily_start, daily_end)        
        RobotRoutine.__init__(self, daily_start, daily_end, idle_duration=idle_duration, charging_point=charging_point)

    def create_routine(self):
        pass

    def on_idle(self):
        """
            Called when the routine is idle. Default is to trigger travel to the charging. As idleness is determined by the current schedule, if this call doesn't utlimately cause a task schedule to be generated this will be called repeatedly.
        """
        pose = rospy.wait_for_message("/people_tracker/pose", geometry_msgs.msg.PoseStamped)

        if (pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z):
            rospy.loginfo('Can not see anyone')
        else:
            rospy.loginfo('Calculating move_base')
            move_task = move_to(pose)
            rospy.loginfo('Calculating tell_joke')
            joke_task = tell_joke()
            rospy.loginfo('Calculating scheulding tasks')
            self.add_tasks([move_task, joke_task])
        rospy.loginfo('I am idle')    



if __name__ == '__main__':

    rospy.init_node('routine1')

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,00, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(180)

    routine = ExampleRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    

    routine.create_routine()

    routine.start_routine()

    rospy.spin()
