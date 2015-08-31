import sys
import rospy
import std_srvs.srv

def calltrig(name):
    rospy.wait_for_service(name)
    try:
        s = rospy.ServiceProxy(name,std_srvs.srv.Trigger)
        resp = s()
    except rospy.ServiceException,e:
        print "Failed: %s" % e
    
if __name__ == "__main__":
    rospy.init_node("testserv")
    calltrig('aes/nice')
