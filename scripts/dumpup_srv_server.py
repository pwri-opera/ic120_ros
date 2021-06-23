#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nav_msgs import msg
import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ic120_nav.srv import dump_nav,dump_navResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


dumpup_pub = rospy.Publisher('/ic120/is_dumpup', Bool, queue_size=1)

dumping_time = 5

def server(req):    
    print("Get service call for dumpup")
    is_dump = Bool()
    is_dump.data=True
    for i in range(1,dumping_time):
        print("is dumping up")
        dumpup_pub.publish(is_dump)
        rospy.sleep(1.0)
    rospy.sleep(5.0)
    is_dump.data=False
    for i in range(1,dumping_time):
        print("is dumping down")
        dumpup_pub.publish(is_dump)
        rospy.sleep(1.0)
    response = dump_navResponse()
    response.is_ok.data = True
    print("Done")
    print("Waiting for next service call")
    return response

if __name__ == '__main__':
    rospy.init_node("dumpup_server")
    
    s = rospy.Service("dumpup_srv", dump_nav, server)
    print("Ready to Dump up service client.")
    rospy.spin()
