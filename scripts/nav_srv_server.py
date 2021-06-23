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

def server(req):
    print("Get service call for dumpup")
    response = dump_navResponse()
    response.is_ok.data = True
    return response

if __name__ == '__main__':
    rospy.init_node("ic120_nav_server")
    
    s = rospy.Service("ic120_nav_srv", dump_nav, server)
    print("Ready to navigation service client")
    rospy.spin()
