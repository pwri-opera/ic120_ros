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
from ic120_nav.srv import dump_nav,dump_navResponse,dump_navRequest
from geometry_msgs.msg import PoseStamped


# X,Y,Theta, orientation_flag, dumpup_flag
waypoints = [
    [0,0,math.pi,True,False],#1
    [10,0,math.pi,True,False],#2
    [20,0,math.pi,False,False],#3
    [10,0,math.pi,False,False],#4
    [10,-10,-math.pi/2,False,False],#5
    [10,-15,math.pi,True,False],#6
    [20,-15,math.pi,True,True],#7
    [10,-15,math.pi,False,False],#8
    [10,-10,math.pi/2,False,False]#9
    #2に戻る
]

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


waypoint_num=0

def server(req):
    print("Get service call for navigation")
    global waypoint_num

    print("waypoint:", waypoint_num)

    waypoint = waypoints[waypoint_num]
    waypoint_num+=1
    if(waypoint_num>=len(waypoints)):
        waypoint_num=1
    response = dump_navResponse()
    response.is_ok.data = True
    response.target_pose.header.frame_id="map"
    response.target_pose.pose.position.x=waypoint[0]
    response.target_pose.pose.position.y=waypoint[1]
    response.target_pose.pose.orientation=euler_to_quaternion(Vector3(0,0,waypoint[2]))    
    response.orientation_flag.data=waypoint[3]
    response.dump_flag.data=waypoint[4]
    return response

if __name__ == '__main__':
    rospy.init_node("fake_const_manager")

    s = rospy.Service("ic120_nav_srv", dump_nav, server)
    print("Ready to navigation service client")
    rospy.spin()