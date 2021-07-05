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
    # [0,0,math.pi,True,False],#1
    [0,0,math.pi,True,True],#1
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


def waypoint_to_dump_navRequest_msg(waypoint):
    request = dump_navRequest()
    request.target_pose.header.frame_id = "map"
    request.target_pose.pose.position.x=waypoint[0]
    request.target_pose.pose.position.y=waypoint[1]
    request.target_pose.pose.orientation=euler_to_quaternion(Vector3(0,0,waypoint[2]))    
    request.orientation_flag.data=waypoint[3]
    request.dump_flag.data=waypoint[4]
    return request

if __name__ == '__main__':
    #client
    rospy.init_node("fake_const_manager")
    # print("waiting for service call")
    rospy.wait_for_service('ic120_nav_srv')

    for waypoint in waypoints:
        nav_srv_proxy  = rospy.ServiceProxy("ic120_nav_srv", dump_nav)
        print("sending next waypoint")

        request = waypoint_to_dump_navRequest_msg(waypoint)
        response = nav_srv_proxy (request)
        if(response.is_ok==True):
            print("waypoint reached")
        else:
            print("failed to reach waypoint")
        # rospy.sleep(1) 

        # s = rospy.Service("ic120_nav_srv", dump_nav, server)
        # print("Ready to navigation service client")
        # rospy.spin()
