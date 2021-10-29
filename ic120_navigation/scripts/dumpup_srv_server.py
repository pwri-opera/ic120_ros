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
from ic120_navigation.srv import dump_nav,dump_navResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

dumpup_pub = rospy.Publisher('/ic120/cmd_dump', Bool, queue_size=1)
dump_direction_pub = rospy.Publisher('/ic120/cmd_dump_direction', Bool, queue_size=1)
left_track_pid_pause_pub = rospy.Publisher('/ic120/left_track/pid_enable', Bool, queue_size=5)
right_track_pid_pause_pub = rospy.Publisher('/ic120/right_track/pid_enable', Bool, queue_size=5)

dumping_time = 20*10

vessel_angle=0 
def js_callback(data):
    if data.name[0] == "vessel_pin_joint":
        global vessel_angle
        vessel_angle = data.position[0]
        # print("vessel_angle:", vessel_angle*180/3.1415,"deg")

def server(req):    
    print("Get service call for dumpup")
    pid_enable = Bool()
    pid_enable.data = False

    ### dumpup
    is_dump_direction = Bool()
    is_dump_direction.data=True
    is_dump = Bool()
    is_dump.data=True

    while vessel_angle >= -65.0/180*math.pi:
        dump_direction_pub.publish(is_dump)
        dumpup_pub.publish(is_dump_direction)
        left_track_pid_pause_pub.publish(pid_enable)
        right_track_pid_pause_pub.publish(pid_enable)
        rospy.sleep(0.5)

    rospy.sleep(2.0)
    ### dump down
    is_dump.data=False

    while vessel_angle <= 0.0:
        dump_direction_pub.publish(is_dump)
        dumpup_pub.publish(is_dump_direction)
        left_track_pid_pause_pub.publish(pid_enable)
        right_track_pid_pause_pub.publish(pid_enable)
        rospy.sleep(0.5)
    rospy.sleep(2.0)
    response = dump_navResponse()
    response.is_ok.data = True
    print("Done")
    print("Waiting for next service call")
    pid_enable.data = True
    left_track_pid_pause_pub.publish(pid_enable)
    right_track_pid_pause_pub.publish(pid_enable)
    return response

if __name__ == '__main__':
    rospy.init_node("dumpup_server")
    
    rospy.Subscriber("/ic120/joint_states", JointState, js_callback,queue_size=10)

    s = rospy.Service("dumpup_srv", dump_nav, server)
    print("Ready to Dump up service client.")
    rospy.spin()
