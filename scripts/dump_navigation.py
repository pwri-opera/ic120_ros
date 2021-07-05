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
from ic120_nav.srv import dump_nav,dump_navRequest,dump_navResponse

def goal_pose(pose):
    print(pose)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose = pose
    return goal_pose

#service
rospy.init_node("ic120_navigation")
client = actionlib.SimpleActionClient('/ic120/move_base', MoveBaseAction) 
listener = tf.TransformListener()

def server(req):
    print("Get service call for navigation")
    goal = goal_pose(req.target_pose)
    orientation_flag = req.orientation_flag.data
    dump_flag = req.dump_flag.data

    client.send_goal(goal)

    while True:
        now = rospy.Time.now()
        listener.waitForTransform("map", "/ic120_tf/base_link", now, rospy.Duration(1.0))
        position, quaternion = listener.lookupTransform("map", "/ic120_tf/base_link", now)

        if(orientation_flag == True):
            print("orientation_flat==True")
            if(client.wait_for_result(rospy.Duration(0.5)) == True):
                if(dump_flag == True):
                    print("waiting for dumpup_manager")
                    rospy.wait_for_service('dumpup_srv') 
                    dumpup_srv_proxy  = rospy.ServiceProxy('dumpup_srv', dump_nav)
                    request = dump_navRequest()
                    response = dumpup_srv_proxy (request)
                    rospy.sleep(1)                    
                    if(response.is_ok.data == True):
                        print("permitted")
                        break
                print("next waypoint")
                break
            print("moving!")
        else:
            print("orientation_flag==False")
            # ウェイポイントのゴールの周囲１ｍ以内にロボットが来たら、次のウェイポイントを発行する
            if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 1):
                print("next waitpoint")
                break
            else:
                print("moving!!!")
                rospy.sleep(0.5)
    response = dump_navResponse()
    response.is_ok.data = True
    return response


if __name__ == '__main__':
    #service
    # rospy.init_node("ic120_navigation")
    # global client
    # client = actionlib.SimpleActionClient('/ic120/move_base', MoveBaseAction) 
    # listener = tf.TransformListener()
    client.wait_for_server()
    listener.waitForTransform("map", "/ic120_tf/base_link", rospy.Time(), rospy.Duration(4.0))

    s = rospy.Service("ic120_nav_srv", dump_nav, server)
    print("Ready to navigation")
    print("waiting for waypoint from service server")
    rospy.spin()