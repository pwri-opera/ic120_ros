#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


waypoints = [
    [(20,0.0,0.0),(0.0,0.0,0.0,1.0)],
    [(40,0.0,0.0),(0.0,0.0,0.0,1.0)]
]


def goal_pose(pose): 
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    print("start")

    client = actionlib.SimpleActionClient('/ic120/move_base', MoveBaseAction) 
    # while not client.wait_for_server(rospy.Duration(5)):
    #     rospy.loginfo("Waiting for the move_base action server to come up")
    client.wait_for_server()
    listener.waitForTransform("map", "/ic120/base_link", rospy.Time(), rospy.Duration(4.0))

    print("start while loop")

    while True:
        for pose in waypoints: 
            goal = goal_pose(pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("map", "/ic120/base_link", now, rospy.Duration(4.0))

                # map座標系の現在位置をtfから取得する
                position, quaternion = listener.lookupTransform("map", "/ic120/base_link", now)

                # ウェイポイントのゴールの周囲１ｍ以内にロボットが来たら、次のウェイポイントを発行する
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 1):
                    print("next!!")
                    break

                else:
                    print("moving!!")
                    rospy.sleep(0.5)