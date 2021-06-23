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
from ic120_nav.srv import dump_nav,dump_navRequest

# X,Y,Theta, pose_flag, wait_for_nav_service_call, wait_for_dumpup_service_call
waypoints = [
    [0,0,math.pi,True,False,False],#1
    [10,0,math.pi,True,False,False],#2
    [20,0,math.pi,False,True,False],#3
    [10,0,math.pi,False,True,False],#4
    [10,-10,-math.pi/2,False,False,True],#5
    [10,-15,math.pi,True,False,False],#6
    [20,-15,math.pi,True,False,True],#7
    [10,-15,math.pi,False,False,False],#8
    [10,-10,math.pi/2,False,False,False]#9
    #2に戻る
]

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def goal_pose(pose):
    print(pose)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation = euler_to_quaternion(Vector3(0.0,0.0,pose[2]))
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
            if(pose[4] == True):
                print("waiting for service call")
                rospy.wait_for_service('ic120_nav_srv')
                while True:
                    print("waiting for permission from const_manager")
                    srv_proxy  = rospy.ServiceProxy('ic120_nav_srv', dump_nav)
                    request = dump_navRequest()
                    response = srv_proxy (request)
                    rospy.sleep(1)                    
                    if(response.is_ok.data == True):
                        print("permitted")
                        break
            client.send_goal(goal)
            pose_flag = pose[3]
            dumup_flag = pose[5]
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("map", "/ic120/base_link", now, rospy.Duration(4.0))

                # map座標系の現在位置をtfから取得する
                position, quaternion = listener.lookupTransform("map", "/ic120/base_link", now)

                if(pose_flag == True):
                    # print("wait for service back from local/global planner")
                    if(client.wait_for_result(rospy.Duration(0.5)) == True):
                        if(dumup_flag == True):
                            print("waiting for dumpup_manager")
                            srv_proxy  = rospy.ServiceProxy('dumpup_svs', dump_nav)
                            request = dump_navRequest()
                            response = srv_proxy (request)
                            rospy.sleep(1)                    
                            if(response.is_ok.data == True):
                                print("permitted")
                                break
                        print("next waypoint")
                        break
                    print("moving")
                else:
                    # ウェイポイントのゴールの周囲１ｍ以内にロボットが来たら、次のウェイポイントを発行する
                    if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 1):
                        print("next waitpoint")
                        break
                    else:
                        print("moving!!")
                        rospy.sleep(0.5)
            