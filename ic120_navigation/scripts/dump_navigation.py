#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nav_msgs import msg
import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Vector3, Pose
from ic120_nav.srv import dump_nav,dump_navRequest,dump_navResponse

world2map_trans=[0,0,0]
world2map_rot=[0,0,0,0]

def goal_pose(pose):
    print(pose)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x = pose.pose.position.x-world2map_trans[0]
    goal_pose.target_pose.pose.position.y = pose.pose.position.y-world2map_trans[1]
    goal_pose.target_pose.pose.position.z = pose.pose.position.z-world2map_trans[2]
    goal_pose.target_pose.pose.orientation = pose.pose.orientation
    return goal_pose

if __name__ == '__main__':
    print("Start nav")
    rospy.init_node("ic120_navigation")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    listener = tf.TransformListener()

    listener.waitForTransform("world", "map", rospy.Time(0), rospy.Duration(4.0))
    world2map_trans,world2map_rot = listener.lookupTransform("world", "map", rospy.Time(0))

    print("wait actionclient")

    client.wait_for_server()

    print("wait service")

    rospy.wait_for_service('nav_srv')

    print("Start loop")

    while True:
        nav_srv_proxy  = rospy.ServiceProxy('nav_srv', dump_nav)
        request = dump_navRequest()
        response = nav_srv_proxy (request)
        if(response.is_ok.data == False):
            rospy.sleep(1)
            continue
        goal = goal_pose(response.target_pose)
        client.send_goal(goal)
        while True:
            now = rospy.Time.now()
            listener.waitForTransform("map", "/ic120_tf/base_link", now, rospy.Duration(4.0))
            position, quaternion = listener.lookupTransform("map", "/ic120_tf/base_link", now)

            if(response.orientation_flag.data == True):
                print("orientation_flat==True")
                if(client.wait_for_result(rospy.Duration(0.5)) == True):
                    if(response.dump_flag.data == True):
                        rospy.sleep(1.0)
                        print("waiting for dumpup_manager")
                        rospy.wait_for_service('dumpup_srv') 
                        dumpup_srv_proxy  = rospy.ServiceProxy('dumpup_srv', dump_nav)
                        request = dump_navRequest()
                        response = dumpup_srv_proxy (request)
                        if(response.is_ok.data == True):
                            print("finished")
                            break
                    print("next waypoint")
                    break
                print("moving!")
            else:
                print("orientation_flag==False")
                # ウェイポイントのゴールの周囲１ｍ以内にロボットが来たら、次のウェイポイントを発行する
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                    print("next waitpoint")
                    break
                else:
                    print("moving!!!")
                    rospy.sleep(0.5)
        # rospy.spin()