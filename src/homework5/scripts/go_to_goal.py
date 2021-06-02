#!/usr/bin/python3

import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospy

def createMarker(x ,y ,z, w):
    pass

def goToGoal(x ,y ,z, w):
    action_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    action_client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.header.stamp = rospy.Time.now()
    action_client.send_goal(goal)

rospy.init_node("send_goal")
if len(sys.argv) > 5:
    x,y,z,w = float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5])
    if sys.argv[1] == "m":
        createMarker(x,y,z,w)
    elif sys.argv[1] == "g":
        goToGoal(x,y,z,w)
elif len(sys.argv) > 1:
    if sys.argv[1] == "r":
        goToGoal(0,0,-0.70710678118,0.70710678118)
    elif sys.argv[1] == "g":
        coords = input()
        coords = coords.split()
        x,y,z,w = float(coords[0]),float(coords[1]),float(coords[2]),float(coords[3])
        goToGoal(x,y,z,w)
else:
    print("Premalo arugmentov")
