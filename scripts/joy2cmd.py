#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import rospy
import time
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectoryPoint

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


"""
jntState = JointState()
trajectory = JointTrajectoryPoint()


def cb_jntState(msg):
    jntState = msg
"""


## Class
## End class

joyState = Joy()

def cb_joy(msg):
    global joyState
    joyState = msg

if __name__ == '__main__':
    global joyState#, jntState, trajectory

    rospy.init_node('joy2cmd', anonymous=True)

    sub_joy = rospy.Subscriber("/joy", Joy, cb_joy)
    #sub_jointState = rospy.Subscriber("/joint_states", JointState, cb_jntState)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator_i5"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    print('start')

    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        #print(joyState)
        if len(joyState.axes) is 8:
            if int(joyState.axes[7]) == -1:
                print("aubo_robot.go_to_joint_state()")
                #group.go([0,0,0,0,0,0], wait=True)
                currentState = robot.get_current_state()
                position = currentState.joint_state.position
                group.go([0,0,0,0,0,position[5]+float(0.5)], wait=True)
                group.stop()
                current_joints = group.get_current_joint_values()

        rate.sleep()

    sub_joy.unregister()
    pub_cmd.unregister()
