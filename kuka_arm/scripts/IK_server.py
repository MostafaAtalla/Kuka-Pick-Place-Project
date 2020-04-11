#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
#from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from FK import *


def initialization():
    # Initialize the symbolic variables
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')       
    # Create Modified DH parameters dictionary
    DH = {'alpha': [0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0],
              'a': [0, 0.35, 1.25, -0.054, 0, 0, 0],
              'd': [0.75, 0, 0, 1.5, 0, 0, 0.303],
              'q': [q1, q2 - pi / 2, q3, q4, q5, q6,0]}
    # Define the correction transformation between the gripper DH frame and gripper URDF frame
    Tdh_urdf = Matrix([[0,  0,   1,  0],
                       [0, -1,   0,  0],
                       [1,  0,   0,  0],
                       [0,  0,   0,  1]])

    # Call forward kinematics function from the FK module imported above
    T0_gripper,T0_frame,transforms = forward_kinematics(DH,Tdh_urdf)
    return (T0_gripper,T0_frame,transforms)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Calculate the wrist center position vector
	    directional_vector = Matrix([1,0,0])   # directional vector from gripper frame origin to the wrist center

        p_gripper = Matrix([px,py,pz])         # gripper frame origin position vector

        p_wc = p_gripper - 0.303*directional_vector  # wrist center position vector
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node
    rospy.init_node('IK_server')
    # calculate the forward kinematics
    T0_gripper,T0_frame,transforms = initialization()
    # declare IK service
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
