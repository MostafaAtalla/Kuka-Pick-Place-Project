#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Updated by: Mostafa Atalla April,12,2020: FK and IK solution added and tested 

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from numpy import array, dot





def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialization
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
        
        num_of_transforms = len(DH['a'])    # Number of transforms from the base frame to the DH gripper frame
        
        transforms = []    #transforms list to accumulate the transforms from each joint to the following one.
    
        # Create the DH frames transformations iteratively from joint 1 to joint 6
        for i in range(num_of_transforms):
            # Extracting the Dh paramters of the corresponding frame from the Dh dictionary
            alpha = DH['alpha'][i]     #link twist
            a = DH['a'][i]             #link length
            d = DH['d'][i]             #joint offset
            q = DH['q'][i]             #joint angle

            # Create the frame transformation matrix relative to the previous frame and append it the transforms list
            transforms.append(Matrix([[cos(q), -sin(q), 0, a],
                                     [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                                     [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                                     [0, 0, 0, 1]]))

        transforms.append(Tdh_urdf)

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
            
            T_gripper_rotation = tf.transformations.euler_matrix(roll,pitch,yaw)
            # Assemble a translation matrix corresponding to position values px,py,pz
            T_gripper_translation = tf.transformations.translation_matrix(array([px,py,pz]))
            # Assemble the homogenous transformation matrix corresponding to the pose
            T_gripper = Matrix(dot(T_gripper_translation,T_gripper_rotation))
            # Slicing the Rotation matrix out of the total transformation from base to DH gripper frame
            R_gripper = T_gripper[:3,:3]
            # Account for the difference between the DH and URDF frames
            R_gripper_corrected = R_gripper*transforms[-1][:3,:3]
            
            
            
            ############### Calculating Inverse Position Solution (theta1,theta2,theta3) ########################
            # Calculate the wrist center position vector
            p_wc = T_gripper[:3,3] - 0.303*R_gripper_corrected[:3,2]  # wrist center position vector
            #print(p_wc)
            # Calculate theta1
            theta1 = atan2(p_wc[1],p_wc[0])

            # Caculate theta2
            # Calculate the triangle sides asscoicated with theta2 calculations
            a = 1.501 
            b = ((((p_wc[0]**2+p_wc[1]**2)**0.5) - 0.35)**2 + (p_wc[2]-0.75)**2)**0.5
            c = 1.25
            # Calculate the triangle angles needed for calculating theta2 and theta3
            angle_a = acos((b**2+c**2-a**2)/(2*b*c))
            angle_b = acos((a**2+c**2-b**2)/(2*a*c))
            # Calculate theta2 value
            theta2 = pi/2 - angle_a - atan2(p_wc[2]-0.75,((p_wc[0]**2+p_wc[1]**2)**0.5)-0.35) 

            # Calculate theta3
            theta3 = pi/2 - (angle_b + 0.036)
            
            ############### Calculate the inverse Orientation Solution (theta4,theta5,theta6) #####################
            # Calculating R0_3: Transformation from base to joint 3
            R0_3 = transforms[0][:3,:3]*transforms[1][:3,:3]*transforms[2][:3,:3]
            # Evaluating the numeric R0_3 given the inverse position solution obtained above
            R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            # Caculating R3_gripper: Transformation from joint 3 to gripper frame 
            R3_gripper = R0_3.transpose()*R_gripper_corrected
           
            # Calculating theta4, theta5 theta6
            theta4 = atan2(R3_gripper[2,2], -R3_gripper[0,2])
            theta5 = atan2(sqrt(R3_gripper[0,2]*R3_gripper[0,2] + R3_gripper[2,2]*R3_gripper[2,2]), R3_gripper[1,2])
            theta6 = atan2(-R3_gripper[1,1], R3_gripper[1,0])
           
            

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node
    rospy.init_node('IK_server')
    # declare IK service
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    # Call IK_server function
    IK_server()
