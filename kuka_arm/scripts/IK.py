'''
This function is to test the inverse kinematics solution for the kuka K210
It takes the pose parameters as an input, returns the corresponding joint values as an output

The debug function at the end of this script is meant to test the function by setting the inputs
and call the inverse kinematics function. For more information please refer to the debug function
below. 
'''
import rospy
import tf

import numpy as np
from mpmath import *
from sympy import *
from FK import forward_kinematics


def inverse_kinematics(px,py,pz,roll,pitch,yaw,transforms):
       
    # Initialize the symbolic variables
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 
    # Assemble a rotation matrix corresponding to those euler angles   
    T_gripper_rotation = tf.transformations.euler_matrix(roll,pitch,yaw)
    # Assemble a translation matrix corresponding to position values px,py,pz
    T_gripper_translation = tf.transformations.translation_matrix(np.array([px,py,pz]))
    # Assemble the homogenous transformation matrix corresponding to the pose
    T_gripper = Matrix(np.dot(T_gripper_translation,T_gripper_rotation))
    # Slicing the rotation matrix of the total transformation matrix
    R_gripper = T_gripper[:3,:3]
    # Account for the difference between the gripper DH frame and the gripper URDF frame
    R_gripper_corrected = R_gripper*transforms[-1][:3,:3]
    
     
    
  
    # Calculate the wrist center position vector
    p_wc = T_gripper[:3,3] - 0.303*R_gripper_corrected[:3,2]  # wrist center position vector
    
    # Calculate joint angles using Geometric IK method
    # Calculate the inverse position solution
    # calculate first joint angle
    theta1 = atan2(p_wc[1],p_wc[0])

    # calculate second joint angle
    a = 1.501
    b = ((((p_wc[0]**2+p_wc[1]**2)**0.5) - 0.35)**2 + (p_wc[2]-0.75)**2)**0.5
    c = 1.25
    
    angle_a = acos((b**2+c**2-a**2)/(2*b*c))

    angle_b = acos((a**2+c**2-b**2)/(2*a*c))

    theta2 = pi/2 - angle_a - atan2(p_wc[2]-0.75,((p_wc[0]**2+p_wc[1]**2)**0.5)-0.35) 
    # calculate third joint angle
    theta3 = pi/2 - (angle_b + 0.036)
 
    # Calculate the inverse Orientation Solution
    # Calculating R0_3
    R0_3 = transforms[0][:3,:3]*transforms[1][:3,:3]*transforms[2][:3,:3]
    R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})


    # Caculating R3_6
    R3_6 = R0_3.transpose()*R_gripper_corrected
    

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
   
    return (theta1,theta2,theta3,theta4,theta5,theta6)


def debug(quat,p):

    # Call forward kinematics function
    transforms = forward_kinematics() 

    # Calculate roll,pitch,yaw from quaternion
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quat)
    
    # Call inverse kinematics function
    theta = inverse_kinematics(p[0],p[1],p[2],roll,pitch,yaw,transforms)
    
    print('theta1 = ',float(theta[0]))
    print('theta2 = ',float(theta[1]))
    print('theta3 = ',float(theta[2]))
    print('theta4 = ',float(theta[3]))
    print('theta5 = ',float(theta[4]))
    print('theta6 = ',float(theta[5]))

'''
# Remove the quotation marks above and below to start debugging
# Once the parameters are set run the script.
# The inputs are the pose postion vector and orientation quaternion

# Initialize the symbolic variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 

# Create quaternion
quat = [0.7339, -0.47561, -0.30374, 0.37806]



# Enter px,py,pz values
p = [1.155, -1.1857, 2.7356]

debug (quat,p)
'''