'''
This function calculates the inverse kinematics solution for the kuka K210
It takes the pose parameters as an input, returns the corresponding joint values as an output
'''
import tf
from sympy import *

def inverse_kinematics(px,py,pz,roll,pitch,yaw):
    # Assemble a Rotation matrix corresponding to those euler angles   
    T_gripper = tf.transformations.euler_matrix(roll,pitch,yaw)
    # Cast the rotation matrix to be sympy rotation matrix
    T_gripper = Matrix(T_gripper)
    # Assemble the homogenous transformation matrix corresponding to the pose
    T_gripper[:3,3] = Matrix([px,py,pz])

    R_gripper = T_gripper[:3,:3]

    R_gripper_corrected = R_gripper * (Tdh_urdf[:3,:3].transpose()) 

    ### Your IK code here
    # Calculate the wrist center position vector
    directional_vector = Matrix([1,0,0])   # directional vector from gripper frame origin to the wrist center      

    p_wc = T_gripper[:3,3] - 0.303*directional_vector  # wrist center position vector

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
    theta3 = pi/2 - (angle_b+0.036)

    # Calculate the inverse Orientation Solution
    # Calculating R0_3
    R0_3 = HT[0][:3,:3]*HT[1][:3,:3]*HT[2][:3,:3]
    R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
    # Caculating R3_6
    R3_6 = R0_3.transpose()*R_gripper_corrected

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    return (theta1,theta2,theta3,theta4,theta5,theta6)