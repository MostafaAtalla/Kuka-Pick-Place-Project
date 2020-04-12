'''
This function takes the following inputs:
DH table dictionary containing the following key-values pairs:
Key: 'a'     (link length)   - Value: list of link lengths
Key: 'alpha' (link twist)    - Value: list of link twist angles in radians
Key: 'd'     (joint offset)  - Value: list of joint offsets 
Key: 'q'     (joint angles)  - Value: list of joint angles 


It produces the following outputs:
List of Sympy transformation matrices from base to gripper frame each w.r.t. the prior frame

Dependacies: Sympy
'''
from sympy import *
import tf
import numpy as np

def forward_kinematics(DH):
    # Initialization
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

   
    return transforms
   

############################### Debugging Section ###############################################
'''
# Remove the comment quotation marks from here and at the end of this section to start debugging

######## Calculate Pose Using Function Herein ############
# Create Symbolic Variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')       
# Create Modified DH parameters dictionary
DH = {'alpha': [0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0],
              'a': [0, 0.35, 1.25, -0.054, 0, 0, 0],
              'd': [0.75, 0, 0, 1.5, 0, 0, 0.303],
              'q': [q1, q2 - pi / 2, q3, q4, q5, q6,0]}
# Define the correction transformation between the gripper DH frame and gripper URDF frame
# Create the gripper frame correction matrix to align the DH frames with the URDF Frames
# The rotation matrix of this transformations is the unit vectors of the URDF gripper frame
# With respect to the DH gripper frame. The translation component is zero
Tdh_urdf = Matrix([[0,  0,   1,  0],
                       [0, -1,   0,  0],
                       [1,  0,   0,  0],
                       [0,  0,   0,  1]])

# Call forward kinematics function
transforms = forward_kinematics(DH)
# Append the transformation matrix between the DH gripper frame and the URDF gripper frame
transforms.append(Tdh_urdf)
    
# Create the total transforms from each frame to the base frame
T0_gripper = transforms[0]   # Transformation matrix from the URDF gripper frame to the base frame
    
for i in range(1,len(transforms)):
    T0_gripper = T0_gripper*transforms[i]     # Accumulating the transformations

# Calcuate the pose 4*4 matrix of the robot calculated through the developed forward kinematics function
# Joint values dictionary
# Enter Joint Values set in Joint State Publisher

values={}
values[q1] = -2.91
values[q2] = -0.78
values[q3] =  0.25
values[q4] =  5.19
values[q5] =  0.36
values[q6] =  3.57

T0_gripper = T0_gripper.evalf(subs=values)

######## Calculate Pose Matrix Using RVIZ ACTUAL VALUES ############
# Create translation vector
# Enter translation vector valuesof gripper_link from RVIZ
translation = np.array([[-1.0606, -0.15275, 2.4486]])

# Create orienatation quaternion
# Enter orientation quaternion values of gripper_link from RVIZ
quaternion = [0.094633, 0.94858, 0.264, -0.14678]

# Create Translation Matrix
T0_gripper_actual_translation = tf.transformations.translation_matrix(translation)
# Create Rotation Matrix
T0_gripper_actual_rotation = tf.transformations.quaternion_matrix(quaternion)
# Calculate Total Transformation and Cast it into Sympy Matrix
T0_gripper_actual = Matrix(np.dot(T0_gripper_actual_translation,T0_gripper_actual_rotation))


######## Calculating and Printing Pose Error ############
pprint(T0_gripper_actual-T0_gripper)

# Remove Quotation Marks Below and at the Beginning of This Section to Start Debugging
'''





