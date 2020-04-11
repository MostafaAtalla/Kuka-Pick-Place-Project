#!/usr/bin/env python
from sympy import *

# Initialization
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')        #Initialize the symbolic variables 

# Create Modified DH parameters dictionary
DH = {'alpha': [0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0],
      'a': [0, 0.35, 1.25, -0.054, 0, 0, 0],
      'd': [0.75, 0, 0, 1.5, 0, 0, 0.303],
      'q': [q1, q2 - pi / 2, q3, q4, q5, q6,0]}

num_of_transforms = len(DH['a'])    # Number of transforms from the base frame to the DH gripper frame
transforms = []    #transforms list to accumulate the transforms from each joint to the following one.
# Create the DH frames transformations iteratively from joint 1 to joint 6
for i in range(num_of_transforms):
    # Extracting the Dh paramters of the corresponding frame from the Dh dictionary
    alpha = DH['alpha'][i]     #link twist
    a = DH['a'][i]             #link length
    d = DH['d'][i]             #joint  
    q = DH['q'][i]             #joint angle

    # Create the frame transformation matrix relative to the previous frame and append it the transforms list
    transforms.append(Matrix([[cos(q), -sin(q), 0, a],
                              [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                              [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                              [0, 0, 0, 1]]))

# Create the gripper frame correction matrix to align the DH frames with the URDF Frames
# The rotation matrix of this transformations is the unit vectors of the URDF gripper frame
# With respect to the DH gripper frame. The translation component is zero
transforms.append(Matrix([[0,  0,   1,  0],
                         [0, -1,   0,  0],
                         [1,  0,   0,  0],
                         [0,  0,   0,  1]]))

# Create the total transforms from each frame to the base frame
T0_frame= [transforms[0]]    # Transformations list from each frame to the base sequentially

T0_gripper = transforms[0]   # Transformation matrix from the URDF gripper frame to the base frame

for i in range(1,len(transforms)):
    T0_gripper = T0_gripper*transforms[i]     # Accumulating the transformations
    T0_frame.append(T0_gripper)               # Appending the transformations from each frame to the base


compute_T0_gripper= T0_gripper.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0})

print(compute_T0_gripper)


