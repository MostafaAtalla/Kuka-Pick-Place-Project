'''
This function calculates the forward kinematics symbolic solution for the
kuka k210. It returns a list of symbolic transformations from each frame 
to the subsquent one.

Dh Parameters Table is already defined within the function
DH table dictionary containing the following key-values pairs:
Key: 'a'     (link length)   - Value: list of link lengths
Key: 'alpha' (link twist)    - Value: list of link twist angles in radians
Key: 'd'     (joint offset)  - Value: list of joint offsets 
Key: 'q'     (joint angles)  - Value: list of joint angles 


It produces the following outputs:
List of Sympy transformation matrices from base to gripper frame each w.r.t. the prior frame

Function Dependacies: Sympy

Debugging function is attached after the end of the function to test it
to use debug function within this script, uncomment the debug function call section 
and enter values for q,t and quat and run the script
for more information about the debug function inputs please refer to the function and
its call section
 
'''
from sympy import *
import tf
import numpy as np

def forward_kinematics():
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

    return transforms
   

############################### Debugging Section ###############################################

def debug(q,t,quat):

    ######## Calculate Pose Using Function Herein ############
    # Initialize the symbolic variables
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 

    # Call forward kinematics function
    transforms = forward_kinematics()
            
    # Create the total transforms from each frame to the base frame
    T0_gripper = transforms[0]   # Transformation matrix from the URDF gripper frame to the base frame
        
    for i in range(1,len(transforms)):
        T0_gripper = T0_gripper*transforms[i]     # Accumulating the transformations

    # Calcuate the pose 4*4 matrix of the robot calculated through the developed forward kinematics function
    # Joint values dictionary
    # Enter Joint Values set in Joint State Publisher

    values={}
    values[q1] = q[0]
    values[q2] = q[1]
    values[q3] = q[2]
    values[q4] = q[3]
    values[q5] = q[4]
    values[q6] = q[5]

    T0_gripper = T0_gripper.evalf(subs=values)

    ######## Calculate Pose Matrix Using RVIZ ACTUAL VALUES ############
    # Create translation vector
    # Enter translation vector valuesof gripper_link from RVIZ
    translation = np.array([t])

    # Create orienatation quaternion
    # Enter orientation quaternion values of gripper_link from RVIZ
    quaternion = quat

    # Create Translation Matrix
    T0_gripper_actual_translation = tf.transformations.translation_matrix(translation)
    # Create Rotation Matrix
    T0_gripper_actual_rotation = tf.transformations.quaternion_matrix(quaternion)
    # Calculate Total Transformation and Cast it into Sympy Matrix
    T0_gripper_actual = Matrix(np.dot(T0_gripper_actual_translation,T0_gripper_actual_rotation))


    ######## Calculating and Printing Pose Error ############
    pprint(T0_gripper_actual-T0_gripper)

'''
# To use the debugging function to test the forward kinematics function
# uncomment this test section and enter the q, t and quat values
# Debugging Function Inputs
q = [-2.91,-0.78,0.25,5.19,0.36,3.57]         # list of joint values in radians

t = [-1.0606, -0.15275, 2.4486]               # list of translation vector entries of the gripper URDF frame w.r.t base (obtained from RVIZ)

quat =  [0.094633, 0.94858, 0.264, -0.14678]  # list of quaternion values of the gripper URDF frame w.r.t base (obtained from RVIZ)

debug(q,t,quat)
'''



