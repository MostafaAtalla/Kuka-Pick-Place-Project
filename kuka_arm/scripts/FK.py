'''
This function takes the following inputs:
1. DH table dictionary containing the following key-values pairs:
Key: 'a'     (link length)   - Value: list of link lengths
Key: 'alpha' (link twist)    - Value: list of link twist angles in radians
Key: 'd'     (joint offset)  - Value: list of joint offsets 
Key: 'q'     (joint angles)  - Value: list of joint angles 
2. Transformation matrix between the DH gripper frame and the URDF gripper frame

It produces the following outputs:
1. Sympy transformation matrix from the base to the gripper frame 
2. List of Sympy transformation matrices from base to gripper frame w.r.t. the base frame
3. List of Sympy transformation matrices from base to gripper frame each w.r.t. the prior frame

Dependacies: Sympy
'''
def forward_kinematics(DH,Tdhg_urdfg):
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

    # Create the gripper frame correction matrix to align the DH frames with the URDF Frames
    # The rotation matrix of this transformations is the unit vectors of the URDF gripper frame
    # With respect to the DH gripper frame. The translation component is zero
    transforms.append(Tdhg_urdfg)

    # Create the total transforms from each frame to the base frame
    T0_frame= [transforms[0]]    # Transformations list from each frame to the base sequentially

    T0_gripper = transforms[0]   # Transformation matrix from the URDF gripper frame to the base frame

    for i in range(1,len(transforms)):
        T0_gripper = T0_gripper*transforms[i]     # Accumulating the transformations
        T0_frame.append(T0_gripper)               # Appending the transformations from each frame to the base

    return (T0_gripper,T0_frame,transforms)



