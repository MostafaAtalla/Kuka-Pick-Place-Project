[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

[//]: # (Image References)
[image1]: ./misc_images/kuka_DH_diagram.png
[image2]: ./misc_images/DH-frames2.jpg
[image3]: ./misc_images/FK1.png
[image4]: ./misc_images/FK2.png
[image5]: ./misc_images/correction.png
[image6]: ./misc_images/wristcenter.png
[image7]: ./misc_images/wirstcenter2.png
[image8]: ./misc_images/IK1.png
[image9]: ./misc_images/inverseorientation.png
[image10]: ./misc_images/beta.png
[image11]: ./misc_images/alpha.png
[image12]: ./misc_images/gamma.png

# Robotic Pick and Place - Kuka KR210 Project
## Description
This project simulates the amazon pick and place challenge where a robot is required to pick items from shelves and place them in a container. The challenge to to do this pick and place action correctly every time regardless of which shelf the item is on.

In this project, a KuKa KR210 is simulated in Gazebo simulation environement is required pick a cylinderical item from shelves set in front of the robot and place it in a container beside it. This task invovles:
- Identifying the item location
- Plan a path to approach the item and a path to place in the container
- Solve the IK problem for the path points to obtain the corresponding joint angles
- Create feasible trajectories for each join to follow the cartesian path
- Follow those trajectories using an appropriate controller

The main task required in this project is to correctly solve the forward and inverse kinematics problems of the arm and code it up in an IK ROS service. This service works in a request-response basis where other nodes might request the inverse kinematics solution for a specific trajectory points and the service will respond with the IK solution accordingly. 

In this write up, a complete report about the project is presented with details about the kinematic analysis and the project implementation. The steps taken to complete this project are:
1 - Solve the forward kinematics problem by:
- Properly set the DH frames and construct the DH paramters table
- Extract the DH constant parameters from the URDF file

2 - Solve the inverse kinematics problem by:
- Decouple the IK problem into inverse position and inverse orientation problems
- Solve the inverse position problem first using the geometrical approach
- Solve the inverse oreintation problem second using a similar geometrical approach given the inverse position solution obtained in the previous step

## Kinematic Analysis
### Forward Kinematics
### DH Frames Assignment 
The picture below illustrates the DH frames assignment used to solve the FK problem of the arm. The general procedure for asigning frames is to:
- Assign Z axes to the joints's axis of rotation
- Assign X axes to the common normal between Zi-1 and Zi
- Construct Y in accordance with the right hand rule
- For the first frame, make Z1 identical to Z0
- For the gripper frame assign Z and X similar to Z6 and X6 respectively

It is worth noting that the final girpper DH frame does not necessarily coincide with the gripper URDF frame ( which is the case here). In this case, a transformation is needed between both frames to make sure that the pose calculations is accurate. In the picture below, the red colored frames correspond to the DH frames and the yellow ones correspond to the URDF frames (Notice the orientation difference in the gripper frames between both of them).


![alt text][image1]

### DH Parameters
Using the constructed DH frames, the DH parameters table below is obtained.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | - pi/2 | a2 | 0 | q2-pi/2
2->3 | 0 | a3 | 0 | q3
3->4 |  -pi/2 | a4 | d4 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->g | 0 | 0 | dg | 0

The constant DH parameters are obtained from the URDF file of the Kuka KR210 to be as follows:
d1 = 0.33 + 0.42 = 0.75
a2 = 0.35
a3 = 1.25
a4 = -0.054   d4 = 0.96 + 0.54 = 1.5
dg = 0.11+0.193 = 0.303

For better understanding of what those parameters are on the physical robot, please refer to the image below where each parameter is shown on the robot schematic.

![alt text][image2]

### FK Transformation Matrices 
The deduced DH parameters above are used to calculate the transformation matrices between each frame and the subsequent one from the base frame to the gripper frame. Each row of the DH table represents the parameters required to build the transformation between each pair of frames (for example row 1 represents the parameters required to build the transformations between frame 1 and frame 2). To build each transformation, the 4 DH parameters are plugged into the following sequence of transformations to obtained the matrix shown below. 

![alt text][image3]
![alt text][image4]

### Correction Transformation
One last transformation needed here is the transformation between the gripper DH frame and the gripper URDF frame. This transformation is needed to align both frames to make sure that the simulation pose we get from ROS is expressed in the correct frame. This transformation has only a rotation part while the translation part is zero since both of them share the same origin. The picture below shows the transformation deduced between the two frames with information about how construction of the transformation matrix.
![alt text][image5]
### Inverse Kinematics
Once we have our FK done, we are ready to start working on the inverse kinematics solution. The inverse kinematics solution as discussed earlier can be splitted into two sub-problems: 1) Inverse position that includes the first three joint values (theta1, theta2, theta3) 2) Inverse orientation that involves the last three joint values (theta4, theta5 , theta6). This problem simplification is possible because of the robot design that has a spherical wrist where the last three joints axes intersect in only one point. For more information about that the interested reader is referred to: 
https://robotacademy.net.au/lesson/different-approach-to-solving-inverse-kinematics/ 

### Inverse Postion 
To solve the inverse postion problem, first we need to obtain the spherical wrist postion vector. This can be done through a simple vector subtraction operation illustrated in the picutre below.

![alt text][image6]

The terms used in thos equation are as follows:
- W vector is the position vector of the wrist center
- P vector is the position vector of the gripper frame
- R is the corrected rotation matrix of the gripper frame (rotation of the end effector frame after being corrected from the urdf frame to the dh frame)
- d is the DH paramters that represents the euclidean distance between the wrist center and the end effector frame
- The last vector is the Z vector which will be rotated to correspond to the direction of the vector from the wrist center to the end effector frame

The following picture shows the geometric interpretation of the vector subtration operation done in the previous step.

![alt text][image7]

After successfully finding the wrist center postion, the inverse postion problem can be solved by dudcing each joint angle using a geometric approach. The derivation of each joint angle is illustrated in the image below.

![alt text][image8]

### Inverse Orientation
The first step to solve the inverse oreintation problem is to calculate the rotation matrix from the wrist center to the end effector. This can be done as follows:
- Calculate R0_3: the rotation matrix from base frame to the wrist center
- Subsititute the inverse position values obtained earlier (theta1, theta2 ,theta3) to obtain the numric R0_3 
- Calculate R3_g: the rotation matrix from wrist center to end effetor frame. This can be done through the operation illustrated in the picture below where R0_6 corresponds to R0_g 

![alt text][image9]

After that, the 3 last joint angles can be treated as a combination of euler angles since their axes of riotation intersect in the same point. To obtain this combination of euler angles, we can use the matrix formula for the euler angles rotation (based on the convetion we select) and try to match the entries of this formula with the numeric matrix R3_g computed earlier. The convention used in this project is the XYZ extrinsic rotations shown in the picutre below.

![alt text][image10]

The three euler angles can be computing by matching the entries of both matrices as follows:
![alt text][image11]
![alt text][image12]
![alt text][image13]


Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

