[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)


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
### DH Frames Assignment 
The picture below illustrates the DH frames assignment used to solve the FK problem of the arm. 

[//]: # (Image References)
[image1]: ./misc_images/kuka_DH_diagram.png

[alt text][image1]











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

