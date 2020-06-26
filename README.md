# Project-2-Go-Chase-It
## Introduction
This is a project for Udacity's Robotic Software Engineering NanoDegree. It's a robot that finds a white ball in a forward-facing camera and drives towards it. 

## Concepts Explored
Project 2 covers the following concepts
- Gazebo model and world-building
- Gazebo plugins
- URDF (Universal Robotic Description Format) Files
- ROS publishers, subscribers, and services
- RVIZ (3D visualization tool for ROS)
- ROSCPP (C++)
- CMakeLists

## Getting Started
To view this project, you must have Gazebo and ROS installed on Linux
ROS installation: http://wiki.ros.org/ROS/Installation 
Gazebo installation: http://gazebosim.org/

Next you will need to create a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace), and then clone my repo as the src folder into the workspace. Navigate to your home director and do the following commands

* $ mkdir -p catkin_ws
* $ cd catkin_ws
* $ git clone https://github.com/DariusStu/Project-2-Go-Chase-It.git
* $ cd catkin_ws/src
* $ catkin_init_workspace

Now navigate to the tope level of the catkin workspace directory and build the executables:
* $ cd .. && catkin_make
* $ source devel/setup.bash

Time to launch gazebo with the robot in it, run the following command in terminal:
* $ roslaunch drive_bot world.launch

Open another terminal window and run the following commands
* $ cd catkin_ws
* $ source devel/setup.bash
* $ roslaunch ball_chaser ball_chaser.launch

Launching this will start the process image node for the forward-facing camera and commands the robot to chase the white ball in its field of view. Add a ball to the field of view by navigating to the insert tab in gazebo and place the ball in front of the drive_bot.
