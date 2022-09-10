# Landmark Monitoring Simulation in Gazebo using ROS1      
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white) ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)	![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)

## Table of Content
* [Task Description](#task-description)
* [Environment Setup](#environment-setup)

## Task Description
A robot `turtlebot3` is moving the unknown world, that contains some objects like `construction_barrel, dumpster, jersey_barrier, number1 etc`.
- Figure out what is closest landmark/object to the robot.
- Give distance of each landmark/object from the robot

## Environment Setup:
**If ROS-Noetic is not installed** [follow this](http://wiki.ros.org/noetic/Installation/Ubuntu)

`If ROS-Noetic Version already installed in the system` : Run the following commands in the linux terminal to setup the ros workspace with turtlebot3 packages.
```shell
$ cd ~/catkin_ws/src             # will only run if catkin_ws is available at root directory
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
$ cd ~/catkin_ws && catkin_make

# for installing the turtlebo3_simulation
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

```







