# iai_naive_kinematics_sim
A set of naive robot kinematics simulators for ROS. Main design goal: Providing ROS-based kinematics simulators for early-stage development of kinematics algorithms that are light-weight in comparison to full-scale simulators, e.g. Gazebo.

## Installation
### Installation from source using ```catkin``` and ```rosdep```
This installation assumes you are using ROS Indigo on Ubuntu 14.04.

Step-by-step instructions for installing this package in a ```catkin``` workspace located at ```~/catkin_ws/src``` with the ```~/.bashrc``` properly set up and source:
* ```rosdep update```
* ```cd ~/catkin_ws/src```
* ```wstool merge https://raw.githubusercontent.com/code-iai/iai_naive_kinematics_sim/master/rosinstall/base.rosinstall```
* ```wstool update```
* ```rosdep install --ignore-src --from-paths iai_naive_kinematics_sim```
* ```cd ~/catkin_ws```
* ```catkin_make```
