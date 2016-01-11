# iai_naive_kinematics_sim
TODO: document this package!

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
