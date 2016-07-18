# iai_naive_kinematics_sim
A set of naive robot kinematics simulators for ROS. Main design goal: Providing ROS-based kinematics simulators for early-stage development of kinematics algorithms that are light-weight in comparison to full-scale simulators, e.g. Gazebo.

![rviz view](https://raw.githubusercontent.com/code-iai/iai_naive_kinematics_sim/master/doc/pr2_sim_example.png)

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

## ROS Interface
###  Velocity-resolved simulation: ```velocity_resolved_sim```
This node periodically publishes on ```/joint_states``` and subscribes to velocity commands for a configurable subset of its joints. It is recommended to use ```rviz``` in conjunction with the ```robot_state_publisher``` for visualizing the simulated robot.

There are two test/tutorial launch-files provided in this package to show-case the usage of the ```velocity_resolved_sim```:
* ```roslaunch iai_naive_kinematics_sim test_sim.launch``` starts the simulation for a minimalistic mechanism.
* ```roslaunch iai_naive_kinematics_sim pr2.launch``` starts the for the PR2.

Topic Publications:
* ```/joint_states``` (sensor_msgs/JointState): joint positions, velocities, and efforts for all joints of type ```prismatic```, ```revolute```, or ```continuous``` present URDF in parameter ```/robot_description```.

Topic Subscriptions:
* ```/<joint_name>/vel_cmd``` (std_msgs/Float64): commanded next velocity for a single joint; set of subscriptions can be configured through private ROS parameter ```~controlled_joints``` at deploy-time

Parameters:
* ```/robot_description``` (urdf map) [mandatory]: The urdf xml robot description used for bootstrapping the simulation. All joints of type ```prismatic```, ```revolute```, or ```continuous``` will be simulated.
* ```~controlled_joints``` (string list) [mandatory]: The list of joint names for which a command subscription shall be opened. Has to be a subset of the simulated joints, i.e. joints in  ```/robot_description``` with type ```prismatic```, ```revolute```, or ```continuous```.
* ```~start_config``` (string-double map) [optional, default: all zero]: Simulation start position for an arbitrary subset of the simulated joints.
* ```~watchdog_period``` (double) [optional, default: 0.1s]: Watchdog period used for all controlled joints. Note: Has to be greater than 0s.
* ```~sim_frequency``` (double) [optional, default: 50Hz]: Frequency with which the joints are simulated and published.

Convenience features:
* ```watchdog```: For every joint there is a separate watchdog. If a controlled joint and its watchdog has not received a new command for ```watchdog_period``` then the watchdog sets the velocity command for that joint to 0.
* ```position joint limits```: Joints may not leave their position limits as specified in ```/robot_description```. Every joint that does, has its velocity set to zero its position will stay at its limit subsequent commands take it out of the limit.

Known limitations:
* ```efforts```: The efforts of the ```/joint_states``` are not part of the simulation. They shall be always be set to 0.

## Usage example
### PR2 velocity-resolved simulation
Execute these step-by-step instructions in separate shells to bring up the simulation:
* ```roscore```
* ```roslaunch iai_naive_kinematics_sim pr2.launch```
* ```rviz```

Now configure ```rviz``` to view the simulated robot:
* select ```base_link``` as fixed frame
* add a plugin of type ```RobotModel```

To move, e.g. the ```torso_lift_link``` with 10cm/s, enter the following command in the shell:
* ```rostopic pub /torso_lift_joint/vel_cmd std_msgs/Float64 "data: 0.1" -r 20```

### Boxy velocity-resolved simulation
Execute these step-by-step instructions in separate shells to bring up the simulation:
* ```roscore```
* ```roslaunch iai_naive_kinematics_sim boxy.launch```
* ```rviz```

Now configure ```rviz``` to view the simulated robot:
* select ```base_link``` as fixed frame
* add a plugin of type ```RobotModel```

To move, e.g. the ```triangle_base_link``` with 10cm/s, enter the following command in the shell:
* ```rostopic pub /triangle_base_velocity_controller/command std_msgs/Float64 "data: 0.1" -r 20```
