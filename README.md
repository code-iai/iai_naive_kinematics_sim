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

### Running unit tests
This package comes with a couple of basic unit tests. You can run them like this:
```shell
roscd iai_naive_kinematics_sim
catkin run_tests --this --no-deps
```

## Usage
This package provides a simulator that supports two modes. In the first mode, it can be used as a standalone naive kinematics simulator. In the second mode, it serves as simulation node within projection framework. The following subsection document how to use each of the modes.

### Standalone simulation
TODO: add a figure, and update this subsection

This node periodically publishes on ```/joint_states``` and subscribes to velocity commands for a configurable subset of its joints. It is recommended to use ```rviz``` in conjunction with the ```robot_state_publisher``` for visualizing the simulated robot.

There are two test/tutorial launch-files provided in this package to show-case the usage of the ```velocity_resolved_sim```:
* ```roslaunch iai_naive_kinematics_sim test_sim.launch``` starts the simulation for a minimalistic mechanism.

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

### Projection mode
TODO: add a figure depicting the ROS interface

#### Differences to simulation mode
In projection mode, the simulator waits for an external clock to advance the simulation step-by-step. To this end, it subscribes to the topic ```~projection_clock``` with message type ```iai_naive_kinematics_sim/ProjectClock```. That message provides both the next time stamp for the header of the simulated joint states, and it also specifies how much time shall be simulated in the next step.

Additionally, the simulator publishes a message of type ```std_msgs/Header``` on the topic ```~commands_received``` after it has received a new set of joint commands. This message indicates that the simulator is ready for another simulation step, and shall be used to avoid race conditions when using the simulator in a fast-running projection.

#### Tutorial
To examine the workings of the projection in action, follow this minimalistic tutorial. 

Open four terminals that you can see at the same time to run the tutorial:

shell $1: ```roslaunch iai_naive_kinematics_sim test_projection.launch```

This starts the simulator in projection mode with the following internal state:
```shell
header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:  0
  frame_id: ''
name: ['joint1', 'joint2']
position: [0.5, -0.03]
velocity: [0.0, 0.0]
effort: [0.0, 0.0]
```

shell $2: ```rostopic echo /joint_states```

In this shell, you can see the joint states that the simulator publishes. Right now, you should see no messages on this topic.

shell $3: ```rostopic echo /simulator/simulator/commands_received```

In this shell, you can see the acknowledged commands that the simulator received. Also here, you should see no messages in the beginning.

shell $4: ```roscd iai_naive_kinematics_sim/test_scripts```

This is the shell that we use to send commands to the simulator.

First, we trigger the simulator to run a simulation step by calling this in $4: ```./trigger_projection```

This script sends a single clock message with time stamp 123.01 and asks for a simulation step of 10ms. You should see the following joint state in the console:
```shell
header: 
  seq: 0
  stamp: 
    secs: 123
    nsecs:  10000000
  frame_id: ''
name: ['joint1', 'joint2']
position: [0.5, -0.03]
velocity: [0.0, 0.0]
effort: [0.0, 0.0]
```

Secondly, we send a new command to the simulator by calling this in $4: ```./send_cmd```

This scripts tells the simulator to move ```joint1``` with a velocity of 0.1. On the ```~commands_received``` topic you should now see the header that the script send to the simulator:

```shell
seq: 1
stamp: 
  secs: 123
  nsecs:         0
frame_id: ''
```

If we now re-trigger the simulator with ```./trigger_projection```, we will get this joint-state on the ```/joint_states``` topic:

```shell
header: 
  seq: 1
  stamp: 
    secs: 123
    nsecs:  10000000
  frame_id: ''
name: ['joint1', 'joint2']
position: [0.501, -0.03]
velocity: [0.1, 0.0]
effort: [0.0, 0.0]
```

Now, velocity and position and ```joint1``` have changed as expected. Note, that the time stamp of this new joint state is unchanged. The reason is that ```./trigger_projection``` always sends the same time stamp and that the simulator blindly copies it.

## Known limitations:
The efforts of the ```/joint_states``` are not part of the simulation. They are always set to 0.

In simulation mode, the simulator does not provide a simulated clock like other simulators, e.g. gazebo.
