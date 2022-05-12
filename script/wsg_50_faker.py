#!/usr/bin/env python
import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from iai_wsg_50_msgs.msg._PositionCmd import PositionCmd
from iai_wsg_50_msgs.msg._Status import Status
from sensor_msgs.msg._JointState import JointState
from std_msgs.msg._Bool import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class WSG50SimDriver(object):
    def __init__(self, gripper_name, gripper_state, gripper_follow_joint_trajectory):
        self.js = None
        self.link_id = None
        self.gripper_name = gripper_name
        self.gripper_state = gripper_state
        self.gripper_follow_joint_trajectory = gripper_follow_joint_trajectory
        self.joints = rospy.wait_for_message(self.gripper_state,
                                             JointTrajectoryControllerState).joint_names

        self.moving_pub = rospy.Publisher('~moving', Bool, queue_size=10)
        self.state_pub = rospy.Publisher('~status', Status, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.js_cb, queue_size=10)
        rospy.wait_for_message('joint_states', JointState)
        self.follow_joint_traj_client = actionlib.SimpleActionClient(self.gripper_follow_joint_trajectory,
                                                                     FollowJointTrajectoryAction)
        self.follow_joint_traj_client.wait_for_server()
        self.goal_pose_sub = rospy.Subscriber('~goal_position', PositionCmd, self.goal_pose_cb, queue_size=10)
        self.goal_speed_sub = rospy.Subscriber('~goal_speed', PositionCmd, self.goal_speed_cb, queue_size=10)

    def js_cb(self, data):
        if self.link_id is None:
            self.link_id = data.name.index(self.gripper_name)
        self.js = data
        moving = Bool()
        try:
            moving.data = abs(self.js.velocity[self.link_id]) > 1e-4
        except IndexError:
            return
        self.moving_pub.publish(moving)
        state = Status()
        state.header.stamp = rospy.get_rostime()
        state.width = self.js.position[self.link_id]
        state.speed = self.js.velocity[self.link_id] * 1000
        self.state_pub.publish(state)

    def goal_pose_cb(self, data):
        rospy.loginfo('Position command: pos={}, speed={}'.format(data.pos, data.speed))
        multiplier = 1000

        traj = JointTrajectory()
        traj.joint_names = self.joints
        jtp = JointTrajectoryPoint()
        jtp.positions = list(rospy.wait_for_message(self.gripper_state,
                                               JointTrajectoryControllerState).actual.positions)
        jtp.positions[self.joints.index(self.gripper_name)] = data.pos / multiplier
        jtp.velocities = []
        jtp.accelerations = []
        jtp.time_from_start = rospy.Duration(0.5)
        traj.points = [jtp]

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        self.follow_joint_traj_client.send_goal_and_wait(goal)
        rospy.loginfo('done.')

    def goal_speed_cb(self, data):
        self.goal_pose_cb(data)


if __name__ == '__main__':
    rospy.init_node('wsg_50')
    gripper_joint_name = rospy.get_param('~gripper_joint_name', default='gripper_joint')
    gripper_state = rospy.get_param('~gripper_state', default='gripper_controller/state')
    gripper_follow_joint_trajectory = rospy.get_param('~gripper_follow_joint_trajectory', default='gripper_controller/follow_joint_trajectory')
    node = WSG50SimDriver(gripper_joint_name, gripper_state, gripper_follow_joint_trajectory)
    rospy.loginfo("kms 50 sim driver for '{}' joint running.".format(gripper_joint_name))
    rospy.spin()
