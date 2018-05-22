#!/usr/bin/env python

from multiprocessing import Lock
import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseGoal
from tf2_geometry_msgs import do_transform_pose
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster


class TfWrapper(object):
    def __init__(self, buffer_size=2):
        self.tfBuffer = Buffer(rospy.Duration(buffer_size))
        self.tf_listener = TransformListener(self.tfBuffer)
        self.tf_static = StaticTransformBroadcaster()
        self.tf_frequency = rospy.Duration(.05)
        self.broadcasting_frames = {}
        self.broadcasting_frames_lock = Lock()
        rospy.sleep(0.1)

    def transform_pose(self, target_frame, pose):
        try:
            transform = self.tfBuffer.lookup_transform(target_frame,
                                                       pose.header.frame_id,  # source frame
                                                       pose.header.stamp,
                                                       rospy.Duration(2.0))
            new_pose = do_transform_pose(pose, transform)
            return new_pose
        except ExtrapolationException as e:
            rospy.logwarn(e)

    def lookup_transform(self, target_frame, source_frame):
        p = PoseStamped()
        p.header.frame_id = source_frame
        p.pose.orientation.w = 1.0
        return self.transform_pose(target_frame, p)

    def set_frame_from_pose(self, name, pose_stamped):
        with self.broadcasting_frames_lock:
            frame = TransformStamped()
            frame.header = pose_stamped.header
            frame.child_frame_id = name
            frame.transform.translation = pose_stamped.pose.position
            frame.transform.rotation = pose_stamped.pose.orientation
            self.broadcasting_frames[name] = frame

    def start_frame_broadcasting(self):
        self.tf_broadcaster = TransformBroadcaster()
        self.tf_timer = rospy.Timer(self.tf_frequency, self.broadcasting_cb)

    def broadcasting_cb(self, _):
        with self.broadcasting_frames_lock:
            for frame in self.broadcasting_frames.values():
                frame.header.stamp = rospy.get_rostime()
                self.tf_broadcaster.sendTransform(frame)

class NavpFaker(object):
    def __init__(self):
        name = 'nav_pcontroller/move_base'
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_root = rospy.get_param('~root_frame', 'base_footprint')
        self.server = actionlib.SimpleActionServer(name, MoveBaseAction, execute_cb=self.cb, auto_start=False)
        self.simple_goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.simple_goal_cb)
        self.server.start()
        # send simple 2d nav goal using action client to prevent warnings
        self.self_client = actionlib.SimpleActionClient(name, MoveBaseAction)
        self.tf = TfWrapper()
        p = PoseStamped()
        p.header.frame_id = self.odom_frame
        p.pose.orientation.w = 1
        self.tf.set_frame_from_pose(self.robot_root, p)
        self.tf.start_frame_broadcasting()

    def simple_goal_cb(self, goal_pose):
        """
        Callback for the 2D nav goal button in rviz.
        :param goal_pose: goal position for the nav_p controller
        :type goal_pose: PoseStamped
        """
        g = MoveBaseGoal()
        g.target_pose = goal_pose
        self.self_client.send_goal(g)

    def cb(self, move_base_goal):
        """
        Callback for nav_p action server
        :param move_base_goal: goal position for the nav_p controller
        :type move_base_goal: MoveBaseGoal
        """
        move_base_goal = self.tf.transform_pose(self.odom_frame, move_base_goal.target_pose)
        self.tf.set_frame_from_pose(self.robot_root, move_base_goal)
        self.server.set_succeeded(MoveBaseResult())


if __name__ == '__main__':
    rospy.init_node('nav_p')
    muh = NavpFaker()
    rospy.loginfo('navp faker running')
    rospy.spin()
