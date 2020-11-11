#!/usr/bin/env python
import PyKDL
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Quaternion
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_about_axis
from tf2_geometry_msgs import do_transform_pose
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener

# listens to /cmd_vel, turns it into a joint state for the naive kin sim

tfBuffer = None # type: Buffer
tf_listener = None


def init(tf_buffer_size=2):
    """
    If you want to specify the buffer size, call this function manually, otherwise don't worry about it.
    :param tf_buffer_size: in secs
    :type tf_buffer_size: int
    """
    global tfBuffer, tf_listener
    tfBuffer = Buffer(rospy.Duration(tf_buffer_size))
    tf_listener = TransformListener(tfBuffer)
    rospy.sleep(1.0)

def lookup_pose(target_frame, source_frame, time=None):
    p = PoseStamped()
    p.header.frame_id = source_frame
    if time is not None:
        p.header.stamp = time
    p.pose.orientation.w = 1.0
    return pose_to_kdl(transform_pose(target_frame, p).pose)

def transform_pose(target_frame, pose):
    global tfBuffer
    if tfBuffer is None:
        init()
    try:
        transform = tfBuffer.lookup_transform(target_frame,
                                              pose.header.frame_id,  # source frame
                                              pose.header.stamp,
                                              rospy.Duration(5.0))
        new_pose = do_transform_pose(pose, transform)
        return new_pose
    except ExtrapolationException as e:
        print(e)

def pose_to_kdl(pose):
    """Convert a geometry_msgs Transform message to a PyKDL Frame.
    :param pose: The Transform message to convert.
    :type pose: Pose
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.orientation.x,
                                                 pose.orientation.y,
                                                 pose.orientation.z,
                                                 pose.orientation.w),
                       PyKDL.Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z))

def make_twist(x, y, rot):
    t = PyKDL.Twist()
    t.vel[0] = x
    t.vel[1] = y
    t.rot[2] = rot
    return t

def cmd_vel_sub(data):
    """
    :type data: Twist
    """
    twist = make_twist(data.linear.x, data.linear.y, data.angular.z)
    twist = lookup_pose(odom_frame, base_frame).M * twist
    js = JointState()
    js.name = [x_joint, y_joint, z_joint]
    js.velocity.append(twist.vel[0])
    js.velocity.append(twist.vel[1])
    js.velocity.append(twist.rot[2])
    cmd_pub.publish(js)


if __name__ == '__main__':
    try:
        rospy.init_node('base_controller')
        init()
        x_joint = rospy.get_param('~odom_x_joint')
        y_joint = rospy.get_param('~odom_y_joint')
        z_joint = rospy.get_param('~odom_z_joint')
        odom_frame = rospy.get_param('~odom')
        base_frame = rospy.get_param('~base_footprint', 'base_footprint')
        cmd_vel_sub = rospy.Subscriber('~cmd_vel', Twist, cmd_vel_sub, queue_size=10)
        cmd_pub = rospy.Publisher('~commands', JointState, queue_size=10)
        rospy.loginfo('base controller running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
