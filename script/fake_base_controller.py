#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# listens to /cmd_vel, turns it into a joint state for the naive kin sim

def cmd_vel_sub(data):
    """
    :type data: Twist
    """
    js = JointState()
    js.name = [x_joint, y_joint, z_joint]
    js.velocity.append(data.linear.x)
    js.velocity.append(data.linear.y)
    js.velocity.append(data.angular.z)
    cmd_pub.publish(js)


if __name__ == '__main__':
    try:
        rospy.init_node('base_controller')
        x_joint = rospy.get_param('~odom_x_joint')
        y_joint = rospy.get_param('~odom_y_joint')
        z_joint = rospy.get_param('~odom_z_joint')
        cmd_vel_sub = rospy.Subscriber('~cmd_vel', Twist, cmd_vel_sub, queue_size=10)
        cmd_pub = rospy.Publisher('~commands', JointState, queue_size=10)
        rospy.loginfo('base controller running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
