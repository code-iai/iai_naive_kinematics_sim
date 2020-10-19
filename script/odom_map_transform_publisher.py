#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Transform
from tf2_msgs.msg import TFMessage
from iai_naive_kinematics_sim.srv import SetMapOdomTransform
from threading import Lock
import numpy as np


class OdomTransformPublisher(object):

    def __init__(self, name):
        rospy.init_node(name)
        self.tf_pub = rospy.Publisher(u'/tf', TFMessage, queue_size=10)
        while not rospy.has_param('~child_frame'):
            logging.loginfo('waiting for param ' + '~child_frame')
            rospy.sleep(1)
        self.child_frame = rospy.get_param('~child_frame', "")
        while not rospy.has_param('~parent_frame'):
            logging.loginfo('waiting for param ' + '~parent_frame')
            rospy.sleep(1)
        self.parent_frame = rospy.get_param('~parent_frame', "")
        self.transform = Transform()
        self.transform.rotation = Quaternion(0, 0, 0, 1)
        self.rate = rospy.Rate(10)
        self.service = rospy.Service('~update_map_odom_transform', SetMapOdomTransform, self.update_transform_cb)
        self.lock = Lock()
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            self.publish_transform()
            self.rate.sleep()

    def update_transform_cb(self, req):
        with self.lock:
            self.transform = req.transform
            self.transform.rotation = self.normalize_quaternion_msg(self.transform.rotation)
        return True

    def publish_transform(self):
        tf_msg = TFMessage()
        tf =  TransformStamped()
        with self.lock:
            tf.transform = self.transform
        tf.header.frame_id = self.parent_frame
        tf.header.stamp = rospy.get_rostime()
        tf.child_frame_id = self.child_frame
        tf_msg.transforms.append(tf)
        self.tf_pub.publish(tf_msg)

    def normalize_quaternion_msg(self, quaternion):
        q = Quaternion()
        rotation = np.array([quaternion.x,
                             quaternion.y,
                             quaternion.z,
                             quaternion.w])
        normalized_rotation = rotation / np.linalg.norm(rotation)
        q.x = normalized_rotation[0]
        q.y = normalized_rotation[1]
        q.z = normalized_rotation[2]
        q.w = normalized_rotation[3]
        return q


if __name__ == "__main__":
    OdomTransformPublisher("map_odom_transform_publisher")
