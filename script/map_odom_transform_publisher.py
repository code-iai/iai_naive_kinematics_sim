#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Transform
from tf2_msgs.msg import TFMessage
from iai_naive_kinematics_sim.srv import SetMapOdomTransform
from threading import Lock, Thread
import sys


class TransformPublisher(object):

    def __init__(self, child_frame, parent_frame, topic_name, publish_rate=10):
        self.tf_pub = rospy.Publisher(u'/tf', TFMessage, queue_size=10)
        self.thread = Thread(target=self.loop)
        self.transform = Transform()
        self.transform.rotation = Quaternion(0, 0, 0, 1)
        self.rate = rospy.Rate(publish_rate)
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.service = rospy.Service(topic_name, SetMapOdomTransform, self.update_transform_cb)
        self.lock = Lock()
        self.running = True
        self.thread.start()

    def __del__(self):
        self.running = False
        self.thread.join()

    def loop(self):
        while not rospy.is_shutdown() and self.running:
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
    rospy.init_node("map_odom_transform_publisher")
    while not rospy.has_param('~child_frame') or not rospy.has_param('~parent_frame'):
        if(rospy.is_shutdown()):
            sys.exit(0)
        print('waiting for param {0}/child_frame and/or {0}/parent_frame'.format(rospy.get_name()))
        rospy.sleep(1)
    child_frame = rospy.get_param('~child_frame', "")
    parent_frame = rospy.get_param('~parent_frame', "")
    TransformPublisher(child_frame, parent_frame, '~update_map_odom_transform')
    rospy.spin()
