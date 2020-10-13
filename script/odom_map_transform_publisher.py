#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Transform
from tf2_msgs.msg import TFMessage
from iai_naive_kinematics_sim.srv import SetMapOdomTransform, SetOdomMapTransformRequest, SetOdomMapTransformResponse 

from giskardpy import logging
from giskardpy.plugin import GiskardBehavior
from giskardpy.utils import normalize_quaternion_msg


class OdomTransformPublisher(object):

    def __init__(self, name):
        rospy.init_node(name)
        self.tf_pub = rospy.Publisher(u'/tf', TFMessage, queue_size=10)
        self.transform = Transform()
        self.transform.rotation = Quaternion(0, 0, 0, 1)
        self.rate = rospy.Rate(10)
        self.service = rospy.Service('update_map_odom_transform', SetMapOdomTransform, self.update_transform_cb)
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            self.publish_transform()
            self.rate.sleep()

    def update_transform_cb(self, req):
        self.transform = req.transform
        return True

    def publish_transform(self):
        try:
            tf_msg = TFMessage()
            tf =  TransformStamped()
            tf.transform = self.transform
            #tf.header = fk.header
            tf.header.frame_id = "map"
            tf.header.stamp = rospy.get_rostime()
            tf.child_frame_id = "odom_combined"
            #tf.transform.translation.x = fk.pose.position.x
            #tf.transform.translation.y = fk.pose.position.y
            #tf.transform.translation.z = fk.pose.position.z
            #tf.transform.rotation = normalize_quaternion_msg(fk.pose.orientation)
            tf_msg.transforms.append(tf)
            self.tf_pub.publish(tf_msg)
        except KeyError as e:
            pass
        except UnboundLocalError as e:
            pass
        except ValueError as e:
            pass


if __name__ == "__main__":
    OdomTransformPublisher("map_odom_transform_publisher")
