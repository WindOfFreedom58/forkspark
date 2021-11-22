
import rospy

import sys

import tf_conversions

import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from math import atan2

import numpy as np
import rospy
import copy

## TODO: UNDER DEVELOPMENT, DO NOT USE
import tf_conversions


def get_rotation(q1, q2):
    yaw1 = tf_conversions.transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])[2]
    yaw2 = tf_conversions.transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])[2]

    rotation_yaw = yaw2 - yaw1
    return tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, rotation_yaw)

class OdomLocalFrameNode():
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber(
            "/odom_gps", Odometry, self.odom_callback, queue_size=10)

        self.local_odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.br = tf2_ros.TransformBroadcaster()
        self.br_static = tf2_ros.StaticTransformBroadcaster()
                
        self.local_odom = None
        self.odom_trans = None
        self.initial_odom = None

        self.publish_timer = rospy.Timer(rospy.Duration(0.02), self.publish_callback)
        rospy.loginfo("Odom local frame node initiated!")

    def publish_static_odom_world(self, initial_odom):
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "world"
        static_tf.child_frame_id="odom"
        static_tf.transform.translation.x = initial_odom.pose.pose.position.x;
        static_tf.transform.translation.y = initial_odom.pose.pose.position.y;
        static_tf.transform.translation.z = initial_odom.pose.pose.position.z;
        static_tf.transform.rotation = initial_odom.pose.pose.orientation;
        rospy.loginfo("Static tf published")
        self.br_static.sendTransform(static_tf)
        


    def odom_callback(self, odom_msg):
        current_time = rospy.Time.now()

        local_odom = odom_msg
        local_odom.header.stamp = current_time
        local_odom.header.frame_id = "odom"

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        if self.initial_odom is None:
            self.initial_odom = copy.deepcopy(odom_msg)
            local_odom.pose.pose.position.x = 0.0
            local_odom.pose.pose.position.y = 0.0
            local_odom.pose.pose.position.z = 0.0
            local_odom.pose.pose.orientation.x = 0.0
            local_odom.pose.pose.orientation.y = 0.0
            local_odom.pose.pose.orientation.z = 0.0
            local_odom.pose.pose.orientation.w = 0.0
            rospy.loginfo("Local odometry set")
            rospy.loginfo(self.initial_odom)
            rospy.loginfo(local_odom)
            self.publish_static_odom_world(self.initial_odom)

        else:
            local_odom.pose.pose.position.x -= self.initial_odom.pose.pose.position.x
            local_odom.pose.pose.position.y -= self.initial_odom.pose.pose.position.y
            local_odom.pose.pose.position.z -= self.initial_odom.pose.pose.position.z
            r = get_rotation(self.initial_odom.pose.pose.orientation, local_odom.pose.pose.orientation)
            local_odom.pose.pose.orientation.x = r[0]
            local_odom.pose.pose.orientation.y = r[1]
            local_odom.pose.pose.orientation.z = r[2]
            local_odom.pose.pose.orientation.w = r[3]

        odom_trans.transform.translation.x = local_odom.pose.pose.position.x;
        odom_trans.transform.translation.y = local_odom.pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = local_odom.pose.pose.orientation;

        self.local_odom = local_odom
        self.odom_trans = odom_trans

    def publish_callback(self, timer_msg):
        if(self.local_odom is None or self.odom_trans is None):
            return
        self.local_odom_publisher.publish(self.local_odom)
        self.br.sendTransform(self.odom_trans)

def main(args=None):
    rospy.init_node("OdomLocalPublisher")
    localizer = OdomLocalFrameNode()
    rospy.spin()


if __name__ == '__main__':
    main()    

