#!/usr/bin/python

import rospy

from nav_msgs.msg import Odometry
from math import atan2

from spark_msgs.msg import Pose

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

class PosePublisherNode:
    def __init__(self):
        self.gps_subscriber = rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=5)
        self.pose_publisher = rospy.Publisher("/localization/pose", Pose, queue_size=5)

    
    def odometry_callback(self, odom_msg):
        pose_msg = Pose()
        pose_msg.x = odom_msg.pose.pose.position.x
        pose_msg.y = odom_msg.pose.pose.position.y
        pose_msg.yaw = quaternion_to_yaw(odom_msg.pose.pose.orientation)
        pose_msg.closed_speed = odom_msg.twist.twist.linear.x
        pose_msg.header.stamp = rospy.Time.now()

        self.pose_publisher.publish(pose_msg)
    

def main():
    rospy.init_node("PosePublisherNode")
    pose_publisher_node = PosePublisherNode()
    rospy.spin()

if __name__ == '__main__':
    main()
