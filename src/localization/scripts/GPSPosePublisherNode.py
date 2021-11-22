#!/usr/bin/python
import rospy

import sys
import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import Int8, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry

from spark_msgs.msg import BehaviourState, ObjectDistances

from math import atan2

import numpy as np
import rospy
import copy
from math import pi

import tf.transformations

import numpy as np
from math import radians, cos, sin, sqrt, atan2

R = 6371 * 1e3  # Earth's radius

DESIRED_SPEED = 2.0


def get_distance(x, y):
    return sqrt(x ** 2 + y ** 2)


def haversine_transform(lat0, long0, lat, long):
    dLon = radians(long) - radians(long0)
    latAverage = (lat + lat0) / 2
    a = cos(radians(latAverage)) * cos(radians(latAverage)) * \
        sin(dLon / 2) * sin(dLon / 2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1 - a))

    x = dist if long > long0 else -dist

    dLat = radians(lat - lat0)
    a = pow(sin(dLat / 2), 2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1 - a))
    y = dist if lat > lat0 else -dist

    return x, y


def get_odom_coordinate(lat_source, lon_source, lat_target, lon_target):
    """
    Returns stamped pose to be added to a path message in odom frame
    """
    x, y = haversine_transform(lat_source, lon_source, lat_target, lon_target)
    pose_msg = Pose()
    pose_msg.position.x = x
    pose_msg.position.y = y
    return pose_msg


def get_yaw_from_coordinates(odom_source, odom_target):
    dx = odom_target.pose.pose.position.x - odom_source.pose.pose.position.x
    dy = odom_target.pose.pose.position.y - odom_source.pose.pose.position.y
    yaw = atan2(dy, dx)
    return yaw


def get_rotation(q1, q2):
    yaw1 = tf.transformations.euler_from_quaternion(
        [q1.x, q1.y, q1.z, q1.w])[2]
    yaw2 = tf.transformations.euler_from_quaternion(
        [q2.x, q2.y, q2.z, q2.w])[2]

    rotation_yaw = yaw2 - yaw1
    return tf.transformations.quaternion_from_euler(0.0, 0.0, rotation_yaw)


yaw_difference_limit = 2 * pi  # radianss


class PosePublisherNode():
    def __init__(self):
        self.current_gps = None
        self.initial_gps = None

        self.previous_odom = None
        self.previous_yaw = None  # radians
        self.local_odom = None
        self.odom_trans = None

        self.gps_subscriber = rospy.Subscriber(
            "/tcpfix", NavSatFix, self.gps_callback, queue_size=10)
        self.init_gps_pose_listener = rospy.Subscriber(
            "/gps_pose/init", Int8, self.init_pose_publish)
        self.plan_ok_listener = rospy.Subscriber("/parking_ok", Empty, self.plan_ok_listener_callback)

        self.local_odom_publisher = rospy.Publisher(
            "/odom", Odometry, queue_size=50)
        self.br = tf2_ros.TransformBroadcaster()
        self.br_static = tf2_ros.StaticTransformBroadcaster()

        self.PARK_MODE = False
        self.PARK_PLANNED = False
        self.park_state_start_position = NavSatFix(
             latitude=40.79016537, longitude=29.509226214)
        # NavSatFix(
        #     latitude=40.7901491883, longitude=29.5091696233)


        self.park_mode_start_threshold = 3  # m
        self.park_index = rospy.get_param("parking_point", 2)
        self.park_positions = [NavSatFix(latitude=40.7902307483, longitude=29.50930095),
                               NavSatFix(latitude=40.7902500483, longitude=29.50927743),
                               NavSatFix(latitude=40.7902655533, longitude=29.5092585616), NavSatFix(
                latitude=40.790278463, longitude=29.509231395), NavSatFix(latitude=40.790300366, longitude=29.50921927833),
                               NavSatFix(latitude=40.79031358, longitude=29.50919531)]
        self.park_stop_yaw = 0.6361157  # radians # TODO: CHECK THIS

        self.goal_state_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=3)
        self.behaviour_publisher = rospy.Publisher(
            "/behaviour_planner", BehaviourState, queue_size=5)

        self.publish_timer = rospy.Timer(
            rospy.Duration(0.02), self.publish_callback)
        rospy.loginfo("Odom local frame node initiated!")

    def plan_ok_listener_callback(self, plan_ok_msg):
        self.PARK_PLANNED = True

    def init_pose_publish(self, init_msg):
        """
        Init messsage is not important, only come to this callback
        """
        if self.current_gps is None:
            rospy.logwarn("GPS poses are not being updated...")
            return
        self.initial_gps = self.current_gps

    def init_park_mode(self):
        self.behaviour_publisher.publish(
            BehaviourState(state=4))  # publish park mode
        self.init_pose_publish(init_msg=None)  # set gps pose

    def publish_goal_point(self):
        rospy.loginfo("Publishing goal point")
        parking_point = self.park_positions[self.park_index]
        park_odom = get_odom_coordinate(
            self.initial_gps.latitude, self.initial_gps.longitude, parking_point.latitude, parking_point.longitude)

        park_pose_stamped = PoseStamped()
        park_pose_stamped.pose.position.x = park_odom.position.x
        park_pose_stamped.pose.position.y = park_odom.position.y
        quat = tf.transformations.quaternion_from_euler(
            0.0, 0.0, self.park_stop_yaw)
        park_pose_stamped.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3])
        self.goal_state_publisher.publish(park_pose_stamped)

    def gps_callback(self, gps_msg):
        # Update current gps
        self.current_gps = gps_msg

        # Check if parking mode should start
        park_start_point_coord = get_odom_coordinate(
            self.current_gps.latitude, self.current_gps.longitude, self.park_state_start_position.latitude,
            self.park_state_start_position.longitude)
        distance = get_distance(
            park_start_point_coord.position.x, park_start_point_coord.position.y)
        rospy.loginfo("Distance: " + str(distance))
        if (distance < self.park_mode_start_threshold and self.PARK_MODE == False):
            self.init_park_mode()
            self.PARK_MODE = True

        if self.initial_gps is None:
            return

        current_time = rospy.Time.now()

        # Use haversine transform to get the odom coordinate
        local_odom = Odometry()
        local_odom.pose.pose = get_odom_coordinate(
            self.initial_gps.latitude, self.initial_gps.longitude, gps_msg.latitude, gps_msg.longitude)
        # rospy.loginfo(local_odom.pose.pose)
        local_odom.header.stamp = current_time
        local_odom.header.frame_id = "odom"

        # Leave orientation empty for the first frame, handle in controller
        if self.previous_odom is not None:
            yaw = get_yaw_from_coordinates(self.previous_odom, local_odom)
            if self.previous_yaw is not None:
                if abs(yaw - self.previous_yaw) > yaw_difference_limit:
                    yaw = self.previous_yaw
            quat = tf.transformations.quaternion_from_euler(
                0.0, 0.0, yaw)
            local_odom.pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])
            self.previous_yaw = yaw
            if self.PARK_PLANNED == False:
                self.publish_goal_point()
        else:
            quat = tf.transformations.quaternion_from_euler(
                0.0, 0.0, 0.0)
            local_odom.pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = local_odom.pose.pose.position.x
        odom_trans.transform.translation.y = local_odom.pose.pose.position.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = local_odom.pose.pose.orientation

        self.local_odom = local_odom
        self.odom_trans = odom_trans
        self.previous_odom = local_odom

    def publish_callback(self, timer_msg):
        if (self.local_odom is None or self.odom_trans is None):
            return
        self.local_odom_publisher.publish(self.local_odom)
        self.br.sendTransform(self.odom_trans)


def main(args=None):
    rospy.init_node("GPSPosePublisher")
    localizer = PosePublisherNode()
    rospy.spin()


if __name__ == '__main__':
    main()
