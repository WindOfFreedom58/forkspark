#!/usr/bin/python

from spark_msgs.msg import BehaviourState, ObjectDistances
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix

from tf.transformations import quaternion_from_euler

from math import pi

import rospy


class ParkingPlanner():
    def __init__(self):
        # parking command listener
        self.behaviour_subscriber = rospy.Publisher(
            "/behaviour_planner", BehaviourState, queue_size=5)
        # Parking positions from left to right
        self.park_state_start_position = NavSatFix() # TODO: do this
        self.park_positions = [NavSatFix(latitude=40.76351, longitude=29.23491), NavSatFix(latitude=40.76351, longitude=29.23491), NavSatFix(latitude=40.76351, longitude=29.23491), NavSatFix(
            latitude=40.76351, longitude=29.23491), NavSatFix(latitude=40.76351, longitude=29.23491), NavSatFix(latitude=40.76351, longitude=29.23491), NavSatFix(latitude=40.76351, longitude=29.23491)]
        self.goal_state_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=3)
        # behaviour states table
        try:
            self.behaviour_states = rospy.get_param("behaviour_states")
        except KeyError as e:
            rospy.logerr(e)
            rospy.signal_shutdown("param not set")
        # states
        self.PARKING = False
        self.park_stop_yaw = pi
        rospy.loginfo("Parking planner initiated!")

    def park_sign_callback(self, sign_distances_msg):
        park_point = None
        for i in range(len(sign_distances_msg.classes)):
            if sign_distances_msg.classes[i].data == "park":
                if park_point is None:
                    park_point = sign_distances_msg.poses[i]
                else:
                    if sign_distances_msg.poses[i].y < park_point.y:
                        park_point = sign_distances_msg.poses[i].y
        if park_point is not None and self.PARKING is True:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = park_point.x
            pose_msg.pose.position.y = park_point.y
            pose_msg.pose.orientation = Quaternion(
                quaternion_from_euler(0.0, 0.0, self.park_stop_yaw))
            self.goal_state_publisher.publish(pose_msg)

    def behaviour_callback(self, behaviour_msg):
        if behaviour_msg.state == self.behaviour_states["park_state"]:
            self.PARKING = True
        else:
            self.PARKING = False
