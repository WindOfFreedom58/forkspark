#!/usr/bin/python

import rospy

from spark_msgs.msg import Pose, Waypoint, BehaviourState, VehicleCmd, Waypoints

from spark_msgs.srv import WaypointInit, WaypointInitResponse

from lgsvl_msgs.msg import VehicleControlData
from nav_msgs.msg import Path, Odometry

import tf.transformations as transformations

from StanleyControl import StanleyControl

import numpy as np


class ControllerNode:
    def __init__(self):
        self.stanley_control = StanleyControl()

        self.waypoint_init_service = rospy.Service(
            'waypoint_init', WaypointInit, self.handle_waypoint_init)
        self.waypoint_subscriber = rospy.Subscriber(
            "/parking_waypoints", Waypoints, self.update_waypoint, queue_size=5)

        self.ego_state_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.update_values, queue_size=5)
        self.behaviour_subscriber = rospy.Subscriber(
            "/behaviour_planner", BehaviourState, self.update_behaviour_state, queue_size=5)

        self.controller_timer = rospy.Timer(
            rospy.Duration(0.02), self.publish_control)
        self.control_publisher = rospy.Publisher(
            "/vehicle_cmd", VehicleCmd, queue_size=5)

        self.local_waypoints = None
        self.first_frame = False

        self.behaviour_states = rospy.get_param("behaviour_states")

        self.publish_control_switch = False

    def handle_waypoint_init(self, req):
        try:
            self.local_waypoints = req.waypoints
            if self.local_waypoints is not None and self.local_waypoints != []:
                self.stanley_control.update_waypoints(self.local_waypoints)
                self.publish_control_switch = True
            else:
                self.publish_control_switch = False
            return WaypointInitResponse(200)
        except Exception as e:
            rospy.logerr(str(e))
            return WaypointInitResponse(500)

    def update_behaviour_state(self, behaviour_state):

        if behaviour_state.state == self.behaviour_states["stop_state"]:
            self.stanley_control.set_desired_speed(0)
        elif behaviour_state.state == self.behaviour_states["left_state"] or behaviour_state.state == self.behaviour_states["right_state"]:
            self.stanley_control.set_desired_speed(2)
        elif behaviour_state.state == self.behaviour_states["park_state"]:
            # NO NEED TO DO ANYTHING. #TODO: test
            rospy.loginfo("TESTTT")
            self.stanley_control.set_desired_speed(5)

    def update_waypoint(self, waypoint_msg):
        rospy.loginfo("Coming here")
        self.local_waypoints = waypoint_msg.path
        self.stanley_control.update_waypoints(
            self.local_waypoints, waypoint_msg.speeds)
        self.publish_control_switch = True

    """
    Unless pose message comes, do not update the controls
    """
    # TODO: If pose message is old, stop controls

    def update_values(self, pose_msg):
        orientation = pose_msg.pose.pose.orientation
        self.stanley_control.update_values(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y,
                                           transformations.euler_from_quaternion(
                                               [orientation.x, orientation.y, orientation.z, orientation.w])[2],
                                           pose_msg.header.stamp.to_sec(),
                                           self.first_frame)
        if not self.first_frame:
            rospy.loginfo("First frame skipped")
            self.first_frame = True
        if self.local_waypoints is not None and self.local_waypoints != []:
            self.stanley_control.update_controls()

    def publish_control(self, timer):
        if self.publish_control_switch:
            cmd_speed, cmd_steer, cmd_brake = self.stanley_control.get_commands()
            control_msg = VehicleCmd()
            control_msg.speed = cmd_speed
            control_msg.steer = cmd_steer
            control_msg.brake = cmd_brake
            self.control_publisher.publish(control_msg)


def main(args=None):
    rospy.init_node("ControllerNode")
    controller_node = ControllerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
