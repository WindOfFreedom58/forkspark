#!/usr/bin/python

import rospy

from spark_msgs.msg import VehicleCmd, BehaviourState
from std_msgs.msg import Float64

import numpy as np
import threading

from PIDControl import PIDControl


class ControlPublisherNode():
    def __init__(self):
        rospy.loginfo("ControlPublisherNode has been started.")

        self.publisher = rospy.Publisher(
            "/vehicle_cmd",
            VehicleCmd,
            queue_size=5
        )

        self.error_subscriber = rospy.Subscriber(
            "/barrier_error",
            Float64,
            self.error_callback
        )

        self.behaviour_subscriber = rospy.Subscriber(
            "/behaviour_planner", BehaviourState, self.handle_behaviour)

        self.reference_speed = 0
        self.brake = 0
        self.blind = False
        self.blindSeconds = 15

        try:
            self.behaviour_states = rospy.get_param("/behaviour_states")
        except KeyError as e:
            rospy.logerr(e)
            rospy.signal_shutdown("param_not_set")
        self.pidControl = PIDControl(self.sendCommand)

    def handle_behaviour(self, behaviour):
        if (behaviour.state == self.behaviour_states["stopped_state"]):
            self.reference_speed = 0
            self.brake = 0
            self.blind = False
        elif (behaviour.state == self.behaviour_states["stop_state"]):
            self.reference_speed = 0
            self.brake = 1
        elif (behaviour.state == self.behaviour_states["straight_state"]):
            self.blind = True
            self.brake = 0
        elif (behaviour.state != self.behaviour_states["park_state"]):
            self.reference_speed = 7
            self.brake = 0
            self.blind = False

    def sendCommand(self, steer):
        rospy.loginfo("Commanding steer angle: %d" % steer)
        cmd = VehicleCmd()
        if self.blind:
            cmd.steer = 0
        else:
            cmd.steer = min(max(steer, -100), 100)
        cmd.speed = self.reference_speed
        cmd.brake = self.brake

        self.publisher.publish(cmd)

    def error_callback(self, error):
        self.pidControl.updateError(error.data)

    def _switchBlind(self):
        self.blind = False


def main(args=None):
    rospy.init_node("ControlPublisherNode")
    ControlPublisherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
