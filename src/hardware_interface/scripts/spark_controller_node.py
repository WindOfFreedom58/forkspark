#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from spark_msgs.msg import VehicleCmd, BehaviourState
from std_msgs.msg import String, Int8, Float64

from spark_control_interface import SparkControlInterface

import serial

class SPARKControllerNode:

    def __init__(self):
        try:
            self.control_interface = SparkControlInterface(rospy.get_param("drive_motor_port"), rospy.get_param("steer_port"))
        except KeyError as e:
            rospy.logerr("SPARKControllerNode: Port parameters has not been set")
            rospy.signal_shutdown("param_not_set")
        

        self.state_dict = {"brake": False, "forward": False,
                           "reverse": False, "stop": False,
                           "enable": False, "autonomous": False}

        self.command_numbers = {"forward": 0, "brake": 1, "stop":2, "reverse":3, "autonomous":4, "enable": 7}

        self.prev_state_dict = self.state_dict.copy()

        self.last_pressed_buttons = [0] * 11
        self.last_speed = 0
        self.last_steer = 0.0
        self.max_speed = 6

        
        self.steer_angle_publisher = rospy.Publisher("/wheel_angle", Float64, queue_size=10)
        self.steer_angle_timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.steer_angle_max = 0.40 # radians

        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
        self.command_subscriber = rospy.Subscriber("/vehicle_cmd", VehicleCmd, self.cmd_callback, queue_size=1)
        self.bp_publisher = rospy.Publisher("/behaviour_planner/init_signal", Int8, queue_size=1)

    def timer_callback(self, timer):
        msg = Float64()
        msg.data = self.last_steer
        self.steer_angle_publisher.publish(msg)

    def joy_callback(self, msg):
        # converting joy message to vehicle command message
        msg_speed = msg.axes[1]*self.max_speed
        msg_steer = round(msg.axes[3]*100,1)
        pressed_button_index = -1

        for index, button in enumerate(self.last_pressed_buttons):
            # if a button is unpressed
            if button == 1 and msg.buttons[index] == 0:
                pressed_button_index = index
        self.last_pressed_buttons = msg.buttons

        if self.state_dict["autonomous"]:
            if pressed_button_index == self.command_numbers["enable"] or pressed_button_index == self.command_numbers["autonomous"]:
                self.common_callback(pressed_button_index, self.last_speed, self.last_steer)
            if pressed_button_index == 6: # back button
                rospy.loginfo("Switching to go state")
                self.bp_publisher.publish(Int8(1))
        else:
            if pressed_button_index == self.command_numbers["autonomous"]:
                msg_speed = 0.0
                msg_steer = 0.0
            self.common_callback(pressed_button_index, msg_speed, msg_steer)
    
    def cmd_callback(self, msg):
        if self.state_dict["autonomous"]:
            command_no = -1
            if msg.speed > 0:
                if not self.state_dict["reverse"]: # wheel goes forward in the reverse motor mode
                    command_no = self.command_numbers["reverse"]
            elif msg.speed < 0:
                if not self.state_dict["forward"]:
                    command_no = self.command_numbers["forward"]
            elif msg.brake > 0:
                if not self.state_dict["brake"]:
                    command_no = self.command_numbers["brake"]

            self.common_callback(command_no, msg.speed, msg.steer)

    def common_callback(self, command_no, speed, steer):
        if command_no != -1:
            # check for states
            # switch mode = LB
            if 4 == command_no:
                self.set_all_false()
                self.state_dict["autonomous"] = not self.state_dict["autonomous"]
                print(self.state_dict["autonomous"])
                self.last_speed = 0.0
                self.last_steer = 0.0
            # emergency stop = Start
            elif 7 == command_no:
                self.set_all_false()
                self.state_dict["enable"] = not self.state_dict["enable"]
            elif self.state_dict["enable"]:
                # brake = B
                if 1 == command_no:
                    self.set_all_false("brake")
                    self.state_dict["brake"] = not self.state_dict["brake"]
                # stop = X
                elif 2 == command_no:
                    self.set_all_false("stop")
                    self.state_dict["stop"] = not self.state_dict["stop"]
                # forward = A
                elif 0 == command_no:
                    self.set_all_false("forward")
                    self.state_dict["forward"] = not self.state_dict["forward"]
                # reverse = Y
                elif 3 == command_no:
                    self.set_all_false("reverse")
                    self.state_dict["reverse"] = not self.state_dict["reverse"]
        if command_no !=-1 or self.last_speed != speed or self.last_steer != steer:
            self.send_states(speed, steer, command_no != -1)

    def set_all_false(self, remember=""):
        for state in self.state_dict.keys():
            if state == remember or state == "enable" or state=="autonomous":
                continue
            self.state_dict[state] = False

    def send_states(self, speed, steer, is_state_changed):
        mode_codes = []
        vehicle_cmd = VehicleCmd()

        if is_state_changed:
            for state in self.state_dict.keys():
                if self.state_dict[state] != self.prev_state_dict[state]:
                    mode_code = String()
                    mode_code.data = state
                    mode_codes.append(mode_code)
        vehicle_cmd.modes = mode_codes

        if self.state_dict["enable"]:
            vehicle_cmd.speed = speed
            vehicle_cmd.steer = steer
        else:
            vehicle_cmd.speed = 0.0
            vehicle_cmd.steer = 0.0

        self.control_interface.command_vehicle(vehicle_cmd)
        self.last_speed = vehicle_cmd.speed
        self.last_steer = vehicle_cmd.steer
        self.prev_state_dict = self.state_dict.copy()

    def on_shutdown(self):
        self.control_interface.end_serial_loop = True
        self.control_interface.thread_check_serial.join()

def main(args=None):
    rospy.init_node("SPARKControllerNode")
    controller_node = SPARKControllerNode()
    #rospy.on_shutdown(controller_node.on_shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()