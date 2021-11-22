#!/usr/bin/python
import rospy
import serial

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import tf2_ros
from tf_conversions import transformations
import math

from threading import Thread

class WheelOdometryNode():
    def __init__(self, rate):
        rospy.loginfo("Serial Monitor Node for Wheel Odometry has been activated")

        self.wheel_angle_subscriber = rospy.Subscriber("/wheel_angle", Float64, self.update_wheel_angle)

        self.port_name = rospy.get_param("wheel_odometry_port")
        self.rate = rate

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time = None

        self.velocity = 0.0 # m/s

        self.wheel_angle = 0.0 #rad

        # Odometry params
        self.pulse_per_rotation = 120
        self.wheel_distance = 1.25 # m TODO: change this
        self.L = 1.36 # m TODO: change this
        self.wheel_radius = 0.235 # m 
        
        self.vt = (1/self.pulse_per_rotation) * 2 * math.pi * self.wheel_radius

        self.end_serial_loop = False
        self.serial_port = serial.Serial(
            port = self.port_name,
            baudrate= self.rate,
            bytesize= 8,
            timeout = .1,
            stopbits=serial.STOPBITS_ONE
        )

        self.serial_port.flushInput()
        self.serial_port.flushOutput()

        self.odometry_publisher = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=3
        )
        self.br = tf2_ros.TransformBroadcaster()
        self.odom_timer = rospy.Timer(rospy.Duration(0.01), self.publish_odom_callback)

        self.check_serial_thread = Thread(target=self.checkSerial)
        self.check_serial_thread.start()

    def update_wheel_angle(self, msg):
        self.wheel_angle = msg.data
        
    def checkSerial(self):
        serial_string_wheel = ""
        while True:
            if self.end_serial_loop:
                break
            if self.serial_port.in_waiting > 0:
                serial_string_wheel = self.serial_port.readline().decode("ascii").strip()
                self.serial_port.flushInput()
                if serial_string_wheel == "t":
                    serial_string_wheel = ""
                    self.yaw = max(-180, min(180, self.yaw + (self.vt * math.tan(self.wheel_angle))/self.L))
                    self.x += self.vt * math.cos(self.yaw)
                    self.y += self.vt * math.sin(self.yaw)            
                    rospy.loginfo(self.x)

    def publish_odom_callback(self, timer):
        message = Odometry()
        odom_transform = TransformStamped()
        
        current_time = rospy.Time.now()
        quat = transformations.quaternion_from_euler(0.0,0.0, self.yaw)

        message.header.stamp = current_time
        message.header.frame_id = "odom"
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        odom_transform.header.stamp = current_time
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        self.odometry_publisher.publish(message)
        self.br.sendTransform(odom_transform)

    def on_shutdown(self):
        self.end_serial_loop = True
        self.check_serial_thread.join(timeout=0.5)
    
if __name__ == "__main__":
    rate = 115200

    rospy.init_node("WheelOdometry")

    wheel_odometry_node = WheelOdometryNode(rate)

    rospy.on_shutdown(wheel_odometry_node.on_shutdown)

    rospy.spin()