#!/usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix


class GPSWaypointSaver:
    def __init__(self):
        self.navsat_subscriber = rospy.Subscriber(
            "/tcpfix", NavSatFix, self.navsat_callback)
        self.waypoint_file = open("waypoint_file_parkur.txt", "w")

    def navsat_callback(self, nav_msg):
        self.waypoint_file.write("%.10f %.10f\n" % (
            nav_msg.latitude, nav_msg.longitude))

    def on_shutdown(self):
        self.waypoint_file.close()


if __name__ == '__main__':
    rospy.init_node("gps_waypoint_saver")
    waypoint_saver = GPSWaypointSaver()
    rospy.on_shutdown(waypoint_saver.on_shutdown)
    rospy.spin()
