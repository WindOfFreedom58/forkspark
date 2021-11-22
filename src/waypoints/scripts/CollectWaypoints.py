#!/usr/bin/python

import rospy
from datetime import datetime

from sensor_msgs.msg import NavSatFix

class WaypointCollector:
    def __init__(self):
        self.gps_subscriber = rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        self.save_string = ""
        rospy.on_shutdown(self.save_waypoints)
        self.waypoint_file_name = "{}_waypoints".format(datetime.now())

        self.frequency_divider = 50
        self.counter = 0

    def gps_callback(self, gps_msg):
        if self.counter == self.frequency_divider - 1:
            rospy.loginfo("Got 1 GPS point..")
            latitude = gps_msg.latitude
            longitude = gps_msg.longitude
            self.save_string = self.save_string + str(latitude) + ", " + str(longitude) + "\n"
            self.counter = 0
        else:
            self.counter+=1
    
    def save_waypoints(self):
        rospy.loginfo("Saving GPS waypoints")
        if self.save_string == "":
            return
        
        with open(self.waypoint_file_name, "w") as f:
            f.write(self.save_string)


def main(args=None):
    rospy.init_node("WaypointCollector")
    wc = WaypointCollector()
    rospy.spin()

if __name__ == '__main__':
    main()
