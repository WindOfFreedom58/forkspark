#!/usr/bin/python

import rospy

from spark_msgs.msg import Waypoints

from spark_msgs.srv import WaypointInit
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


import numpy as np
import csv
import numpy as np
from math import radians, cos, sin, sqrt, atan2

R = 6371*1e3 # Earth's radius

DESIRED_SPEED = 2.0

def haversine_transform(lat0, long0, lat, long):
    dLon = radians(long) - radians(long0)
    latAverage = (lat + lat0)/2
    a = cos(radians(latAverage))*cos(radians(latAverage))*sin(dLon / 2)*sin(dLon/2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1-a))

    x =  dist if long > long0 else -dist

    dLat = radians(lat - lat0)
    a = pow(sin(dLat/2), 2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1-a))
    y = dist if lat > lat0 else -dist

    return x,y

def get_odom_coordinate(lat_source, lon_source, lat_target, lon_target):
    """
    Returns stamped pose to be added to a path message in odom frame
    """
    x,y = haversine_transform(lat_source, lon_source, lat_target, lon_target)
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    return pose_msg
        
class GPSWaypointPublisher:
    def __init__(self):
        self.waypoints = []
        self.odom_waypoints = None

        waypoint_path = rospy.get_param("gps_waypoints_path")
        # wait for reading waypoints..
        while(not self.read_waypoints(waypoint_path)):
            pass
        
        self.gps_waypoints_publisher = rospy.Publisher("/gps_waypoints", Waypoints, queue_size=5)
        self.initial_position = None

        self.gps_subscriber = rospy.Subscriber("/gps", NavSatFix, self.init_gps)   

    def convert_waypoints_to_odom_path(self, waypoints, initial_position):
        waypoints_msg = Waypoints()
        speeds = []
        path_msg = Path()
        path_msg.header.frame_id="odom"
        path_msg.header.stamp=rospy.Time.now()

        if initial_position is None:
            return
        for i, waypoint in enumerate(waypoints):
            pose_msg = get_odom_coordinate(initial_position.latitude, initial_position.longitude, waypoint[0], waypoint[1])
            path_msg.poses.append(pose_msg)
            if i == len(waypoints)-1:
                speeds.append(0.0)
            else:
                speeds.append(DESIRED_SPEED)
        
        waypoints_msg.path = path_msg
        waypoints_msg.speeds = speeds

        self.odom_waypoints = waypoints_msg
    
    def init_gps(self, gps_msg):
        rospy.loginfo("GPS Waypoint Publisher: Waiting for subscriber..")
        while(self.gps_waypoints_publisher.get_num_connections() == 0):
            pass        
        rospy.loginfo("GPS Waypoint Publisher: A subscriber is attached to waypoint publisher, proceeding..")
        
        if self.initial_position is None:
            self.initial_position = gps_msg
            # convert waypoints to odom_waypoints according to 
            self.convert_waypoints_to_odom_path(self.waypoints, self.initial_position)
            self.publish_static_waypoint()

    def read_waypoints(self, waypoint_path):
         with open(waypoint_path) as waypoints_file_handle:
            try:
                self.waypoints = list(csv.reader(waypoints_file_handle,
                                        delimiter=' ',
                                        quoting=csv.QUOTE_NONNUMERIC))
                if(len(self.waypoints) == 0):
                    raise Exception("The waypoints file has no waypoints")
            except Exception as e:
                rospy.logerr(e)

            return True

    def publish_static_waypoint(self):
        if self.odom_waypoints is None:
            rospy.logerr("There are no waypoints to publish in the given path.")
            return
        rospy.loginfo(self.odom_waypoints)
        self.gps_waypoints_publisher.publish(self.odom_waypoints)
        rospy.loginfo("Waypoints succesfully instantiated.")
        # rospy.wait_for_service('waypoint_init')
        # try:
        #     static_waypoint_init = rospy.ServiceProxy('waypoint_init', WaypointInit)
        #     response = static_waypoint_init(self.odom_waypoints)
        #     return response.response
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s" % e)


def main(args=None):
    rospy.init_node("GPSWaypointPublisher")
    static_waypoint_publisher = GPSWaypointPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()