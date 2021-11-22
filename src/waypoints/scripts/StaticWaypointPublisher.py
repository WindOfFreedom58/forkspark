#!/usr/bin/python

import rospy

from spark_msgs.msg import Waypoint
from spark_msgs.msg import Pose

from spark_msgs.srv import WaypointInit

import numpy as np
import csv

def convert_waypoint_list_to_pose_msgs(waypoints, waypoints_pose_msg_list):
    for waypoint in waypoints:
        pose_msg = Pose()
        pose_msg.x = waypoint[0]
        pose_msg.y = waypoint[1]
        pose_msg.open_speed = 8.0
        waypoints_pose_msg_list.append(pose_msg)

class StaticWaypointPublisher:
    def __init__(self):
        self.waypoints = []
        waypoint_path = rospy.get_param("waypoint_path")
        self.read_waypoints(waypoint_path)

        if self.publish_static_waypoint() == 200:
            rospy.loginfo("Static waypoints succesfully instantiated.")
        else:
            rospy.logerror("Static waypoints cannot be instantiated.")
    
    def read_waypoints(self, waypoint_path):
         with open(waypoint_path) as waypoints_file_handle:
            waypoints = list(csv.reader(waypoints_file_handle,
                                       delimiter=' ',
                                       quoting=csv.QUOTE_NONNUMERIC))
            convert_waypoint_list_to_pose_msgs(waypoints, self.waypoints)

    def publish_static_waypoint(self):
        if len(self.waypoints) == 0:
            rospy.logerror("There are no waypoints to publish in the given path.")
            return
        
        rospy.wait_for_service('waypoint_init')
        try:
            static_waypoint_init = rospy.ServiceProxy('waypoint_init', WaypointInit)
            response = static_waypoint_init(self.waypoints)
            return response.response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


def main(args=None):
    rospy.init_node("StaticWaypointPublisher")
    static_waypoint_publisher = StaticWaypointPublisher()
    rospy.spin()
if __name__ == '__main__':
    main()