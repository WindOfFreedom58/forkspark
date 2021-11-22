#!/usr/bin/python

import rospy
from spark_msgs.srv import WaypointInit, WaypointInitResponse
from spark_msgs.msg import Pose
from spark_msgs.msg import Waypoint

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class WaypointVisualizer:
    def __init__(self):
        self.waypoint_init_service = rospy.Service('waypoint_init', WaypointInit, self.handle_waypoint_init)
        #self.visualization_timer = rospy.Timer(rospy.Duration(0.02), self.visualize)
        self.visualization_waypoint_publisher = rospy.Publisher("/waypoint_visualizer/markers", MarkerArray, queue_size=5)
        
        self.waypoints = None

    def handle_waypoint_init(self,req):
        try:
            self.waypoints = req.waypoints
            self.visualize()
            return WaypointInitResponse(200)
        except Exception as e:
            rospy.logerr(str(e))
            return WaypointInitResponse(500)   

    def visualize(self):
        if self.waypoints == None:
            rospy.logerr("Waypoints has not been instantiated.")
            return
        marker_array = MarkerArray()
        marker_idx = 0
        for waypoint in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.id = marker_idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = 11.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_idx+=1
            marker_array.markers.append(marker)
        self.visualization_waypoint_publisher.publish(marker_array)

def main(args=None):
    rospy.init_node("WaypointVisualizer")
    waypoint_visualizer = WaypointVisualizer()
    rospy.spin()

if __name__=="__main__":
    main()