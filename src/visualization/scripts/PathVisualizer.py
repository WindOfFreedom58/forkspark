import rospy
from spark_msgs.msg import Waypoints

from nav_msgs.msg import Path


class PathVisualizer:
    def __init__(self):
        self.waypoint_subscriber = rospy.Subscriber(
            rospy.get_param("~waypoint_topic"), Waypoints, self.waypoint_callback, queue_size=5)
        self.marker_publisher = rospy.Publisher(
            "/path_visualization", Path, queue_size=5)
        self.marker_count = 0

    def waypoint_callback(self, waypoint_msg):
        self.marker_publisher.publish(waypoint_msg.path)


def main():
    rospy.init_node("path_visualizer")
    vis = PathVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
