import rospy
from spark_msgs.msg import ObjectDistances

from visualization_msgs.msg import Marker


class TrafficSignVisualizer:
    def __init__(self):
        self.traffic_sign_subscriber = rospy.Subscriber(
            "/object_distances", ObjectDistances, self.sign_callback, queue_size=5)
        self.marker_publisher = rospy.Publisher(
            "/traffic_sign_markers", Marker, queue_size=5)
        self.marker_count = 0

    def sign_callback(self, object_distances_msg):
        rospy.loginfo("Publishing marker for traffic sign")
        for i in range(len(object_distances_msg.classes)):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.id = self.marker_count
            marker.lifetime = rospy.Duration(5.0)

            marker.text = object_distances_msg.classes[i].data
            marker.pose.position.x = object_distances_msg.poses[i].x
            marker.pose.position.y = object_distances_msg.poses[i].y
            marker.pose.position.z = object_distances_msg.poses[i].z
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            self.marker_count += 1

            self.marker_publisher.publish(marker)


def main():
    rospy.init_node("traffic_sign_visualizer")
    vis = TrafficSignVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
