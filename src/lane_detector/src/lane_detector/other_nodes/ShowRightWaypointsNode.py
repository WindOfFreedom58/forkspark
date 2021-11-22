import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from spark_msgs.msg import WaypointData
import numpy as np
from cv_bridge import CvBridge
import cv2

class ShowRightWaypointsNode(Node):

    def __init__(self):
        super().__init__("ShowRightWaypoints")

        self.image_subscription = self.create_subscription(
            CompressedImage,
            "/simulator/camera_node/image/compressed",
            self.image_callback,
            1
        )

        self.waypoint_subscription = self.create_subscription(
            WaypointData,
            "/lane_detection/right_waypoints",
            self.waypoint_callback,
            1
        )

        self.waypoint_image_publisher = self.create_publisher(
            Image,
            "/lane_detection/right_waypoint_view",
            1
        )

        self.image = None

        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.image = img

    def waypoint_callback(self, msg):
        if self.image is not None:
            img = np.copy(self.image)
            x_space = list(msg.x)
            y_space = list(msg.y)

            for x,y in zip(x_space, y_space):
                cv2.circle(img, center=(int(x),int(y)), radius=5, thickness=5,color=(0,0,255))

            self.waypoint_image_publisher.publish(self.bridge.cv2_to_imgmsg(img))


def main(args=None):
    rclpy.init(args=args)

    subscriber = ShowRightWaypointsNode()

    rclpy.spin(subscriber)

    subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()