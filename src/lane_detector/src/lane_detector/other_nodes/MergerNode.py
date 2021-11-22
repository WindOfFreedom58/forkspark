import rclpy
import numpy as np
from rclpy.node import Node
from spark_msgs.msg import LaneWaypoints,WaypointData

class MergerNode(Node):
    def __init__(self):
        super().__init__("Merger")
        # Merge Everything into one topic

        self.publisher = self.create_publisher(
            LaneWaypoints,
            "lane_detection/waypoint_pixels",
            1
        )

        self.left_subscriber = self.create_subscription(
            WaypointData,
            "lane_detection/left_waypoints",
            self.callback,
            3
        )

        self.right_subscriber = self.create_subscription(
            WaypointData,
            "lane_detection/right_waypoints",
            self.callback,
            3
        )

        self.is_left = False
        self.is_right = False
        self.is_front = False

        self.message = LaneWaypoints()


    def callback(self, msg):
        source = msg.name

        if source == "left":
            x = np.array(list(msg.y)).astype("int16").tolist()
            y = np.array(list(msg.x)).astype("int16").tolist()

            self.message.left_y = x
            self.message.left_x = y

            self.is_left = True

        elif source == "right":
            x = np.array(list(msg.y)).astype("int16").tolist()
            y = np.array(list(msg.x)).astype("int16").tolist()

            self.message.right_y = x
            self.message.right_x = y

            self.is_right = True

        else:
            x = np.array(list(msg.y)).astype("int16").tolist()
            y = np.array(list(msg.x)).astype("int16").tolist()

            self.message.forward_y= x
            self.message.forward_x = y

            self.is_front = True

        if self.is_left and self.is_right:
            self.publisher.publish(self.message)
            self.message = LaneWaypoints()
            self.is_left = False
            self.is_right = False
            self.is_front = False


def main(args=None):
    rclpy.init(args=args)

    subscriber = MergerNode()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()