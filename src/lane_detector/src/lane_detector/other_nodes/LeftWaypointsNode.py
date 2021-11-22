import rclpy
from rclpy.node import Node
from spark_msgs.msg import WaypointData, LaneCoeffs
import numpy as np
import cv2


class LeftWaypointsNode(Node):
    def __init__(self):
        super().__init__("LeftWaypoints")

        self.subscriber = self.create_subscription(
            LaneCoeffs,
            "/lane_detection/left_coeffs",
            self.callback,
            1
        )

        self.publisher = self.create_publisher(
            WaypointData,
            "/lane_detection/left_waypoints",
            1
        )

        self.publisher2 = self.create_publisher(
            LaneCoeffs,
            "/lane_detection/left_waypoint_coeffs",
            1
        )


    def callback(self, msg,n_samples = 100, x_max = 1920, y_max=1080):
        slope = msg.slope
        bias = msg.bias

        t_slope, t_bias = self.transform(slope, bias)

        if t_slope is not np.inf:
            # Ortaya göre yapalım önce.
            start_x = x_max/2
            start_y = y_max

            x_space = np.linspace(start_x - n_samples/2, start_x + n_samples/2, n_samples +1, dtype="float64")
            line_coeffs = np.array([t_slope, start_y - t_slope*start_x])
            line = np.poly1d(line_coeffs)
            y_space = line(x_space)

            # Coefficients of the line found and whether it is vertical or rotated
            data2 = LaneCoeffs()
            data2.name = "Rotated Left"
            data2.slope = line_coeffs[0]
            data2.bias = line_coeffs[1]

            # Waypoint data
            data = WaypointData()
            data.name = "left"
            data.x = list(x_space)
            data.y = list(y_space)

            self.publisher2.publish(data2)
            self.publisher.publish(data)

        else:
            data = WaypointData()
            data2 = LaneCoeffs()

            data.name = "left"
            data.x = [x_max/2]*n_samples
            data.y = list(np.linspace(y_max-100, y_max-100-n_samples-1, n_samples, dtype="float64"))

            data2.name = "Vertical Left"
            data2.slope = 999999999999999.
            data2.bias = x_max/2

            self.publisher2.publish(data2)
            self.publisher.publish(data)


    def transform(self, slope, bias, vertical_slope_thresh=20):
        # Rotate the line 90 degrees
        rot_slope = (1/slope)
        rot_bias = bias

        # If slope's absolute value is greater than threshold, the line is vertical
        if abs(rot_slope) >= vertical_slope_thresh:
            rot_slope = np.inf


        return (rot_slope, rot_bias)


def main(args=None):
    rclpy.init(args=args)

    subscriber = LeftWaypointsNode()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()