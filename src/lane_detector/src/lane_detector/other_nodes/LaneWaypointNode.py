import rclpy

from rclpy.node import Node

from spark_msgs.msg import LaneWaypoints
from spark_msgs.msg import Waypoints
from spark_msgs.msg import WaypointsList

import numpy as np


def convert_msg_to_waypoint(waypoint_pixels_msg, direction):
    if direction == "right":
        x = waypoint_pixels_msg.right_x
        y = waypoint_pixels_msg.right_y
    elif direction == "left":
        x = waypoint_pixels_msg.left_x
        y = waypoint_pixels_msg.left_y
    else:
        x = waypoint_pixels_msg.forward_x
        y = waypoint_pixels_msg.forward_y

    if x is None or y is None:
        return None

    x.reverse()
    y.reverse()

    return np.vstack((np.asarray(x),
                      np.asarray(y))).T


class LaneWaypointNode(Node):
    def __init__(self):
        super().__init__("LaneWaypoint")

        self.quadratic_subscriber = self.create_subscription(LaneWaypoints, "/lane_detection/waypoint_pixels",
                                                             self.waypoint_callback)

        # Timer for testing waypoint generation node
        # self.timer = self.create_timer(20, self.timer_callback)

        self.waypoint_publisher = self.create_publisher(WaypointsList, "/lane_detection/waypoints", 1)
        self.visualization_waypoint_publisher = self.create_publisher(WaypointsList, "/waypoint_visualizer/waypoints", 1)

        self.calibration_dict = {'CameraExtrinsicMat': np.asarray([[2.22044605e-16, 0.00000000e+00, 1.00000000e+00,
                                                                    -2.00000003e-01],
                                                                   [-1.00000000e+00, 2.22044605e-16, 2.22044605e-16,
                                                                    -0.00000000e+00],
                                                                   [-2.22044605e-16, -1.00000000e+00, 1.23259516e-32,
                                                                    1.70000005e+00],
                                                                   [0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                                    1.00000000e+00]]),
                                 'CameraMat': np.asarray([[1.15803374e+03, 0.00000000e+00, 9.60000000e+02],
                                                          [0.00000000e+00, 1.15803374e+03, 5.40000000e+02],
                                                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
                                 'DistCoeff': np.asarray([[0., 0., 0., 0., 0.]]),
                                 'ImageSize': [1920, 1080],
                                 'ReprojectionError': 0.0}
        self.tf = self.calibration_dict['CameraExtrinsicMat']
        self.intrinsic = self.calibration_dict['CameraMat']
        self.intrinsic_inv = np.linalg.inv(self.intrinsic)
        self.Y_CAM = 1.7
        self.desired_speed = 1

    def convert_waypoint_pixels_to_waypoints(self, path_pixels):
        if path_pixels is not None:
            path_pixels = np.insert(path_pixels, 2, np.ones(len(path_pixels)), axis=1)
            homolog_coords = self.intrinsic_inv.dot(path_pixels.T)
            path = np.multiply(homolog_coords, (self.Y_CAM / homolog_coords[1, :]))
            path = np.vstack((path[2], path[0], self.desired_speed * np.ones(len(path[0]))))

            path_msg = path.T.flatten().tolist()

            waypoints = Waypoints()
            waypoints.waypoint_count = len(path[0])
            waypoints.waypoints = path_msg
            waypoints.header.stamp = self.get_clock().now().to_msg()
            return waypoints
        else:
            return None

    def timer_callback(self):
        # self.quadratic_callback(None)
        pass

    def waypoint_callback(self, waypoint_pixels_msg):
        forward_path_pixels = convert_msg_to_waypoint(waypoint_pixels_msg, "forward")
        left_path_pixels = convert_msg_to_waypoint(waypoint_pixels_msg, "left")
        right_path_pixels = convert_msg_to_waypoint(waypoint_pixels_msg, "right")

        forward_path = self.convert_waypoint_pixels_to_waypoints(forward_path_pixels)
        left_path = self.convert_waypoint_pixels_to_waypoints(left_path_pixels)
        right_path = self.convert_waypoint_pixels_to_waypoints(right_path_pixels)

        waypoints_all_msg = WaypointsList()

        waypoints_all_msg.left = left_path
        waypoints_all_msg.right = right_path
        waypoints_all_msg.forward = forward_path

        self.waypoint_publisher.publish(waypoints_all_msg)
        self.visualization_waypoint_publisher.publish(waypoints_all_msg)
        # marker_idx = 0
        # marker_array = MarkerArray()
        # for point in path.T:
        #     marker = Marker()
        #     marker.header.frame_id = "velodyne"
        #     marker.id = marker_idx
        #     marker.type = Marker.SPHERE
        #     marker.action = Marker.ADD
        #     marker.pose.position.x = point[0]
        #     marker.pose.position.y = -point[1]
        #     marker.pose.position.z = -1.7
            
        #     marker.scale.x = 0.3
        #     marker.scale.y = 0.3
        #     marker.scale.z = 0.3
            
        #     marker.color.r = 0.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.color.a = 1.0
        #     marker_idx+=1
        #     marker_array.markers.append(marker)
        # self.visualization_waypoint_publisher.publish(marker_array)
            


def main(args=None):
    rclpy.init(args=args)
    lane_waypoint = LaneWaypointNode()

    rclpy.spin(lane_waypoint)
    lane_waypoint.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
