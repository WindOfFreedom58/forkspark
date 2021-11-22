#!/usr/bin/python

import message_filters
import rospy
from LocalPlanner import LocalPlanner

from spark_msgs.msg import Pose, Waypoints
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from tf import transformations
import numpy as np

INTERP_DISTANCE_RES = 0.01  # distance between interpolated points


def yaw_from_quat_msg(quat_msg):
    return transformations.euler_from_quaternion(
        [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w])[2]


def rotation_matrix(theta):
    return np.asarray([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


class CubicSpiralPlannerNode:
    def __init__(self, waypoint_publisher_name="/parking_waypoints"):
        num_paths = 7
        path_offset = 0.2  # meters

        # Local Planner object
        self.lp = LocalPlanner(num_paths, path_offset)

        # Subscribers
        self.poseSubscriber = rospy.Subscriber(
            "/odom", Odometry, self.update_pose, queue_size=5)
        self.goalStateSubscriber = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.plan_goal_state, queue_size=5)

        # Publisher
        self.local_waypoints_publisher = rospy.Publisher(
            waypoint_publisher_name, Waypoints, queue_size=1)
        self.parking_planned_publisher = rospy.Publisher(
            "/parking_ok", Empty, queue_size=5)
        # State variables
        self.PLANNING = False
        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0

        # Constants
        rospy.loginfo("CubicSpiralPlanner node initiated!")

    def update_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.yaw = transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]

    def plan_goal_state(self, msg):
        rospy.loginfo("x: {}, y: {}, yaw: {}".format(self.x, self.y, self.yaw))
        while(self.x is None or self.y is None or self.yaw == 0.0): pass
        self.speed = 9.0
        if not self.PLANNING:
            self.PLANNING = True
            rospy.loginfo("Planning for given goal state..")

            rot_mat = rotation_matrix(-self.yaw)  # np.pi/2 - self.yaw)

            goal_state = np.asarray(
                [msg.pose.position.x - self.x, msg.pose.position.y - self.y])
            goal_state_converted = np.hstack(
                (rot_mat.dot(goal_state), [yaw_from_quat_msg(msg.pose.orientation)-self.yaw]))
            path, path_validity = self.lp.plan_paths([goal_state_converted])
            if path_validity[0] is True:
                rospy.loginfo("Path is succesfully planned!")
                path_waypoints = np.vstack((path[0][0], path[0][1]))
                path_waypoints = rotation_matrix(self.yaw).dot(path_waypoints)
                velocity_profiled_waypoints = self.lp._velocity_planner.decelerate_profile(path_waypoints, self.speed)
                path_waypoints = np.transpose(path_waypoints)
                self.parking_planned_publisher.publish(Empty())

                waypoints_msg = Waypoints()
                local_path = Path()

                for waypoint in velocity_profiled_waypoints:
                    coord = np.asarray(
                        [waypoint[0], waypoint[1]])
                    coord_rotated = rotation_matrix(0).dot(coord)
                    waypoint_msg = PoseStamped()
                    waypoint_msg.pose.position.x = coord_rotated[0] + self.x
                    waypoint_msg.pose.position.y = coord_rotated[1] + self.y
                    local_path.poses.append(waypoint_msg)
                    waypoints_msg.speeds.append(waypoint[2])

                local_path.header.stamp = rospy.Time.now()
                local_path.header.frame_id = "odom"

                waypoints_msg.path = local_path

                self.local_waypoints_publisher.publish(waypoints_msg)
            self.PLANNING = False


def main(args=None):
    rospy.init_node("CubicSpiralPlanner")
    cubic_spiral_planner = CubicSpiralPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
