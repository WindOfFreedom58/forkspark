#!/usr/bin/python
import csv
from enum import Enum
import os
import rospy

from math import radians, cos, sin, sqrt, atan2

from spark_msgs.srv import gpsPoint, gpsPointResponse
from spark_msgs.msg import Waypoints
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from Dijkstra import Dijkstra

R = 6371*1e3  # Earth's radius


class NodeTypes(Enum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2
    INTERSECTION = 3
    END = 4


class Node:
    def __init__(self, lat, lon, id):
        self.latitude = lat
        self.longitude = lon
        self.id = id

        self.neighbors = [None, None, None]
        self.distances = [None, None, None]
        self.multiple_inverse_neighbors = False

        self.type = None

        self.neighbor_indexes = {
            0: "STRAIGHT",
            1: "LEFT",
            2: "RIGHT"
        }

    def __str__(self):
        string = f"w{self.id} : {self.type}"
        return string

    def print_neighbors(self):
        for i, neighbor in enumerate(self.neighbors):
            if neighbor is not None:
                print(
                    f"w{neighbor.id} is {self.neighbor_indexes[i]} neighbor.")


def haversine_transform(lat0, long0, lat, long):
    dLon = radians(long) - radians(long0)
    latAverage = (lat + lat0)/2
    a = cos(radians(latAverage))*cos(radians(latAverage)) * \
        sin(dLon / 2)*sin(dLon/2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1-a))

    x = dist if long > long0 else -dist

    dLat = radians(lat - lat0)
    a = pow(sin(dLat/2), 2)
    dist = R * 2 * atan2(sqrt(a), sqrt(1-a))
    y = dist if lat > lat0 else -dist

    return x, y


def getDistance(node1, node2):
    dLat = radians(node2.latitude) - radians(node1.latitude)
    dLon = radians(node2.longitude) - radians(node1.longitude)
    a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(node1.latitude)) * \
        cos(radians(node2.latitude)) * sin(dLon / 2) * sin(dLon / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


def get_odom_coordinate(lat_source, lon_source, lat_target, lon_target):
    """
    Returns stamped pose to be added to a path message in odom frame
    """
    x, y = haversine_transform(lat_source, lon_source, lat_target, lon_target)
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    return pose_msg


MAX_TOLERABLE_DISTANCE = 3.0
STRAIGHT_SPEED = 5.0
TURNING_SPEED = 3.0


class SparkMap:

    def __init__(self, waypoints_path_csv, graph_path_txt):
        self.nodes = []
        self.sourcePoint = None
        self.initial_position = None

        self.map_publisher = rospy.Publisher(
            "/route_network", Path, queue_size=1, latch=True)
        try:
            self.load_nodes(waypoints_path_csv)
            self.nodes.sort(key=lambda x: x.id)
            self.create_neighbors(graph_path_txt)
            self.nodes[0].type = NodeTypes.STRAIGHT
            
        except Exception as e:
            rospy.logerr(e)


        self.gps_subscriber = rospy.Subscriber(
            "/gps", NavSatFix, self.init_gps)

        self.gps_waypoint_publisher = rospy.Publisher(
            "/gps_waypoints", Waypoints, queue_size=5)

        self.dijkstra = Dijkstra()
        self.make_plan = rospy.Service('make_plan', gpsPoint, self.make_plan)

    def publishRouteNetwork(self):
        waypoints_msg = self.get_waypoints_msg_from_nodes(
            list(range(len(self.nodes))))
        rospy.loginfo(waypoints_msg.path)
        self.map_publisher.publish(waypoints_msg.path)

    def init_gps(self, gps_msg):
        # rospy.loginfo("GPS Waypoint Publisher: Waiting for subscriber..")
        # while(self.gps_waypoint_publisher.get_num_connections() == 0):
        #     pass
        # rospy.loginfo(
        #     "GPS Waypoint Publisher: A subscriber is attached to waypoint publisher, proceeding..")

        if self.initial_position is None:
            self.initial_position = gps_msg
            self.sourcePoint = self.get_closest_node(
                gps_msg.latitude, gps_msg.longitude)
            self.publishRouteNetwork()

    def make_plan(self, req):
        if self.sourcePoint is None:
            return gpsPointResponse(gpsPointResponse.NOT_INIT)
        target = self.get_closest_node(
            req.position.latitude, req.position.longitude)
        if target is None:
            return gpsPointResponse(gpsPointResponse.TARGET_IS_OUT_OF_WAY)
        try:
            path = self.dijkstra.findShortestPath(
                self.get_graph(), self.sourcePoint, target)
            self.publishGpsWaypoints(path)
        except Exception as e:
            rospy.logerr(e)
            return gpsPointResponse(gpsPointResponse.PLAN_FAILED)

    def get_waypoints_msg_from_nodes(self, path):
        waypoints_msg = Waypoints()
        path_msg = Path()
        speeds = []

        path_msg.header.frame_id = "odom"

        for node_idx in path:
            pose_msg = self.convert_node_to_odom_path(
                self.nodes[node_idx])
            path_msg.poses.append(path_msg)
            if(self.nodes[node_idx].type == NodeTypes.STRAIGHT):
                speeds.append(STRAIGHT_SPEED)
            elif(self.nodes[node_idx].type == NodeTypes.END):
                speeds.append(0.0)
            else:
                speeds.append(TURNING_SPEED)
        waypoints_msg.path = path_msg
        waypoints_msg.speeds = speeds

        return waypoints_msg

    def publishGpsWaypoints(self, path):
        self.gps_waypoint_publisher.publish(self.get_waypoints_msg_from_nodes(path))

    def convert_node_to_odom_path(self, node):
        if self.initial_position is None:
            return
        return get_odom_coordinate(self.initial_position.latitude, self.initial_position.longitude, node.latitude, node.longitude)

    def get_closest_node(self, latitude, longitude):
        start_point = Node(latitude, longitude, -1)
        min_node = 0
        min_distance = float("inf")
        for i in range(len(self.nodes)):
            distance = getDistance(self.nodes[i], start_point)
            if distance < min_distance:
                min_distance = distance
                min_node = i
        if min_distance > MAX_TOLERABLE_DISTANCE:
            return None
        return min_node

    def get_graph(self):
        return [parent.distances for parent in self.nodes]

    def load_nodes(self, filepath):
        with open(filepath) as f:
            data = f.readlines()[1:]

            for line in data:
                line = line.strip().split(",")

                x = float(line[0])
                y = float(line[1])
                node_id = int(line[2].replace("w", ""))

                node = Node(x, y, node_id)

                self.nodes.append(node)

    def create_neighbors(self, filepath):
        with open(filepath) as f:
            data = f.readlines()[1:]

            for line in data:
                line = line.strip().split(":")

                node_id = int(line[0])
                #print(f"{node_id} is being processed")
                neighbors = line[1].split(",")
                #print(f"Given neighbors: {neighbors}")

                if len(neighbors) > 1:
                    self.nodes[node_id].type = NodeTypes.INTERSECTION
                    self.nodes[node_id].multiple_inverse_neighbors = True
                    #print("Assigned type: intersection")

                if len(neighbors) == 0:
                    self.nodes[node_id].type = NodeTypes.END
                    #print("Assigned type: END")

                for i, neighbor in enumerate(neighbors):
                    try:
                        neighbor_id = int(neighbor)
                    except:
                        continue

                    if i == 0 and not self.nodes[neighbor_id].multiple_inverse_neighbors:
                        self.nodes[neighbor_id].type = NodeTypes.STRAIGHT
                        #print(f"{neighbor_id} is assigned: straight")

                    elif i == 1 or i == 2:
                        self.nodes[neighbor_id].type = NodeTypes.INTERSECTION
                        #print(f"{neighbor_id} is assigned: left")

                    self.nodes[node_id].neighbors.insert(
                        i, self.nodes[neighbor_id])
                    self.nodes[node_id].distances.insert(i, getDistance(
                        self.nodes[node_id], self.nodes[neighbor_id]))

        for i in range(len(self.nodes)):
            if self.nodes[i].type is None:
                #print(f"w{self.nodes[i].id} is assigned: end")
                self.nodes[i].type = NodeTypes.END


def main():
    root_dir = "/home/enes/Desktop/spark_real/spark/src/waypoints/waypoint_files"
    waypoints_path_csv = os.path.join(root_dir, "parkur.csv")
    graph_path_txt = os.path.join(root_dir, "parkur_graph.txt")

    rospy.init_node("SparkGlobalPlanner")
    spark_map = SparkMap(waypoints_path_csv, graph_path_txt)
    rospy.spin()
    # for i, node in enumerate(map.nodes):
    #     neighbors = node.neighbors

    #     print(node)
    #     print("Neighbors:")
    #     for neighbor in neighbors:
    #         print(neighbor, end=" ")
    #     print()
    #     print("*********")


if __name__ == "__main__":
    main()
