#!/usr/bin/python
import rospy
import numpy as np
from math import radians, cos, sin, sqrt, atan2

R = 6371*1e3 # Earth's radius

class WaypointConverter:
    def __init__(self, waypoints_path, conversion="haversine", origin_from="file"):
        self.converter_function = None
        if conversion == "spherical":
            self.converter_function = self.spherical_transform

        elif conversion == "haversine":
            self.converter_function = self.haversine_transform

        self.path = waypoints_path

        self.origin_from = origin_from # Set this to parameter_server or file. If file, the node uses
        # file's origin. Otherwise, it gets it from ros parameter server
        self.origin = None

        self.data_indexes = { # Indexes of rows of csv file
             "latitude": 1,
            "longitude": 0,
            "name": 2
        }
        self.waypoints = []
        self.waypoint_names = []
        self.xy_coords = []

        self.parse_file()
        print("Origin: ",self.origin)
        print("Waypoints: ",self.waypoints)
        self.convert_odom_frame()

        print("XY coords = ",self.xy_coords)

    def parse_file(self):
        with open(self.path) as f:
            buffers = f.readlines()

            for i in range(1, len(buffers)):
                string = buffers[i].strip()
                data = string.split(",")

                name = data[self.data_indexes["name"]]
                if name == "odom_origin" and self.origin_from == "file":
                    self.origin = np.array((float(data[self.data_indexes["latitude"]]), float(data[self.data_indexes["longitude"]])))

                else:
                    self.waypoints.append(np.array((float(data[self.data_indexes["latitude"]]), float(data[self.data_indexes["longitude"]]))))
                    self.waypoint_names.append(data[self.data_indexes["name"]])

    def convert_odom_frame(self):
        if self.origin_from == "parameter_server":
            self.origin = np.array(rospy.get_param("odom_origin"))

        for point in self.waypoints:
            self.xy_coords.append(self.converter_function(point[0], point[1], self.origin[0], self.origin[1]))

    def spherical_transform(self, lat, long, lat0, long0):
        # Lat and long should be in degrees
        lat = radians(lat)
        long = radians(long)

        lat0 = radians(lat0)
        long0 = radians(long0)

        x = R*(cos(lat)*cos(long) - cos(lat0)*cos(long0))
        y = R*(cos(lat)*sin(long) - cos(lat0)*sin(long0))

        return x,y

    def haversine_transform(self, lat, long, lat0, long0):
        dLon = radians(long) - radians(long0)
        latAverage = (lat + lat0)/2
        a = cos(radians(latAverage))*cos(radians(latAverage))*sin(dLon / 2)*sin(dLon/2)
        dist = R * 2 * atan2(sqrt(a), sqrt(1-a))

        x =  dist if long > long0 else -dist

        dLat = radians(lat - lat0)
        a = pow(sin(dLat/2), 2)
        dist = R * 2 * atan2(sqrt(a), sqrt(1-a))
        y = dist if lat > lat0 else -dist

        return x,y

if __name__ == "__main__":
    conversion = "haversine"
    path = "../google_earth/Waypoints.csv"
    converter = WaypointConverter(path, conversion)


    for i in range(len(converter.xy_coords)):
        point1 = converter.xy_coords[i]
        name1 = converter.waypoint_names[i]
        norm1 = np.linalg.norm(point1)
        print(f"Distance from origin to {name1} = ", norm1)

        for j in range(i+1, len(converter.xy_coords)):
                point2 = converter.xy_coords[j]
                name2 = converter.waypoint_names[j]
                norm2 = np.linalg.norm(point2)
                dot = np.dot(point1, point2)

                angle = np.arccos(dot/(norm1*norm2))*180/np.pi

                print(f"Angle between {name1} and {name2} = ", angle)


