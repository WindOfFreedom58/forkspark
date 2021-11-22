#!/usr/bin/python
import rospy
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix

class NavsatNode():
    def __init__(self):
        rospy.loginfo("Navsat Converter has been activated")

        self.navsat_publisher = rospy.Publisher(
            "/gps",
            NavSatFix,
            queue_size=3
        )

        self.nmea_subscriber = rospy.Subscriber(
            "/gps_device",
            Sentence,
            self.sentence_callback,
            queue_size=3
        )

        self.sign = {
            "N": 1,
            "S": -1,
            "E": 1,
            "W": -1
        }

        self.minutes_to_degrees = 1./60.

    def sentence_callback(self, msg):
        sentence = msg.sentence
        if sentence[0] == "$":
            navsat = NavSatFix()

            sentence = sentence[1:]
            sentence = sentence.split(",")

            lat = sentence[2]
            lat_semicircle = sentence[3]

            lat = self.convert_latitude(lat, lat_semicircle)

            long = sentence[4]
            long_semicircle = sentence[5]

            long = self.convert_longtitude(long, long_semicircle)

            alt = sentence[9]

            #print(f"Latitude: {lat}, Longtitude: {long}, altitude: {alt}")

            navsat.longitude = long
            navsat.latitude = lat
            navsat.altitude = float(alt)

            self.navsat_publisher.publish(navsat)
            #print("Published gps message")

    def convert_latitude(self, lat, semicircle):
        left, right = lat.split(".")

        left = left.zfill(4)
        degrees = int(left[0] + left[1])

        minutes = float(f"{left[2:]}.{right}")

        degrees += minutes*self.minutes_to_degrees

        degrees *= self.sign[semicircle]

        return float(degrees)

    def convert_longtitude(self, long, semicircle):
        left, right = long.split(".")

        left = left.zfill(5)
        degrees = int(left[0] + left[1] + left[2])

        minutes = float(f"{left[3:]}.{right}")

        degrees += minutes*self.minutes_to_degrees

        degrees *= self.sign[semicircle]

        return degrees

if __name__ == "__main__":
    rospy.init_node("GpsConverterNode")

    gps_converter = NavsatNode()
    rospy.spin()

