#!/usr/bin/python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Flipper():

    def __init__(self):
        rospy.loginfo("Flipper node has been initialized")

        self.image_subscriber = rospy.Subscriber(
            "/flipped_camera/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        self.image_publisher = rospy.Publisher(
            "/main_camera/image_raw",
            Image,
            queue_size=1
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(img_msg=msg)

        corrected_image = cv2.flip(image, 1)
        corrected_image = cv2.flip(corrected_image, 0)

        corrected_image_msg = self.bridge.cv2_to_imgmsg(corrected_image)
        
        corrected_image_msg.header = msg.header
        corrected_image_msg.height = msg.height
        corrected_image_msg.width = msg.width
        corrected_image_msg.encoding = msg.encoding
        corrected_image_msg.is_bigendian = msg.is_bigendian
        corrected_image_msg.step = msg.step

        self.image_publisher.publish(corrected_image_msg)

if __name__ == "__main__":

    rospy.init_node("flipNode")
    flip_node = Flipper()
    rospy.spin()