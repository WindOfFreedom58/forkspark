#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from spark_msgs.msg import LaneCoeffs

import numpy as np

from BinaryToFit import BinaryToFit


class ProcessLaneNode():

    def __init__(self):
        rospy.loginfo("ProcessLaneNode has been started.")

        self.right_publisher = rospy.Publisher(
            "/lane_detection/right_coeffs",
            LaneCoeffs,
            queue_size=1
        )

        self.subscription_right = rospy.Subscriber(
            "/lane_detection/right_view",
            Image,
            self.right_listener_callback)

        self.left_publisher = rospy.Publisher(
            "/lane_detection/left_coeffs",
            LaneCoeffs,
            queue_size=1
        )

        self.subscription_left = rospy.Subscriber(
            '/lane_detection/left_view',
            Image,
            self.left_listener_callback)

        self.image_publisher = rospy.Publisher(
            "/lane_detection/lane_detection_img",
            Image,
            queue_size=2
        )

        self.leftCameraFitter = BinaryToFit(
            show_results=True, camera_type="left", callback=self.publish_image)
        self.rightCameraFitter = BinaryToFit(
            show_results=True, camera_type="right")
        self.bridge = CvBridge()

    def left_listener_callback(self, msg, histogram=True):
        image = np.array(self.bridge.imgmsg_to_cv2(msg))
        fit_params = self.leftCameraFitter.getFitParams(
            image) if histogram else self.line_fitter(image)

        coeffs = LaneCoeffs()
        coeffs.coeffs = fit_params.astype("float32").tolist()
        self.left_publisher.publish(coeffs)

    def right_listener_callback(self, msg, histogram=True):
        image = np.array(self.bridge.imgmsg_to_cv2(msg))
        fit_params = self.rightCameraFitter.getFitParams(
            image) if histogram else self.line_fitter(image)

        coeffs = LaneCoeffs()
        coeffs.coeffs = fit_params.astype("float32").tolist()
        self.right_publisher.publish(coeffs)

    def line_fitter(self, image):

        y, x = np.where(image == 1)
        coeffs = np.polyfit(x, -1*y, 2)

        return coeffs

    def publish_image(self, image):
        msg = self.bridge.cv2_to_imgmsg(image)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne'
        self.image_publisher.publish(msg)


def main(args=None):
    rospy.init_node("ProcessLaneNode")
    processLaneNode = ProcessLaneNode()
    rospy.spin()


if __name__ == "__main__":
    main()
