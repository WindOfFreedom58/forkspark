#!/usr/bin/python

import rospy

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from make_predictions import load_model
import numpy as np
import cv2
import torch
import time


class LaneDetectorNode():

    def __init__(self):
        print("Hello, world!")

        self.bridge = CvBridge()

        self.left_image, self.right_image = None, None

        self.model_path = "/home/enes/Desktop/spark_real/spark_mini/catkin_ws/src/lane_detector/models/final_9_batch_8.model"

        self.model_object, self.device = load_model(self.model_path)

        self.subscription_left = rospy.Subscriber(
            "/left_camera/image_raw",
            Image,
            self.left_callback
        )

        self.publisher_left = rospy.Publisher(
            "/lane_detection/left_view",
            Image,
            queue_size=1
        )

        self.subscription_right = rospy.Subscriber(
            "/right_camera/image_raw",
            Image,
            self.right_callback
        )

        self.publisher_right = rospy.Publisher(
            "/lane_detection/right_view",
            Image,
            queue_size=1
        )

    def right_callback(self, msg):
        image = np.array(self.bridge.imgmsg_to_cv2(msg))
        self.right_image = self.process_image(image)
        self.predict_publish()

    def left_callback(self, msg):
        image = np.array(self.bridge.imgmsg_to_cv2(msg))
        self.left_image = self.process_image(image)
        self.predict_publish()

    def process_image(self, image):
        gt_img_org = np.copy(image)

        gt_image = cv2.resize(gt_img_org, dsize=(
            512, 256), interpolation=cv2.INTER_LINEAR)
        gt_image = gt_image / 127.5 - 1.0
        gt_image = np.transpose(gt_image, (2, 0, 1))
        return gt_image

    def predict_publish(self):
        if self.left_image is not None and self.right_image is not None:
            images_to_predict = torch.tensor(
                [self.left_image, self.right_image], dtype=torch.float, device=torch.device("cuda"))

            binary_prediction = self.model_object(images_to_predict)
            binary_prediction = torch.argmax(
                binary_prediction, dim=1).cpu().numpy().astype("float32")

            left_binary_compressed = self.bridge.cv2_to_imgmsg(
                binary_prediction[0])
            right_binary_compressed = self.bridge.cv2_to_imgmsg(
                binary_prediction[1])

            self.publisher_left.publish(left_binary_compressed)
            self.publisher_right.publish(right_binary_compressed)
            self.left_image, self.right_image = None, None


def main(args=None):
    rospy.init_node("LaneDetector")
    lane_detector = LaneDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
