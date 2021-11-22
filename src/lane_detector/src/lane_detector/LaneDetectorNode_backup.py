#!/usr/bin/python

import rospy

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from make_predictions import load_model
import numpy as np
import cv2
import torch
import time

bridge = CvBridge()

left_image = None
right_image = None

model_path = "/home/enes/Desktop/spark_real/spark_mini/catkin_ws/src/lane_detector/models/final_9_batch_8.model"

model_object, device = load_model(model_path)

subscription_left = None
publisher_left = None
subscription_right = None
publisher_right = None

def right_callback(msg):
    image = np.array(bridge.imgmsg_to_cv2(msg))
    right_image = process_image(image)
    predict_publish()


def left_callback(msg):
    image = np.array(bridge.imgmsg_to_cv2(msg))
    left_image = process_image(image)
    predict_publish()


def process_image(image):
    gt_img_org = np.copy(image)

    gt_image = cv2.resize(gt_img_org, dsize=(
        512, 256), interpolation=cv2.INTER_LINEAR)
    gt_image = gt_image / 127.5 - 1.0
    gt_image = np.transpose(gt_image, (2, 0, 1))
    return gt_image


def predict_publish():
    if left_image is not None and right_image is not None:
        images_to_predict = torch.tensor(
            [left_image, right_image], dtype=torch.float, device=torch.device("cuda"))

        binary_prediction = model_object(images_to_predict)
        binary_prediction = torch.argmax(
            binary_prediction, dim=1).cpu().numpy().astype("float32")

        left_binary_compressed = bridge.cv2_to_imgmsg(
            binary_prediction[0])
        right_binary_compressed = bridge.cv2_to_imgmsg(
            binary_prediction[1])

        publisher_left.publish(left_binary_compressed)
        publisher_right.publish(right_binary_compressed)
        left_image, right_image = None, None


def main(args=None):
    rospy.init_node("LaneDetectorNode")

    subscription_left = rospy.Subscriber(
        "/left_camera/image_raw",
        Image,
        left_callback,
        1
    )

    publisher_left = rospy.Publisher(
        "/lane_detection/left_view",
        Image,
        queue_size=1
    )

    subscription_right = rospy.Subscriber(
        "/right_camera/image_raw",
        Image,
        right_callback,
        queue_size=1
    )

    publisher_right = rospy.Publisher(
        "/lane_detection/right_view",
        Image,
        queue_size=1
    )

    rospy.spin()


if __name__ == "__main__":
    main()
