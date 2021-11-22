#!/usr/bin/python
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import torch
from yolov5.msg import BoundingBox,BoundingBoxes
from light_detection import *

class YoloV5():
    def __init__(self, model_path, classes_dict,sign_confidences):
        rospy.loginfo("YoloV5 node has been started")
        self.sign_confidences = sign_confidences
        self.classes_dict = classes_dict
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  # default

        self.enlargement_thresholds = [2.5, 1.4]
        self.shrink_thresholds = [0.75, 0.9]

        rospy.loginfo("Yolov5 model is loaded")

        rospy.set_param("/darknet_ros/yolo_model/detection_classes/names",
                        list(classes_dict.keys())
                        )

        self.bridge = CvBridge()

        self.image_subscriber = rospy.Subscriber(
            "/syncedImage",
            Image,
            self.image_callback,
            queue_size=15,
        )

        print("Subscribed to %s", "/syncedImage")

        self.bboxes_publisher = rospy.Publisher(
            "/darknet_ros/bounding_boxes",
            BoundingBoxes,
            queue_size=15
        )

    def enlargment_coeff(self, min, max, edge_id, enlarge=True):
        if enlarge:
            return ((max - min) / 2) * (self.enlargement_thresholds[edge_id] - 1)
        return ((max - min) / 2) * (self.shrink_thresholds[edge_id] - 1)

    def image_callback(self, msg):
        bboxes_msg = BoundingBoxes()
        bboxes_msg.header.stamp = msg.header.stamp
        bboxes_list = []

        bboxes_image_header = Header()
        bboxes_image_header.stamp = msg.header.stamp
        bboxes_msg.image_header = bboxes_image_header

        image = self.bridge.imgmsg_to_cv2(img_msg=msg)
        imgs = [image]

        results = self.model(imgs, size=640)

        dataframe = results.pandas().xyxy[0]

        for index,row in dataframe.iterrows():
            if row["confidence"] > self.sign_confidences[row["name"]]:
                bbox = BoundingBox()

                name = row["name"]
                xmin = int(row["xmin"])
                ymin = int(row["ymin"])
                xmax = int(row["xmax"])
                ymax = int(row["ymax"])

                shrink_x = int(self.enlargment_coeff(xmin, xmax, 0, enlarge=False))
                shrink_y = int(self.enlargment_coeff(xmin, xmax, 1, enlarge=False))

                bbox.Class = name if name != "trafik isigi" else determine_light_color(image, xmin - shrink_x, xmax + shrink_x, ymin - shrink_y, ymax + shrink_y)

                bbox.id = self.classes_dict[name]
                bbox.probability = row["confidence"]

                shift_x = self.enlargment_coeff(xmin, xmax, 0)
                shift_y = self.enlargment_coeff(ymin, ymax, 1)

                bbox.xmin = int(xmin - shift_x if name != "park" or name != "park yasak" else xmin)
                bbox.ymin = int(ymin - shift_y if name != "park" or name != "park yasak" else ymin)
                bbox.xmax = int(xmax + shift_x if name != "park" or name != "park yasak" else xmax)
                bbox.ymax = int(ymax + shift_y if name != "park" or name != "park yasak" else ymax)

                bboxes_list.append(bbox)

                image = draw(image, bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.Class)

        bboxes_msg.bounding_boxes = bboxes_list

        self.bboxes_publisher.publish(bboxes_msg)

        cv2.imshow("w", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

if __name__ == "__main__":
    root_path = os.getcwd()
    relative_path = "src/yolov5/models/best.pt"
    #model_path = "/home/ozgurk5/spark/spark/src/yolov5/models/best.pt"
    model_path = "/home/yigithan/cluster_models/operation/epoch14.pt"

    sign_confidences = {
        "dur": 0.8,
        "durak": 0.82,
        "saga donulmez": 0.82,
        "sola donulmez": 0.82,
        "girilmez": 0.8,
        "park yasak": 0.5,
        "park": 0.5,
        "trafik isigi": 0.7,
        "ileriden sola mecburi": 0.4,
        "ilerden saga mecburi": 0.4,
        "ileri ve sag": 0.4,
        "ileri ve sol": 0.4
    }

    classes_dict = {
        "dur": 0,
        "durak": 1,
        "saga donulmez": 2,
        "sola donulmez": 3,
        "girilmez": 4,
        "park yasak": 5,
        "park": 6,
        "trafik isigi": 7,
        "ileriden sola mecburi": 8,
        "ilerden saga mecburi": 9,
        "ileri ve sag": 10,
        "ileri ve sol": 11
    }

    rospy.init_node("YoloV5")
    yolov5_node = YoloV5(model_path, sign_confidences=sign_confidences, classes_dict=classes_dict)
    rospy.spin()
    cv2.destroyAllWindows()


