import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from lane_detector.lane_detector.make_predictions import load_model
import numpy as np
import cv2
import torch
import time

class LaneDetectorNode(Node):

    def __init__(self):
        super().__init__("RightLaneDetector")

        self.subscription_right = self.create_subscription(
            CompressedImage,
            "/simulator/camera_node/right_image/compressed",
            self.right_callback,
            1)

        self.publisher_right = self.create_publisher(
            CompressedImage,
            "/lane_detection/right_view",
            1
        )

        self.bridge = CvBridge()

        self.model_path = "/home/yigithan/lane_follower/dev_ws/src/lane_detector/lane_detector/models/final_9_batch_8.model"

        self.model_object, self.device = load_model(self.model_path)


    def right_callback(self, msg):
        tic = time.time()
        image = np.array(self.bridge.compressed_imgmsg_to_cv2(msg))

        gt_img_org = np.copy(image)

        gt_image = cv2.resize(gt_img_org, dsize=(512, 256), interpolation=cv2.INTER_LINEAR)
        gt_image = gt_image / 127.5 - 1.0
        gt_image = torch.tensor(gt_image, dtype=torch.float)
        gt_image = np.transpose(gt_image, (2, 0, 1))

        binary, _ = self.model_object(gt_image.unsqueeze(0).cuda())
        binary_img = torch.argmax(binary, dim=1).squeeze().cpu().numpy()

        #binary_img = cv2.resize(binary_img, dsize=(image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)

        binary_compressed = self.bridge.cv2_to_compressed_imgmsg(binary_img.astype("float32"))

        self.publisher_right.publish(binary_compressed)

        toc = time.time()

        #print(f"Fps of right callback = {1/(toc-tic)}")


def main(args=None):
    rclpy.init(args=args)

    subscriber = LaneDetectorNode()

    rclpy.spin(subscriber)

    subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()