#include "lidar_to_cam_node.hpp"

int previous_unit_digit = -2;
StampedCloud* stamped_cloud_cache = NULL;

void load_tf(cv::Mat& tf, geometry_msgs::TransformStamped tf_message) {
  tf.at<double>(0, 0) = tf_message.transform.translation.x;
  tf.at<double>(1, 0) = tf_message.transform.translation.y;
  tf.at<double>(2, 0) = tf_message.transform.translation.z;
}

void parse_camera_intrinsic(cv::Mat& K, std::vector<float> camera_intrinsic) {
  ROS_INFO("Intrinsic camera matrix loaded..");
  K.at<double>(0, 0) = camera_intrinsic[0];
  K.at<double>(0, 1) = camera_intrinsic[1];
  K.at<double>(0, 2) = camera_intrinsic[2];
  K.at<double>(1, 0) = camera_intrinsic[3];
  K.at<double>(1, 1) = camera_intrinsic[4];
  K.at<double>(1, 2) = camera_intrinsic[5];
  K.at<double>(2, 0) = camera_intrinsic[6];
  K.at<double>(2, 1) = camera_intrinsic[7];
  K.at<double>(2, 2) = camera_intrinsic[8];
}

void parse_distortion_coefficients(cv::Mat& distCoeffs,
                                   std::vector<float> distortion_coeffs) {
  ROS_INFO("Distortion coeffients loaded..");
  distCoeffs.at<double>(0, 0) = distortion_coeffs[0];
  distCoeffs.at<double>(0, 1) = distortion_coeffs[1];
  distCoeffs.at<double>(0, 2) = distortion_coeffs[2];
  distCoeffs.at<double>(0, 3) = distortion_coeffs[2];
  distCoeffs.at<double>(0, 4) = distortion_coeffs[2];
}

StampedCloud::StampedCloud(double time_stamp,
                           std::vector<cv::Point> projected_points,
                           std::vector<cv::Point2d> lidar_points) {
  this->time_stamp = time_stamp;
  this->projected_points = projected_points;
  this->lidar_points = lidar_points;
}

double StampedCloud::getTimeStamp() { return time_stamp; }

std::vector<cv::Point> StampedCloud::getProjectedPoints() {
  return projected_points;
}

std::vector<double> StampedCloud::getDistancesX() {
  std::vector<double> distances;
  for (auto& point : lidar_points) {
    distances.push_back(point.x);
  }
  return distances;
}

std::vector<double> StampedCloud::getDistancesY() {
  std::vector<double> distances;
  for (auto& point : lidar_points) {
    distances.push_back(point.y);
  }
  return distances;
}

StampedBoxes::StampedBoxes(double time_stamp,
                           darknet_ros_msgs::BoundingBoxes bounding_boxes) {
  this->time_stamp = time_stamp;
  this->bounding_boxes = bounding_boxes;
}

double StampedBoxes::getTimeStamp() { return time_stamp; }

darknet_ros_msgs::BoundingBoxes StampedBoxes::getBoundingBoxes() {
  return bounding_boxes;
}

double convertToDecimal(long number) {
  double decimal = (double)number;
  while (decimal >= 1) {
    decimal /= 10;
  }
  return decimal;
}

void LIDARCamProjectorNode::publishDistances(
    std::map<std::string, cv::Point2d> object_poses) {
  spark_msgs::ObjectDistances object_distances_msg;
  std::map<std::string, cv::Point2d>::iterator it;
  for (it = object_poses.begin(); it != object_poses.end(); it++) {
    if(it->second.x == INFINITY && it->second.y == INFINITY) continue;
    std_msgs::String sign_name;
    sign_name.data = it->first;
    object_distances_msg.classes.push_back(sign_name);
    geometry_msgs::Point pt;
    pt.x = it->second.x;
    pt.y = it->second.y;
    pt.z = 2.0;
    object_distances_msg.poses.push_back(pt);
    ROS_INFO("ID: %d, Pose: (x: %lf, y: %lf)", it->first, pt.x, pt.y);
  }
  object_distances_pub.publish(object_distances_msg);
} 

double getAccurateDistance(std::vector<double> enclosed_distances, bool byAverage) {

  if (byAverage) {
    double sum = 0.0;
    for (double distance : enclosed_distances) {
      sum += distance;
    }
    return sum / enclosed_distances.size();
  }

  const double round_precision_inversed = 100;
  std::map<double, int> rounded_enclosed_distances;

  for (int i = 0; i < enclosed_distances.size(); i++) {
    double rounded_distance =
        round(enclosed_distances.at(i) * round_precision_inversed) /
        round_precision_inversed;
    rounded_enclosed_distances[rounded_distance] += 1;
  }

  std::map<double, int>::iterator it;
  int max_count = 0;
  double most_repetitive = 0.0;
  for (it = rounded_enclosed_distances.begin();
       it != rounded_enclosed_distances.end(); it++) {
    if (it->second > max_count) {
      max_count = it->second;
      most_repetitive = it->first;
    }
  }
  return most_repetitive;
}

// bounding boxlar üst üste çakışırsa problem (ama zaten genelde öndekinin
// uzaklığını almak isteyeceğimiz için problem olmayabilir)
std::map<std::string, cv::Point2d> getObjectDistances(StampedCloud stamped_cloud,
                                            StampedBoxes stamped_boxes) {
  std::map<std::string, cv::Point2d> object_distances;
  auto boxes = stamped_boxes.getBoundingBoxes().bounding_boxes;
  auto points = stamped_cloud.getProjectedPoints();
  auto point_distances = stamped_cloud.getDistancesX();
  auto point_distances_y = stamped_cloud.getDistancesY();

  for (auto box_it = boxes.begin(); box_it != boxes.end(); box_it++) {
    std::vector<double> enclosed_distances{};
    std::vector<double> enclosed_distances_y{};
    for (int i = 0; i < points.size(); i++) {
      auto point = points.at(i);
      if (box_it->xmin < point.x && point.x < box_it->xmax &&
          box_it->ymin < point.y && point.y < box_it->ymax) {
        enclosed_distances.push_back(point_distances.at(i));
        enclosed_distances_y.push_back(point_distances_y.at(i));
      }
    }
    std::cout << "Points Enclosed: " << enclosed_distances.size() << std::endl;
    if(enclosed_distances.size() >= 5){
      cv::Point2d object_pose(getAccurateDistance(enclosed_distances, false),
                            getAccurateDistance(enclosed_distances_y, true));
      object_distances[box_it->Class] = object_pose;
    }else{
      cv::Point2d object_pose(INFINITY, INFINITY); // object is very far
      object_distances[box_it->Class] = object_pose;
    }
  }
  return object_distances;
}

/*
StampedItem::StampedItem(double time_stamp){
  this->time_stamp = time_stamp;
}*/

void LIDARCamProjectorNode::pointCallback(const ImageConstPtr& image,
                                          const PointCloud2ConstPtr& cloud) {
 // ROS_INFO("Image time: %ld.%ld , lidar time: %ld.%ld",
  // image->header.stamp.sec, image->header.stamp.nsec, cloud->header.stamp.sec,
  // cloud->header.stamp.nsec);

  publisher_to_detection.publish(image);

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  PointCloudPtr pc_ptr(new PointCloud);
  try {
    pcl::fromROSMsg(*cloud, *pc_ptr);
  } catch (std::exception& e) {
    ROS_ERROR("pcl exception: %s", e.what());
    return;
  }

  std::vector<cv::Point> projected_pts;
  std::vector<cv::Point2d> lidar_points;
  double time_stamp =
      image->header.stamp.sec + convertToDecimal(image->header.stamp.nsec);

  // ROS_WARN("Image Stamp: %ld.%ld", image->header.stamp.sec,
  // image->header.stamp.nsec);

  cv::Mat projectedImage = projector.projectLidarToCamera(
      cv_ptr->image, pc_ptr, projected_pts, lidar_points);

  auto* stamped_cloud =
      new StampedCloud(time_stamp, projected_pts, lidar_points);
  int current_unit_digit = image->header.stamp.sec % 10;

  if (current_unit_digit == previous_unit_digit + 1 ||
      (current_unit_digit == 0 && previous_unit_digit == 9)) {
    stamped_cloud_cache =
        new StampedCloud(time_stamp, projected_pts, lidar_points);

  } else {
    if (stamped_cloud_cache != NULL &&
        stamped_cloud_cache->getTimeStamp() < time_stamp) {
      // point_buffer.push(*stamped_cloud_cache);
      stamped_cloud_cache = NULL;
    }

    bool can_stay = false;
    if (!box_buffer.empty()) {
      // ROS_INFO("Box Buffer Not Empty");
      // ROS_INFO("Point Cloud Time Stamp: %lf, Bounding Boxes Time Stamp: %lf",
      // stamped_cloud->getTimeStamp(), box_buffer.front().getTimeStamp());
      while (stamped_cloud->getTimeStamp() >
                 box_buffer.front().getTimeStamp() &&
             !box_buffer.empty()) {
        // ROS_INFO("Box Stamped With %lf Deleted Due To The Cloud Stamped With
        // %lf", box_buffer.front().getTimeStamp(),
        // stamped_cloud->getTimeStamp());
        box_buffer.pop();
      }
      if (!box_buffer.empty()) {
        // ROS_INFO("Box Buffer Still Not Empty");
        // ROS_INFO("Point Cloud Time Stamp: %lf, Bounding Boxes Time Stamp:
        // %lf", stamped_cloud->getTimeStamp(),
        // box_buffer.front().getTimeStamp());
        if (box_buffer.front().getTimeStamp() ==
            stamped_cloud->getTimeStamp()) {
          // ROS_INFO("Time Stamps Matched (Point Cloud Side)");
          publishDistances(
              getObjectDistances(*stamped_cloud, box_buffer.front()));
          box_buffer.pop();
        }
      } else {
        can_stay = true;
      }
    } else {
      can_stay = true;
    }
    if (can_stay) {
      // ROS_INFO("Point Cloud Can Stay");
      if (point_buffer.size() == queue_size) {
        // ROS_INFO("Point Cloud Queue Size Reached, Front: %lf, Back: %lf,
        // Placed: %lf", point_buffer.front().getTimeStamp(),
        // point_buffer.back().getTimeStamp(), stamped_cloud->getTimeStamp());
        point_buffer.pop();
      }
      point_buffer.push(*stamped_cloud);
    }

    cv_bridge::CvImage projectedCvImage;
    projectedCvImage.encoding = "bgr8";
    projectedCvImage.image = projectedImage;
    projected_image_pub.publish(projectedCvImage.toImageMsg());
  }

  previous_unit_digit = current_unit_digit;
}

/*
void LIDARCamProjectorNode::processQueues(StampedCloud stamped_cloud,
StampedBoxes stamped_boxes){

}*/

void LIDARCamProjectorNode::boxCallback(
    const darknet_ros_msgs::BoundingBoxes& bounding_boxes) {
  double time_stamp = bounding_boxes.image_header.stamp.sec +
                      convertToDecimal(bounding_boxes.image_header.stamp.nsec);
  // ROS_WARN("Box Stamp: %lf", time_stamp);
  auto* stamped_boxes = new StampedBoxes(time_stamp, bounding_boxes);

  bool can_stay = false;

  if (!point_buffer.empty()) {
    // ROS_INFO("Point Buffer Not Empty");
    // ROS_INFO("Point Cloud Time Stamp: %lf, Bounding Boxes Time Stamp: %lf",
    // point_buffer.front().getTimeStamp(), stamped_boxes->getTimeStamp());
    while (stamped_boxes->getTimeStamp() >
               point_buffer.front().getTimeStamp() &&
           !point_buffer.empty()) {
      // ROS_INFO("Cloud Stamped With %lf Deleted Due To The Box Stamped With
      // %lf", point_buffer.front().getTimeStamp(),
      // stamped_boxes->getTimeStamp());
      point_buffer.pop();
    }
    if (!point_buffer.empty()) {
      // ROS_INFO("Point Buffer Still Not Empty");
      // ROS_INFO("Point Cloud Time Stamp: %lf, Bounding Boxes Time Stamp: %lf",
      // point_buffer.front().getTimeStamp(), stamped_boxes->getTimeStamp());
      if (point_buffer.front().getTimeStamp() ==
          stamped_boxes->getTimeStamp()) {
        // ROS_INFO("Time Stamps Matched (Bounding Box Side)");
        publishDistances(
            getObjectDistances(point_buffer.front(), *stamped_boxes));
        point_buffer.pop();
      }
    } else {
      can_stay = true;
    }
  } else {
    can_stay = true;
  }

  if (can_stay) {
    // ROS_INFO("Box Can Stay");
    if (box_buffer.size() == queue_size) {
      // ROS_INFO("Box Queue Size Reached, Front: %lf, Back: %lf",
      // box_buffer.front(), box_buffer.back());
      box_buffer.pop();
    }
    box_buffer.push(*stamped_boxes);
  }
}

LIDARCamProjectorNode::LIDARCamProjectorNode()
    : node_handle(),
      projector(),
      tfBuffer(),
      point_buffer(),
      box_buffer(),
      tfListener(tfBuffer),
      main_cam_sub(node_handle, "/main_camera/image_raw", 5),
      point_cloud_sub(node_handle, "/velodyne_points", 5),
      sync(SyncPolicy(5), main_cam_sub, point_cloud_sub) {
  std::string lidar_tf;

  geometry_msgs::TransformStamped transformStamped;

  projected_image_pub =
      node_handle.advertise<sensor_msgs::Image>("/projectedImage", 10);
  object_distances_pub = node_handle.advertise<spark_msgs::ObjectDistances>(
      "/object_distances", 5);
  bounding_box_sub =
      node_handle.subscribe("/darknet_ros/bounding_boxes", 10,
                            &LIDARCamProjectorNode::boxCallback, this);
  publisher_to_detection =
      node_handle.advertise<sensor_msgs::Image>("/syncedImage", 10);

  // Camera-LIDAR translation matrix
  // Getting this from tf2
  cv::Mat tf(3, 1, cv::DataType<double>::type);

  // Camera intrinsic matrix
  // Getting this from rosparam
  cv::Mat K(3, 3, cv::DataType<double>::type);

  // Camera distortion matrix
  // Getting this from rosparam
  cv::Mat distCoeffs(1, 5, cv::DataType<double>::type);

  try {
    transformStamped = tfBuffer.lookupTransform(
        "main_camera", "velodyne", ros::Time(0), ros::Duration(1.0));

    load_tf(tf, transformStamped);

    std::vector<float> camera_intrinsic;
    if (!(ros::param::get("/camera_matrix/data", camera_intrinsic))) {
      throw std::runtime_error("'camera_matrix' parameter not defined");
    }

    parse_camera_intrinsic(K, camera_intrinsic);

    std::vector<float> distortion_coeffs;
    if (!(ros::param::get("/distortion_coefficients/data",
                          distortion_coeffs))) {
      throw std::runtime_error(
          "'distortion_coefficients' parameter not defined");
    }

    parse_distortion_coefficients(distCoeffs, distortion_coeffs);

    if(!(ros::param::get("/darknet_ros/yolo_model/detection_classes/names", sign_id_list))){
      throw std::runtime_error("YOLO sign id list has not been set!");
    }

  } catch (const std::exception& ex) {
    ROS_ERROR("%s || shutting down the %s node", ex.what(),
              ros::this_node::getName());
    ros::shutdown();
  }

  // Set dist coeffs before K
  projector.setDistCoeffs(distCoeffs);
  projector.setK(K);
  projector.setTf(tf);
  projector.setR(-0.05);

  sync.registerCallback(
      boost::bind(&LIDARCamProjectorNode::pointCallback, this, _1, _2));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "LIDARCameraProjectorNode");
  auto* projectorNode = new LIDARCamProjectorNode();
  ros::spin();
  return 0;
}
