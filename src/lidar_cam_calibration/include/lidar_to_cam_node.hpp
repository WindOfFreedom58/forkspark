#include "lidar_to_cam.hpp"

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <spark_msgs/ObjectDistances.h>

#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "cv_bridge/cv_bridge.h"
#include <queue>

using namespace message_filters;
using namespace sensor_msgs;

typedef sync_policies::ApproximateTime<Image, PointCloud2> SyncPolicy;

class StampedCloud {// : public StampedItem{

  public:
    StampedCloud(double time_stamp, std::vector<cv::Point> projected_points, std::vector<cv::Point2d> lidar_points);
      //:StampedItem(time_stamp){}
    double getTimeStamp();
    std::vector<cv::Point> getProjectedPoints();
    std::vector<double> getDistancesX();
    std::vector<double> getDistancesY();

  private:
    double time_stamp;
    std::vector<cv::Point> projected_points;
    std::vector<cv::Point2d> lidar_points;
};

class StampedBoxes {// : public StampedItem{
  
  public:
    StampedBoxes(double time_stamp, darknet_ros_msgs::BoundingBoxes bounding_boxes);
    double getTimeStamp();
    darknet_ros_msgs::BoundingBoxes getBoundingBoxes();

  private:
    double time_stamp;
    darknet_ros_msgs::BoundingBoxes bounding_boxes;
};

class LIDARCamProjectorNode {
 public:
  LIDARCamProjectorNode();
  ~LIDARCamProjectorNode();
  //void processQueues(StampedCloud stamped_cloud, StampedBoxes stamped_boxes);

 private:
  ros::NodeHandle node_handle;

  ros::Publisher projected_image_pub;
  ros::Subscriber bounding_box_sub;
  ros::Publisher publisher_to_detection;
  ros::Publisher object_distances_pub;

  const int queue_size = 100;
  std::queue<StampedBoxes> box_buffer;
  std::queue<StampedCloud> point_buffer;

  LIDARCamProjector projector;
  message_filters::Subscriber<Image> main_cam_sub;
  message_filters::Subscriber<PointCloud2> point_cloud_sub;
  
  void pointCallback(const ImageConstPtr& image, const PointCloud2ConstPtr& cloud);
  void boxCallback(const darknet_ros_msgs::BoundingBoxes& bounding_boxes);
  void publishDistances(std::map<std::string, cv::Point2d> object_poses);

  std::vector<std::string> sign_id_list;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  Synchronizer<SyncPolicy> sync;
};

/*
class StampedItem {

  public:
    StampedItem(double time_stamp);

  protected:
    double time_stamp;
}*/