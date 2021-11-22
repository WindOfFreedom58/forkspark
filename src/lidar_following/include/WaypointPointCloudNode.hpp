#ifndef WAYPOINT_POINT_CLOUD_NODE_HPP_
#define WAYPOINT_POINT_CLOUD_NODE_HPP_

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <spark_msgs/BehaviourState.h>
#include <spark_msgs/Waypoints.h>

#include <chrono>
#include <string>

#include "ProcessPointCloud.h"
#include "eigen_utils.h"
#include "ros/ros.h"

class Cluster {
 public:
  Cluster() { lateralDist = 1e10; euclideanDist = 1e10;}
  Cluster(double lateral, double euclidean, std::vector<pcl::PointXYZI> points)
      : lateralDist(lateral), euclideanDist(euclidean), points(points){};
  double lateralDist;
  double euclideanDist;
  std::vector<pcl::PointXYZI> points;
};

class WaypointPointCloudNode {
 public:
  WaypointPointCloudNode();

  ~WaypointPointCloudNode() = default;

 private:
  std::chrono::high_resolution_clock clock;

  int MIN_BARRIER_POINTS = 6;
  // 1 => Left Side || -1 => Right Side
  int SIDE = -1; // default => right

  ros::NodeHandle node;
  ProcessPointClouds cloud_processor;
  PointCloudPtr pcloud;

  ros::Subscriber pc2_subscription_;
  ros::Subscriber behaviour_subscriber;
  ros::Publisher pc2_publisher_left;
  ros::Publisher pc2_publisher_right;
  ros::Publisher pc2_test_publisher;
  ros::Publisher pc2_cleared;
  ros::Publisher lateral_error_publisher;
  ros::Publisher lidar_waypoint_publisher;
  ros::Publisher kill_switch_publisher;

  std::map<std::string, int> behaviour_states;

  double center_distance = 2.50;
  double lookahead = 1.5;
  double waypoint_generation_distance = 4.0;
  double space = 0.5;
  double previous_error = 0.0;

  double distanceToBarrierError(double dist);

  void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void update_behaviour(const spark_msgs::BehaviourState state);

  PointCloudPtr getWaypoints(const Eigen::VectorXd& coeffs);

  Eigen::VectorXd getWaypointCoeffs(Cluster &cluster);

  int publishWaypoints(PointCloudPtr waypoints, bool is_left);
};

#endif