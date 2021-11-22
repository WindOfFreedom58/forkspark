#include "WaypointPointCloudNode.hpp"

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

// TODO: add a service for changing waypoints
WaypointPointCloudNode::WaypointPointCloudNode() : node(), cloud_processor() {
  pc2_subscription_ = node.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 1, &WaypointPointCloudNode::pcl_callback, this);
  behaviour_subscriber = node.subscribe<spark_msgs::BehaviourState>(
      "/behaviour_planner", 1, &WaypointPointCloudNode::update_behaviour, this);
  pc2_publisher_left =
      node.advertise<sensor_msgs::PointCloud2>("/waypoints_left", 5);
  pc2_publisher_right =
      node.advertise<sensor_msgs::PointCloud2>("/waypoints_right", 5);
  pc2_test_publisher =
      node.advertise<sensor_msgs::PointCloud2>("/pc2_clusters", 5);
  pc2_cleared = node.advertise<sensor_msgs::PointCloud2>("/pc2_cleared", 5);
  lateral_error_publisher =
      node.advertise<std_msgs::Float64>("/barrier_error", 5);

  std::string waypoint_topic_name;

  if (node.getParam("/waypoint_topic", waypoint_topic_name)) {
    lidar_waypoint_publisher =
        node.advertise<spark_msgs::Waypoints>(waypoint_topic_name, 5);
  } else {
    ROS_ERROR("waypoint topic param has not been set!");
    ros::shutdown();
  }

  if (node.getParam("/behaviour_states", behaviour_states)) {
  } else {
    ROS_ERROR("behaviour states param has not been set!");
    ros::shutdown();
  }

  kill_switch_publisher = node.advertise<sensor_msgs::Joy>("/joy", 5);
  ROS_INFO("PointCloudNode initiated!");
}

Eigen::VectorXd fitQuadraticToWaypoints(const PointCloudPtr waypoints) {
  Eigen::VectorXd xvals, yvals;
  xvals.resize((long)waypoints->size());
  yvals.resize((long)waypoints->size());
  for (int i = 0; i < waypoints->points.size(); i++) {
    xvals(i) = waypoints->points.at(i).x;
    yvals(i) = waypoints->points.at(i).y;
  }
  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 2);
  return coeffs;
}

Eigen::VectorXd WaypointPointCloudNode::getWaypointCoeffs(Cluster &cluster) {
  PointCloudPtr barrier_points(new PointCloud);

  for (auto &i : cluster.points) {
    barrier_points->push_back(i);
  }

  if (barrier_points->size() < MIN_BARRIER_POINTS)
    throw std::runtime_error(
        "There are not enough points to generate waypoints");

  return fitQuadraticToWaypoints(barrier_points);
}

PointCloudPtr WaypointPointCloudNode::getWaypoints(
    const Eigen::VectorXd &coeffs) {
  PointCloudPtr waypoints(new PointCloud);
  for (int x = 1; x < (waypoint_generation_distance / space); x++) {
    pcl::PointXYZI point;
    point.x = x * space;
    point.y = polyeval(coeffs, point.x);
    waypoints->push_back(point);
  }

  return waypoints;
}

void publishPointCloud(PointCloudPtr pointCloud, ros::Publisher &publisher) {
  auto pc_2_msg = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*pointCloud, pc_2_msg);

  pc_2_msg.header.frame_id = "velodyne";
  pc_2_msg.header.stamp = ros::Time::now();

  publisher.publish(pc_2_msg);
}

/**
std::pair<int,int> decideRoads(double left_slope, double right_slope){
   int left_turn = 0;
   int right_turn = 0;
  if((left_slope < -0.1 || left_slope > 0.1) && (right_slope < -0.1 ||
right_slope > 0.1)){ ROS_INFO("TWO AVAILABLE ROAD");
  }
  if(abs(left_slope) > abs(right_slope) && (left_slope < -0.15 || left_slope >
0.15)){//Aslında sağ? left_turn = 1; ROS_INFO("TO LEFT: %.4f", left_slope);
    ROS_INFO("-----");
  }
  if(abs(left_slope) < abs(right_slope) && (right_slope < -0.15 || right_slope >
0.15)){ //Aslında sol? right_turn = 1; ROS_INFO("TO RIGHT %.4f", right_slope);
    ROS_INFO("-----");
    ROS_INFO("-----");
    ROS_INFO("-----");
  }
} */

// TODO: Move these to a new class PIDError, use inheritance
double meanLateralDistanceFromWaypoints(PointCloudPtr waypoints,
                                        int lookahead_index) {
  return waypoints->at(lookahead_index).y;
}

double WaypointPointCloudNode::distanceToBarrierError(double dist) {
  // ROS_INFO("Distance to wall: %.4f", dist);
  // ROS_INFO("Center distance: %.4f", center_distance);
  // TODO: Implement setting distance at the beginning
  if (center_distance == -1.0)
    throw std::runtime_error("Center distance cannot be determined yet!");
  double error = (dist - SIDE * center_distance);
  if (std::abs(error) < 0.02)  // less than 2 cm
    return 0;
  else
    return -SIDE * error;
}

int WaypointPointCloudNode::publishWaypoints(PointCloudPtr waypoints,
                                             bool is_left) {
  if (SIDE == 0) return -1;
  double offset = is_left ? center_distance : -center_distance;
  spark_msgs::Waypoints waypoints_msg;
  nav_msgs::Path path;
  std::vector<double> speeds;
  for (auto &point : waypoints->points) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y - offset;
    path.poses.push_back(pose);
    speeds.push_back(2.0);
  }
  path.header.frame_id = "velodyne";
  path.header.stamp = ros::Time::now();

  waypoints_msg.path = path;
  waypoints_msg.speeds = speeds;

  if (std::abs(waypoints_msg.path.poses[0].pose.position.y) > 3) return -1;

  lidar_waypoint_publisher.publish(waypoints_msg);
  return 1;
}

void WaypointPointCloudNode::pcl_callback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  PointCloudPtr point_cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *point_cloud);

  auto t1 = clock.now();
  PointCloudPtr cleared_pcloud = cloud_processor.clear_point_cloud(point_cloud);
  auto t2 = clock.now();
  // ROS_INFO("Clearing pcloud takes: %d milliseconds",
  //          std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));

  if (cleared_pcloud->empty()) return;

  publishPointCloud(cleared_pcloud, pc2_cleared);

  t1 = clock.now();
  std::vector<pcl::PointIndices> clusterIndices =
      cloud_processor.euclidean_clustering(cleared_pcloud, 0.5f, 60, 30000)
          .second;
  t2 = clock.now();
  // ROS_INFO("Clustering cloud takes: %d milliseconds",
  //          std::chrono::duration_cast<std::chrono::millisesconds>(t2 - t1));

  ROS_INFO("%ld clusters found", clusterIndices.size());
  // ROS_INFO("SIDE: %d ", SIDE);

  std::vector<Cluster> clusters;
  std::pair<Cluster, Cluster> selectedClusters;

  PointCloudPtr clusterVis(
      new PointCloud);  // point cloud object for visualization
  float intensity = 0.0;
  for (const auto &indices : clusterIndices) {
    std::vector<pcl::PointXYZI> cluster_points;

    double sum_y = 0.0;
    double sum_euclidean = 0.0;
    t1 = clock.now();
    for (const auto idx : indices.indices) {
      pcl::PointXYZI point = (*cleared_pcloud)[idx];
      point.intensity = intensity;
      clusterVis->push_back(point);
      cluster_points.push_back((*cleared_pcloud)[idx]);
      sum_y += (*cleared_pcloud)[idx].y;
      sum_euclidean += std::sqrt(std::pow((*cleared_pcloud)[idx].x, 2) +
                                 std::pow((*cleared_pcloud)[idx].y, 2));
    }
    t2 = clock.now();
    // ROS_INFO("Adding cluster indices to cloud takes: %d milliseconds",
    //          std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));

    Cluster cluster =
        Cluster(sum_y / (int)cluster_points.size(),
                sum_euclidean / (int)cluster_points.size(), cluster_points);
    clusters.push_back(cluster);
    intensity += 100;
  }
  publishPointCloud(clusterVis, pc2_test_publisher);

  if (clusters.size() == 1) {
    if (SIDE == 1)
      selectedClusters.first = clusters[0];
    else if (SIDE == -1)
      selectedClusters.second = clusters[0];
  } else if (clusters.size() > 1) {
    // sorting clusters according to their euclidean distance
    std::sort(clusters.begin(), clusters.end(), [](Cluster c1, Cluster c2) {
      return c1.euclideanDist < c2.euclideanDist;
    });
    if (clusters[0].lateralDist > clusters[1].lateralDist) {
      selectedClusters.first = clusters[0];
      selectedClusters.second = clusters[1];
    } else {
      selectedClusters.first = clusters[1];
      selectedClusters.second = clusters[0];
    }
  }

  double dist_left = 1e10;
  double dist_right = 1e10;
  double left_curvature = 0.0;
  double right_curvature = 0.0;
  PointCloudPtr waypoints_left;
  PointCloudPtr waypoints_right;

  if (!selectedClusters.first.points.empty()) {
    try {
      Eigen::VectorXd left_coeffs = getWaypointCoeffs(selectedClusters.first);
      // ROS_INFO("Left waypoints coeffs: %.2f %.2f %.2f", left_coeffs[0],
      //          left_coeffs[1], left_coeffs[2]);
      left_curvature = get_curvature(left_coeffs, lookahead);

      //      ROS_INFO("Left curvature: %.6f", left_curvature);

      // left_slope = get_slope(left_coeffs, lookahead);
      // ROS_INFO("Left slope: %.6f", left_slope);
      waypoints_left = getWaypoints(left_coeffs);
      dist_left = meanLateralDistanceFromWaypoints(waypoints_left,
                                                   (int)lookahead / space);
      if (dist_left < 0)
        throw std::runtime_error("Left waypoints should be on the left");
      publishPointCloud(waypoints_left, pc2_publisher_left);
      // ROS_INFO("Left distance: %f ", dist_left);
    } catch (std::exception &e) {
      ROS_ERROR("Failed to generate left barrier waypoints!");
    }
  }

  if (!selectedClusters.second.points.empty()) {
    try {
      // ROS_INFO("Right waypoints coming!");
      Eigen::VectorXd right_coeffs = getWaypointCoeffs(selectedClusters.second);
      // ROS_INFO("Right waypoints coeffs: %.2f %.2f %.2f", right_coeffs[0],
      //          right_coeffs[1], right_coeffs[2]);
      right_curvature = get_curvature(right_coeffs, lookahead);
      //      ROS_INFO("Right curvature: %.6f", right_curvature);

      // right_slope = get_slope(right_coeffs, lookahead);
      // ROS_INFO("Right slope: %.6f", right_slope);

      waypoints_right = getWaypoints(right_coeffs);
      publishPointCloud(waypoints_right, pc2_publisher_right);

      dist_right = meanLateralDistanceFromWaypoints(waypoints_right,
                                                    (int)lookahead / space);
      // ROS_INFO("Left distance: %f ", dist_left);
    } catch (std::exception &e) {
      ROS_ERROR("Failed to generate right barrier waypoints!");
    }
  }
  if (clusterIndices.size() > 1) {
    if (center_distance < 0) {
      if (dist_left != 1e10 && dist_right != 1e10) {
        center_distance = (std::abs(dist_left) + std::abs(dist_right)) / 2;
        // ROS_INFO("Center distance found: %f", center_distance);
      } else
        return;
    }
  }

  double error = 0.0;
  int waypoints_ok = 0;
  if (SIDE == 1 && dist_left != 1e10) {
    try {
      error = distanceToBarrierError(dist_left);
    } catch (std::runtime_error &e) {
      ROS_WARN("%s", e.what());
    }
    if (waypoints_left != nullptr)
      waypoints_ok = publishWaypoints(waypoints_left, true);
  } else if (SIDE == -1 && dist_right != 1e10) {
    try {
      error = -distanceToBarrierError(dist_right);
    } catch (std::runtime_error &e) {
      ROS_WARN("%s", e.what());
    }
    if (waypoints_right != nullptr)
      waypoints_ok = publishWaypoints(waypoints_right, false);
  } else {
    return;
  }
  std_msgs::Float64 error_msg;
  if (waypoints_ok != 1)
    error_msg.data =
        previous_error;  // if finding something wrong, give previous error
  else {
    error_msg.data = error;
    previous_error = error;
  }
  // ROS_INFO("Published error: %.4f", error);
  // If switched to park, do nothing..
  if (SIDE == 1 || SIDE == -1)
    lateral_error_publisher.publish(error_msg);
  else if (SIDE == 2) {
    // IF going blind, give zero error
  }
}

void WaypointPointCloudNode::update_behaviour(
    const spark_msgs::BehaviourState state) {
  if (state.state == behaviour_states["left_state"])
    SIDE = 1;
  else if (state.state == behaviour_states["park_state"])
    SIDE = 0;
  else if (state.state == behaviour_states["right_state"])
    SIDE = -1;
  else if (state.state == behaviour_states["straight_state"]) {
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "WaypointPointCloudNode");
  auto *pointCloudNode = new WaypointPointCloudNode();
  ros::spin();
  return 0;
}
