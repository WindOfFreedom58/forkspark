#include <geometry_msgs/Point.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <spark_msgs/cancelledPoint.h>
#include <spark_msgs/newTarget.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <thread>

#include "Dijkstra.h"
#include "SparkMap.h"
#include "Vertex.h"

class Planner {
 public:
  Planner();
  ~Planner() = default;

  bool isInitializedPosition();
  void initialize();
  int makePlan(double target_latitude, double target_longitude);

 protected:
  void initializePos(double lat, double lon);
  int plan(int sourceID, int targetID);
  int cancelPoinmt(int pointID);

  bool updatePose();
  void setPositionFromGPS(geometry_msgs::Point point,
                          geometry_msgs::Quaternion quaternion);
  void setPositionFromOdom(geometry_msgs::Point point);
  int getNearestPointInPath(double x, double y);

 private:
  Map map;
  Dijkstra dijkstra;

  bool initializedPosition;
  bool initializedRos;
  Vertex source;
  Vertex target;

  bool use_tf;
  std::string map_frame, base_link_frame;
  double interpolation_max_distance;

  /*Publisher*/
  ros::Publisher shortest_path_pub;

  // msgs for shortest path
  nav_msgs::Path path;

  /* Services */
  ros::ServiceServer init_service;
  ros::ServiceServer cancel_point_service;

  // callbacks
  bool initCallback(spark_msgs::newTarget::Request &req,
                    spark_msgs::newTarget::Response &res);
  bool cancelPointCallback(spark_msgs::cancelledPoint::Request &req,
                           spark_msgs::cancelledPoint::Response &res);

  double checkDistance(int node_id, double lat, double lon);
  double checkDistance(int node_id, geometry_msgs::Pose pose);
};