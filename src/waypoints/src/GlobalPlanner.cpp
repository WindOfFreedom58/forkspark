#include "GlobalPlanner.h"

Planner::Planner() : map(), dijkstra(), initializedPosition(false) {
  initializedRos = false;
  initialize();
}

void Planner::initialize() {
  if (!initializedRos) {
    // init ros topics and services
    ros::NodeHandle n;

    // source of map
    std::string file = "skuska.osm";
    n.getParam("filepath", file);
    map.setNewMap(file);

    // for point interpolation
    n.param<double>("interpolation_max_distance", interpolation_max_distance,
                    1000);
    map.setInterpolationMaxDistance(interpolation_max_distance);

    std::string topic_name;
    n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path");

    // names of frames
    n.param<std::string>("global_frame", map_frame, "/map");
    n.param<std::string>("robot_base_frame", base_link_frame, "/base_link");
    n.param<bool>("use_tf", use_tf, true);

    // publishers
    shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);

    // services
    init_service =
        n.advertiseService("init_osm_map", &Planner::initCallback, this);
    cancel_point_service =
        n.advertiseService("cancel_point", &Planner::cancelPointCallback, this);

    initializedRos = true;
    ROS_WARN(
        "SPARK Global planner: Waiting for init position, please call init service...");
  }
}

int Planner::makePlan(double target_latitude, double target_longitude) {
  // Reference point is not initialize, please call init service
  if (!initializedPosition) {
    return spark_msgs::newTarget::Response::NOT_INIT;
  }

  updatePose();  // update source point from TF

  // save new target point
  target.setLatitude(target_latitude);
  target.setLongitude(target_longitude);
  target.setId(map.getNearestPoint(target_latitude, target_longitude));
  geometry_msgs::Point cartesianPosition;
  geometry_msgs::Quaternion cartesianOrientation;

  cartesianPosition.x =
      Map::Haversine::getCoordinateX(map.getStartPoint(), target);
  cartesianPosition.y =
      Map::Haversine::getCoordinateY(map.getStartPoint(), target);
  cartesianOrientation = tf::createQuaternionMsgFromYaw(
      Map::Haversine::getBearing(map.getStartPoint(), target));
    
  target.setLocalFramePosition(cartesianPosition);
  target.setLocalFrameOrientation(cartesianOrientation);

  // draw target point
  map.publishPoint(target_latitude, target_longitude,
                   Map::TARGET_POSITION_MARKER);

  // checking distance to the nearest point
  double dist = checkDistance(target.getId(), target.getLatitude(),
                              target.getLongitude());
  if (dist > interpolation_max_distance) {
    ROS_WARN("SPARK global planner: The coordinates is %f m out of the way", dist);

    return spark_msgs::newTarget::Response::TARGET_IS_OUT_OF_WAY;
  }
  int result = plan(source.getId(), target.getId());

  // add end (target) point
  path.poses.push_back(target.getLocalPose());
  shortest_path_pub.publish(path);
  return result;
}

bool Planner::isInitializedPosition() { return initializedPosition; }

void Planner::initializePos(double lat, double lon) {
  map.setStartPoint(lat, lon);

  // Save the position for path planning
  source.setLatitude(lat);
  source.setLongitude(lon);
  source.setId(map.getNearestPoint(lat, lon));
  ROS_INFO("Found the nearest point: %d", source.getId());

  geometry_msgs::Point cartesianPosition;
  
  cartesianPosition.x = 0;
  cartesianPosition.y = 0;

  source.setLocalFramePosition(cartesianPosition);
  // checking distance to the nearest point
  double dist = checkDistance(source.getId(), lat, lon);
  if (dist > interpolation_max_distance)
    ROS_WARN("SPARK global planner: The coordinates is %f m out of the way", dist);

  map.publishPoint(lat, lon, Map::CURRENT_POSITION_MARKER);
  // draw paths network
  map.publishRouteNetwork();
  initializedPosition = true;
  ROS_INFO("SPARK global planner: Initialized. Waiting for request of plan...");
}