#include "SparkMap.h"

//creating the map network with distances
void Map::loadVertices(){
  
}

void Map::createNetwork(){
  if (this->vertices.empty()){
    std::cout << "Map is empty, network cannot be created" << std::endl;
    return;
  }

  double distance = 0.0;
  for(auto& vertex: vertices){
    for(auto& neighbor : vertex.getNeighbors()){
      auto neighborVertex = neighbor.first;
      distance = Haversine::getDistance(vertex, neighborVertex);
      neighbor.second = distance;
    }
  }

}

// getting defined path
nav_msgs::Path Map::getPath(std::vector<int> nodesInPath) {
  // msgs for shortest path
  nav_msgs::Path sh_path;

  sh_path.poses.clear();
  sh_path.header.frame_id = map_frame;

  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  for (int i = 0; i < nodesInPath.size(); i++) {
    pose.pose.position.x =
        Haversine::getCoordinateX(startPoint, vertices[nodesInPath[i]]);
    pose.pose.position.y =
        Haversine::getCoordinateY(startPoint, vertices[nodesInPath[i]]);
    double yaw = Haversine::getBearing(startPoint, vertices[nodesInPath[i]]);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose.header.seq = i;
    sh_path.poses.push_back(pose);
  }

  sh_path.header.stamp = ros::Time::now();
  return sh_path;
}

int Map::getNearestPoint(double lat, double lon) {
  Vertex point;
  point.setLongitude(lon);
  point.setLatitude(lat);
  int id = 0;

  double distance = Haversine::getDistance(point, vertices[0]);
  double minDistance = distance;

  for (int i = 0; i < vertices.size(); i++) {
    distance = Haversine::getDistance(point, vertices[i]);

    if (minDistance > distance) {
      minDistance = distance;
      id = i;
    }
  }
  return id;
}

int Map::getNearestPointXY(double point_x, double point_y) {
  int id = 0;

  double x = Haversine::getCoordinateX(startPoint, vertices[0]);
  double y = Haversine::getCoordinateY(startPoint, vertices[0]);
  double minDistance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));

  for (int i = 0; i < vertices.size(); i++) {
    x = Haversine::getCoordinateX(startPoint, vertices[i]);
    y = Haversine::getCoordinateY(startPoint, vertices[i]);

    double distance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));

    if (minDistance > distance) {
      minDistance = distance;
      id = i;
    }
  }
  return id;
}

Vertex Map::getStartPoint() { return startPoint; }
// return OSM NODE, which contains geographics coordinates
Vertex Map::getNodeByID(int id) { return vertices[id]; }

/* SETTERS */

void Map::setStartPoint(double latitude, double longitude) {
  this->startPoint.setLatitude(latitude);
  this->startPoint.setLongitude(longitude);
}

void Map::setInterpolationMaxDistance(double param) {
  this->interpolation_max_distance = param;
}

// INTERPOLATION - main algorithm
std::vector<Vertex> Map::getInterpolatedVertices(Vertex node1, Vertex node2) {
  std::vector<Vertex> new_nodes;
  Vertex new_node;
  double dist = Haversine::getDistance(node1, node2);
  int count_new_nodes =
      dist /
      interpolation_max_distance;  // calculate number of new interpolated nodes

  for (int i = 0; i < count_new_nodes; i++) {
    // weighted average. Example: when count_new_nodes = 2
    // 1. latitude = (2 * node1.latitude - 1*node2.latitude)/3
    // 2. latitude = (1 * node1.latitude - 2*node2.latitude)/3
    new_node.setLatitude(((count_new_nodes - i) * node1.getLatitude() +
                          (i + 1) * node2.getLatitude()) /
                         (count_new_nodes + 1));
    new_node.setLongitude(((count_new_nodes - i) * node1.getLongitude() +
                           (i + 1) * node2.getLongitude()) /
                          (count_new_nodes + 1));
    new_nodes.push_back(new_node);
  }
  return new_nodes;
}

double Map::Haversine::getDistance(Vertex node1,
                                      Vertex node2) {
  /*  Haversine formula:
   *  a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
   *  c = 2 ⋅ atan2( √a, √(1−a) )
   *  d = R ⋅ c
   *
   *  φ - latitude;
   *  λ - longitude;
   */

  double dLat = node2.getLatitude() * DEG2RAD - node1.getLatitude() * DEG2RAD;
  double dLon = node2.getLongitude() * DEG2RAD - node1.getLongitude() * DEG2RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(node1.getLatitude() * DEG2RAD) *
                                                 cos(node2.getLatitude() * DEG2RAD) *
                                                 sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

double Map::Haversine::getCoordinateX(double lon1, double lon2, double lat1,
                                         double lat2) {
  double dLon = lon2 * DEG2RAD - lon1 * DEG2RAD;
  double latAverage = (lat1 + lat2) / 2;
  double a = cos(latAverage * DEG2RAD) * cos(latAverage * DEG2RAD) *
             sin(dLon / 2) * sin(dLon / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return lon1 < lon2 ? dist : -dist;
}

double Map::Haversine::getCoordinateX(Vertex node1,
                                         Vertex node2) {
  double dLon = node2.getLongitude() * DEG2RAD - node1.getLongitude() * DEG2RAD;
  double latAverage = (node1.getLatitude() + node2.getLatitude()) / 2;
  double a = cos(latAverage * DEG2RAD) * cos(latAverage * DEG2RAD) *
             sin(dLon / 2) * sin(dLon / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return node1.getLongitude() < node2.getLongitude() ? dist : -dist;
}

double Map::Haversine::getCoordinateY(double lat1, double lat2) {
  static double R = 6371e3;
  double dLat = lat2 * DEG2RAD - lat1 * DEG2RAD;
  double a = sin(dLat / 2) * sin(dLat / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return lat1 < lat2 ? dist : -dist;
}

double Map::Haversine::getCoordinateY(Vertex node1,
                                         Vertex node2) {
  static double R = 6371e3;
  double dLat = node2.getLatitude() * DEG2RAD - node1.getLatitude() * DEG2RAD;
  double a = sin(dLat / 2) * sin(dLat / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return node1.getLatitude() < node2.getLatitude() ? dist : -dist;
}

double Map::Haversine::getBearing(Vertex node1,
                                     Vertex node2) {
  /*   Haversine formula:
   *   a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
   *   c = 2 ⋅ atan2( √a, √(1−a) )
   *   d = R ⋅ c
   *
   *  φ - latitude;
   *  λ - longitude;
   */

  double dLon = node2.getLongitude() * DEG2RAD - node1.getLongitude() * DEG2RAD;

  double y = sin(dLon) * cos(node2.getLatitude() * DEG2RAD);
  double x =
      cos(node1.getLatitude() * DEG2RAD) * sin(node2.getLatitude() * DEG2RAD) -
      sin(node1.getLatitude() * DEG2RAD) * cos(node2.getLatitude() * DEG2RAD) * cos(dLon);
  return atan2(y, x) * RAD2DEG;
}