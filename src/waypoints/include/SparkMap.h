#ifndef MAP
#define MAP

#include "Vertex.h"
#include "math.h"
#include "nav_msgs/Path.h"
#include "tf.h"

class Map {
 public:
  Map();
  ~Map() = default;

  // GETTERS

  int getNearestPoint(double lat, double lon);            // return vertex ID
  int getNearestPointXY(double point_x, double point_y);  // return vertex ID
  nav_msgs::Path getPath(std::vector<int> nodesInPath);
  Vertex getStartPoint();
  Vertex getNodeByID(int id);

  void createNetwork();
  void loadVertices();

  // SETTERS
  void setStartPoint(
      double latitude,
      double longitude);  // set the zero point in cartezian coordinates
  void setInterpolationMaxDistance(double param);

  class Haversine {
   public:
    static double getDistance(Vertex node1, Vertex node2);

    static double getCoordinateX(double lon1, double lon2, double lat1,
                                 double lat2);

    static double getCoordinateX(Vertex node1, Vertex node2);

    static double getCoordinateY(double lat1, double lat2);

    static double getCoordinateY(Vertex node1, Vertex node2);

    static double getBearing(Vertex node1, Vertex node2);

   private:
    constexpr static double R = 6371e3;
    constexpr static double DEG2RAD = M_PI / 180;
    constexpr static double RAD2DEG = 180 / M_PI;
  };

 private:
  std::vector<Vertex> vertices;
  std::vector<Vertex> getInterpolatedVertices(Vertex vertex1, Vertex vertex2);
  Vertex startPoint;
  std::string map_frame = "odom";

  double interpolation_max_distance;
};

#endif