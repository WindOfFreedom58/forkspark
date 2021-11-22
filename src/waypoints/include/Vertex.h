#include <vector>
#include <geometry_msgs/Pose.h>

enum Type { INT_LEFT, INT_RIGHT, INT_STRAIGHT, LANE };

class Vertex {
 public:
  Vertex();
  ~Vertex() = default;

  // GETTERS
  double getLatitude() { return latitude; }
  double getLongitude() { return longitude; }
  int getUTM() { return UTM; }
  long getId() { return id; }
  Type getType() { return type; }
  geometry_msgs::PoseStamped getLocalPose(){ return localPose;}
  std::vector<std::pair<Vertex, double>> getNeighbors() { return neighbors; }

  // SETTER
  void setLatitude(double lat) { latitude = lat; };
  void setLongitude(double lon) { longitude = lon; };
  void setUTM(int utm) { this->UTM = utm; };
  void setType(Type type) { this->type = type; };
  void setId(int id) { this->id = id; };

  void setLocalFramePosition(geometry_msgs::Point position){
    this->localPose.pose.position = position;
  }

  void setLocalFrameOrientation(geometry_msgs::Quaternion orientation){
    this->localPose.pose.orientation = orientation;
  }

  // Neighbors
  void removeNeighborWithId(int id);
  double getDistanceToNeighbor(int id);

 private:
  double latitude;
  double longitude;
  int UTM;
  long id;
  Type type;
  geometry_msgs::PoseStamped localPose;

  std::vector<std::pair<Vertex, double>> neighbors;
};