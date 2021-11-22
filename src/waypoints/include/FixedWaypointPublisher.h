#include <nav_msgs/Path.h>
#include <spark_msgs/Waypoints.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

class FixedWaypointPublisher {
 public:
  FixedWaypointPublisher();
  ~FixedWaypointPublisher();

 private:
  std::string waypoints_file;
  ros::Publisher waypoint_pub;
  ros::Subscriber gps_subscriber;

  void gps_callback(sensor_msgs::NavSatFix gps_msg);
}