#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cstdio>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/rclcpp.hpp"

class PCDSaver : public rclcpp::Node {
 public:
  PCDSaver() : Node("pcd_saver") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_raw", 10, std::bind(&PCDSaver::pcl_callback, this, std::placeholders::_1));
  }
  

 private:
  int count=0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Saving Point Cloud");
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);
    RCLCPP_INFO(this->get_logger(), "Converted");
    char name[30];
    sprintf(name, "pcd%d.pcd", count++);
    RCLCPP_INFO(this->get_logger(), "Name: %s", name);
    pcl::io::savePCDFileASCII(name, point_cloud);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDSaver>());
  rclcpp::shutdown();
  return 0;
}
