#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class LIDARCamProjector {
 public:
  LIDARCamProjector();
  LIDARCamProjector(cv::Mat tf, cv::Mat K);
  ~LIDARCamProjector() = default;

  void setK(cv::Mat K);
  void setDistCoeffs(cv::Mat distCoeffs) { this->distCoeffs = distCoeffs; }
  void setTf(cv::Mat tf) { this->tf = tf; }

  void setR(cv::Mat R) { this->R = R; }

  void setR(double rotation_radians) {
    cv::Mat R = cv::Mat(3, 3, cv::DataType<double>::type);
    R.at<double>(0, 0) = std::cos(rotation_radians);
    R.at<double>(0, 1) = -std::sin(rotation_radians);
    R.at<double>(0, 2) = 0.0;
    R.at<double>(1, 0) = std::sin(rotation_radians);
    R.at<double>(1, 1) = std::cos(rotation_radians);
    R.at<double>(1, 2) = 0.0;
    R.at<double>(2, 0) = 0.0;
    R.at<double>(2, 1) = 0.0;
    R.at<double>(2, 2) = 1.0;
    this->R = R;
  }

  cv::Mat projectLidarToCamera(cv::Mat img, PointCloudPtr cloud, std::vector<cv::Point>& projected_pts, std::vector<cv::Point2d>& lidar_points);

 private:
  cv::Mat tf;
  cv::Mat Tc;
  cv::Mat K;
  cv::Mat R;
  cv::Mat distCoeffs;
  cv::Size imageSize = cv::Size(640, 480);

  const float maxX = 20.0f, maxY = 7.0f, minZ = -0.0f;
};