#include "lidar_to_cam.hpp"

LIDARCamProjector::LIDARCamProjector(){
  Tc = cv::Mat(3, 3, cv::DataType<double>::type);

  Tc.at<double>(0, 0) = 0.0;
  Tc.at<double>(0, 1) = -1.0;
  Tc.at<double>(0, 2) = 0.0;
  Tc.at<double>(1, 0) = 0.0;
  Tc.at<double>(1, 1) = 0.0;
  Tc.at<double>(1, 2) = -1.0;
  Tc.at<double>(2, 0) = 1.0;
  Tc.at<double>(2, 1) = 0.0;
  Tc.at<double>(2, 2) = 0.0;
  
  // default no rotation
  this->setR(0.0);
}

LIDARCamProjector::LIDARCamProjector(cv::Mat tf, cv::Mat K) : tf(tf), K(K) {
  LIDARCamProjector();
}

void LIDARCamProjector::setK(cv::Mat K){
  
  try{
    auto newCameraMatrix = cv::getOptimalNewCameraMatrix(K, distCoeffs, imageSize, 1, imageSize, 0);
    this->K = newCameraMatrix;
    ROS_INFO("Distortion coefficient applied intrinsic matrix created");
  } catch(std::exception& e){
    ROS_WARN("Distortion coefficient could not have been applied: %s", e);
    this->K = K;
  }
}

cv::Mat LIDARCamProjector::projectLidarToCamera(cv::Mat img,
                                                PointCloudPtr cloud,
                                                std::vector<cv::Point>& projected_pts,
                                                std::vector<cv::Point2d>& lidar_points) {
  cv::Mat visImg = img.clone();
  cv::Mat overlay = visImg.clone();

  cv::Mat X(3, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
    if (it->x < 0.2f || it->x > maxX || std::abs(it->y) > maxY ||
        it->z < minZ) {  //|| it->r < minR
      continue;
    }

    X.at<double>(0, 0) = it->x;
    X.at<double>(1, 0) = it->y;
    X.at<double>(2, 0) = it->z;

    Y = K * Tc * (R * (X + tf));

    Y.at<double>(0, 0) = Y.at<double>(0, 0) / Y.at<double>(2, 0);
    Y.at<double>(1, 0) = Y.at<double>(1, 0) / Y.at<double>(2, 0);

    cv::Point pt(Y.at<double>(0, 0), Y.at<double>(1, 0));
    projected_pts.push_back(pt);
    
    cv::Point2d lidar_pt(it->x, it->y);
    lidar_points.push_back(lidar_pt);
    
    float val = it->x;
    float maxVal = 15.0;
    int red = std::min(255, (int)(255 * std::abs((val - maxVal) / maxVal)));
    int green =
        std::min(255, (int)(255 * (1 - std::abs((val - maxVal) / maxVal))));
    cv::circle(overlay, pt, 2, cv::Scalar(0, green, red), -1);
  }

  float opacity = 0.6;
  cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

  return visImg;
}