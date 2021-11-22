#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class ProcessPointClouds {
 public:
  // constructor
  ProcessPointClouds(){};

  // deconstructor
  ~ProcessPointClouds(){};

  PointCloudPtr readFromPCD(std::string filename);

  PointCloudPtr clear_point_cloud(PointCloudPtr pcloud);

  std::pair<PointCloudPtr, std::vector<pcl::PointIndices>> euclidean_clustering(
      PointCloudPtr pcloud, float clusterTolerance, int minSize, int maxSize);

  PointCloudPtr apply_voxel_grid(const PointCloudPtr& cloud, float leaf_size);
  PointCloudPtr take_region_with_angle(PointCloudPtr pcloud, double angle_back, double angle_forward);

 private:
  PointCloudPtr segment_plane(PointCloudPtr pcloud);

  PointCloudPtr remove_ground_plane(pcl::PointIndices::Ptr inliers,
                                    PointCloudPtr cloud);

  PointCloudPtr take_region(PointCloudPtr pcloud, Eigen::Vector4f minPoint,
                            Eigen::Vector4f maxPoint);
};

#endif /* PROCESSPOINTCLOUDS_H_ */