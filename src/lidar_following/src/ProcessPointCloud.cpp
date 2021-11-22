#include "ProcessPointCloud.h"

#include <utility>

PointCloudPtr readFromPCD(const std::string& filename) {
  PointCloudPtr cloud(new PointCloud);
  if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read the file.");
    return (nullptr);
  }
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from "
            << filename << std::endl;
  return cloud;
}

PointCloudPtr ProcessPointClouds::clear_point_cloud(PointCloudPtr pcloud) {
  // PointCloudPtr barriers = segment_plane(std::move(pcloud));
  PointCloudPtr roi_barriers =
      take_region(pcloud, Eigen::Vector4f(-2.0, -3.5, -1.25, -1000),
                  Eigen::Vector4f(4.0, 3.5, -0.55, 1000));
  PointCloudPtr roi_barriers_angle =
      take_region_with_angle(roi_barriers, 5, 60);

  // project to x-y plane
  PointCloudPtr voxel_grid_applied =
      apply_voxel_grid(roi_barriers_angle, 0.04f);
  for (auto& point : voxel_grid_applied->points) {
    point.z = 0;
  }

  // Voxel grid applied before clustering
  return voxel_grid_applied;
}

PointCloudPtr ProcessPointClouds::segment_plane(PointCloudPtr pcloud) {
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(60);
  seg.setDistanceThreshold(0.2);

  seg.setInputCloud(pcloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cout
        << "Could not estimate a plane with RANSAC. Returning original cloud"
        << std::endl;
    return pcloud;
  }

  return remove_ground_plane(inliers, pcloud);
}

PointCloudPtr ProcessPointClouds::remove_ground_plane(
    pcl::PointIndices::Ptr inliers, PointCloudPtr cloud) {
  PointCloudPtr barriers(new PointCloud);
  pcl::ExtractIndices<pcl::PointXYZI> extract;

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(true);
  extract.filter(*barriers);

  return barriers;
}

PointCloudPtr ProcessPointClouds::apply_voxel_grid(const PointCloudPtr& cloud,
                                                   float leaf_size) {
  PointCloudPtr downsampledCloud(new PointCloud);
  pcl::VoxelGrid<pcl::PointXYZI> vox;

  vox.setInputCloud(cloud);
  vox.setLeafSize(leaf_size, leaf_size * 15, leaf_size);

  vox.filter(*downsampledCloud);
  return downsampledCloud;
}

PointCloudPtr ProcessPointClouds::take_region(PointCloudPtr pcloud,
                                              Eigen::Vector4f minPoint,
                                              Eigen::Vector4f maxPoint) {
  PointCloudPtr cloudRegion(new PointCloud);
  pcl::CropBox<pcl::PointXYZI> region(true);

  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(pcloud);
  region.filter(*cloudRegion);

  return cloudRegion;
}

PointCloudPtr ProcessPointClouds::take_region_with_angle(PointCloudPtr pcloud,
                                                         double angle_back,
                                                         double angle_forward) {
  PointCloudPtr angledPointCloud(new PointCloud);
  for (auto it = pcloud->points.begin(); it != pcloud->points.end(); ++it) {
    if ((it->x < tan(angle_back * (M_PI / 180.0)) * it->y &&
         it->x < -tan(angle_back * (M_PI / 180.0)) * it->y) ||
        (it->x > -tan(angle_forward * (M_PI / 180.0)) * it->y &&
         it->x > tan(angle_forward * (M_PI / 180.0)) * it->y)) {
      continue;
    }
    angledPointCloud->points.push_back(*it);
  }
  return angledPointCloud;
}

std::pair<PointCloudPtr, std::vector<pcl::PointIndices>>
ProcessPointClouds::euclidean_clustering(PointCloudPtr pcloud,
                                         float clusterTolerance, int minSize,
                                         int maxSize) {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);

  tree->setInputCloud(pcloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcloud);
  ec.extract(cluster_indices);

  return std::pair<PointCloudPtr, std::vector<pcl::PointIndices>>(
      pcloud, cluster_indices);  //
}