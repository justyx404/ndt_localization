#pragma once

#include <cstddef>

#include <Eigen/Core>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_localization
{

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

PointCloud::ConstPtr voxelDownsample(
  const PointCloud::ConstPtr & cloud,
  double leaf_size_m);

PointCloud::ConstPtr deterministicallyCap(
  const PointCloud::ConstPtr & cloud,
  std::size_t maximum_points);

PointCloud::ConstPtr radiusSubmap(
  const PointCloud::ConstPtr & map,
  const pcl::KdTreeFLANN<Point>::Ptr & map_index,
  const Eigen::Vector3d & center,
  double radius_m,
  std::size_t maximum_points);

}  // namespace ndt_localization
