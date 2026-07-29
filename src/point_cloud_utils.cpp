#include "point_cloud_utils.h"

#include <vector>

#include "localization_core.h"

#include <pcl/filters/voxel_grid.h>

namespace ndt_localization
{

PointCloud::ConstPtr voxelDownsample(
  const PointCloud::ConstPtr & cloud,
  double leaf_size_m)
{
  if (!cloud) {
    return PointCloud::ConstPtr(new PointCloud());
  }
  if (cloud->empty() || leaf_size_m <= 0.0) {
    return cloud;
  }

  PointCloud::Ptr filtered(new PointCloud());
  pcl::VoxelGrid<Point> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(
    leaf_size_m, leaf_size_m, leaf_size_m);
  voxel_grid.filter(*filtered);
  return filtered;
}

PointCloud::ConstPtr deterministicallyCap(
  const PointCloud::ConstPtr & cloud,
  std::size_t maximum_points)
{
  if (!cloud) {
    return PointCloud::ConstPtr(new PointCloud());
  }
  if (cloud->size() <= maximum_points) {
    return cloud;
  }

  PointCloud::Ptr capped(new PointCloud());
  capped->reserve(maximum_points);
  capped->is_dense = cloud->is_dense;
  for (const std::size_t index :
    deterministicSampleIndices(cloud->size(), maximum_points))
  {
    capped->push_back((*cloud)[index]);
  }
  return capped;
}

PointCloud::ConstPtr radiusSubmap(
  const PointCloud::ConstPtr & map,
  const pcl::KdTreeFLANN<Point>::Ptr & map_index,
  const Eigen::Vector3d & center,
  double radius_m,
  std::size_t maximum_points)
{
  PointCloud::Ptr local_map(new PointCloud());
  if (!map || map->empty() || !map_index ||
    radius_m <= 0.0 || maximum_points == 0)
  {
    return local_map;
  }

  Point query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = static_cast<float>(center.z());
  std::vector<int> indices;
  std::vector<float> squared_distances;
  map_index->radiusSearch(
    query, radius_m, indices, squared_distances,
    static_cast<unsigned int>(maximum_points));

  local_map->reserve(indices.size());
  local_map->is_dense = map->is_dense;
  for (const int index : indices) {
    local_map->push_back((*map)[static_cast<std::size_t>(index)]);
  }
  return local_map;
}

}  // namespace ndt_localization
