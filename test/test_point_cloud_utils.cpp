#include "point_cloud_utils.h"

#include <gtest/gtest.h>

namespace
{

ndt_localization::PointCloud::Ptr lineCloud(std::size_t size)
{
  ndt_localization::PointCloud::Ptr cloud(
    new ndt_localization::PointCloud());
  cloud->reserve(size);
  for (std::size_t index = 0; index < size; ++index) {
    ndt_localization::Point point;
    point.x = static_cast<float>(index);
    point.y = 0.0F;
    point.z = 0.0F;
    cloud->push_back(point);
  }
  return cloud;
}

}  // namespace

TEST(PointCloudUtils, ReusesCloudWhenNoFilteringIsRequired)
{
  const ndt_localization::PointCloud::ConstPtr cloud = lineCloud(3);

  EXPECT_EQ(
    ndt_localization::voxelDownsample(cloud, 0.0).get(),
    cloud.get());
  EXPECT_EQ(
    ndt_localization::deterministicallyCap(cloud, 3).get(),
    cloud.get());
}

TEST(PointCloudUtils, AppliesDeterministicPointCap)
{
  const ndt_localization::PointCloud::ConstPtr cloud = lineCloud(10);

  const auto capped =
    ndt_localization::deterministicallyCap(cloud, 4);

  ASSERT_EQ(capped->size(), 4u);
  EXPECT_FLOAT_EQ((*capped)[0].x, 0.0F);
  EXPECT_FLOAT_EQ((*capped)[1].x, 2.0F);
  EXPECT_FLOAT_EQ((*capped)[2].x, 5.0F);
  EXPECT_FLOAT_EQ((*capped)[3].x, 7.0F);
}

TEST(PointCloudUtils, ExtractsBoundedRadiusSubmap)
{
  const ndt_localization::PointCloud::ConstPtr map = lineCloud(5);
  pcl::KdTreeFLANN<ndt_localization::Point>::Ptr map_index(
    new pcl::KdTreeFLANN<ndt_localization::Point>(true));
  map_index->setInputCloud(map);

  const auto local_map = ndt_localization::radiusSubmap(
    map, map_index, Eigen::Vector3d(2.0, 0.0, 0.0), 1.1, 10);

  EXPECT_EQ(local_map->size(), 3u);
}
