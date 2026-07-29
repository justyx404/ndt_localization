#pragma once

#include <chrono>
#include <cstddef>
#include <memory>

#include "localization_core.h"
#include "point_cloud_utils.h"

#include <pcl/registration/ndt.h>

namespace ndt_localization
{

class RobustInitializer
{
public:
  using Point = ndt_localization::Point;
  using Cloud = ndt_localization::PointCloud;

  struct Config
  {
    double local_map_radius_m = 35.0;
    std::size_t maximum_local_map_points = 120000;
    std::size_t minimum_local_map_points = 1000;
    std::size_t maximum_hypotheses = 65;
    double coarse_map_leaf_size_m = 0.5;
    double coarse_scan_leaf_size_m = 0.4;
    std::size_t maximum_coarse_scan_points = 2000;
    double coarse_resolution_m = 2.0;
    double coarse_step_size_m = 0.2;
    double coarse_transformation_epsilon = 0.05;
    int coarse_maximum_iterations = 8;
    double refinement_scan_leaf_size_m = 0.0;
    std::size_t maximum_refinement_scan_points = 4000;
    double refinement_resolution_m = 1.0;
    double refinement_step_size_m = 0.1;
    double refinement_transformation_epsilon = 0.005;
    int refinement_maximum_iterations = 15;
    std::size_t refinement_candidates = 3;
    double refinement_reserve_ms = 250.0;
    double fitness_max_range_m = 2.0;
    double maximum_fitness_score = 0.5;
    double minimum_score_margin = 0.01;
    double distinct_translation_m = 0.5;
    double distinct_rotation_deg = 5.0;
  };

  struct Request
  {
    Cloud::ConstPtr scan;
    Eigen::Isometry3d prior_pose = Eigen::Isometry3d::Identity();
    InitializationSearchBounds bounds;
    std::chrono::steady_clock::time_point deadline;
  };

  explicit RobustInitializer(const Config & config);

  void setMap(const Cloud::ConstPtr & map);
  bool search(
    const Request & request,
    Eigen::Isometry3d * selected_pose);

private:
  Eigen::Isometry3d applyOffset(
    const Eigen::Isometry3d & prior_pose,
    const HypothesisOffset & offset) const;
  void configureCoarseMatcher(
    pcl::NormalDistributionsTransform<Point, Point> * matcher) const;
  void configureRefinementMatcher(
    pcl::NormalDistributionsTransform<Point, Point> * matcher) const;

  Config config_;
  Cloud::ConstPtr map_;
  pcl::KdTreeFLANN<Point>::Ptr map_kdtree_;
};

}  // namespace ndt_localization
