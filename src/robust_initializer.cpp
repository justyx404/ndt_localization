#include "robust_initializer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double elapsedMilliseconds(
  const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

bool beforeDeadline(
  const std::chrono::steady_clock::time_point & deadline)
{
  return std::chrono::steady_clock::now() < deadline;
}

}  // namespace

namespace ndt_localization
{

RobustInitializer::RobustInitializer(const Config & config)
: config_(config),
  map_kdtree_(new pcl::KdTreeFLANN<Point>(true))
{
  if (config_.local_map_radius_m <= 0.0 ||
    config_.maximum_local_map_points == 0 ||
    config_.minimum_local_map_points == 0 ||
    config_.minimum_local_map_points > config_.maximum_local_map_points ||
    config_.maximum_hypotheses == 0 ||
    config_.coarse_map_leaf_size_m < 0.0 ||
    config_.coarse_scan_leaf_size_m < 0.0 ||
    config_.maximum_coarse_scan_points == 0 ||
    config_.coarse_resolution_m <= 0.0 ||
    config_.coarse_step_size_m <= 0.0 ||
    config_.coarse_transformation_epsilon <= 0.0 ||
    config_.coarse_maximum_iterations < 1 ||
    config_.refinement_scan_leaf_size_m < 0.0 ||
    config_.maximum_refinement_scan_points == 0 ||
    config_.refinement_resolution_m <= 0.0 ||
    config_.refinement_step_size_m <= 0.0 ||
    config_.refinement_transformation_epsilon <= 0.0 ||
    config_.refinement_maximum_iterations < 1 ||
    config_.refinement_candidates == 0 ||
    config_.refinement_reserve_ms < 0.0 ||
    config_.fitness_max_range_m <= 0.0 ||
    config_.maximum_fitness_score < 0.0 ||
    config_.minimum_score_margin < 0.0 ||
    config_.distinct_translation_m <= 0.0 ||
    config_.distinct_rotation_deg <= 0.0)
  {
    throw std::invalid_argument("invalid robust initializer configuration");
  }
}

void RobustInitializer::setMap(const Cloud::ConstPtr & map)
{
  map_ = map;
  map_kdtree_->setInputCloud(map_);
}

RobustInitializer::Cloud::Ptr RobustInitializer::downsample(
  const Cloud::ConstPtr & cloud,
  double leaf_size_m) const
{
  if (!cloud || cloud->empty() || leaf_size_m <= 0.0) {
    Cloud::Ptr copy(new Cloud());
    if (cloud) {
      *copy = *cloud;
    }
    return copy;
  }
  Cloud::Ptr filtered(new Cloud());
  pcl::VoxelGrid<Point> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(
    leaf_size_m, leaf_size_m, leaf_size_m);
  voxel_grid.filter(*filtered);
  return filtered;
}

RobustInitializer::Cloud::Ptr RobustInitializer::cap(
  const Cloud::ConstPtr & cloud,
  std::size_t maximum_points) const
{
  Cloud::Ptr capped(new Cloud());
  if (!cloud) {
    return capped;
  }
  if (cloud->size() <= maximum_points) {
    *capped = *cloud;
    return capped;
  }
  capped->reserve(maximum_points);
  capped->is_dense = cloud->is_dense;
  for (const std::size_t index :
    deterministicSampleIndices(cloud->size(), maximum_points))
  {
    capped->push_back((*cloud)[index]);
  }
  return capped;
}

RobustInitializer::Cloud::Ptr RobustInitializer::localMap(
  const Eigen::Vector3d & center,
  double radius_m,
  std::size_t maximum_points) const
{
  Cloud::Ptr local_map(new Cloud());
  if (!map_ || map_->empty()) {
    return local_map;
  }
  Point query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = static_cast<float>(center.z());
  std::vector<int> indices;
  std::vector<float> squared_distances;
  map_kdtree_->radiusSearch(
    query, radius_m, indices, squared_distances,
    static_cast<unsigned int>(maximum_points));
  local_map->reserve(indices.size());
  local_map->is_dense = map_->is_dense;
  for (const int index : indices) {
    local_map->push_back((*map_)[static_cast<std::size_t>(index)]);
  }
  return local_map;
}

Eigen::Isometry3d RobustInitializer::applyOffset(
  const Eigen::Isometry3d & prior_pose,
  const HypothesisOffset & offset) const
{
  Eigen::Isometry3d pose = prior_pose;
  pose.translation().x() += offset.x_m;
  pose.translation().y() += offset.y_m;
  const Eigen::AngleAxisd yaw(
    offset.yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ());
  pose.linear() = yaw.toRotationMatrix() * prior_pose.rotation();
  return pose;
}

void RobustInitializer::configureCoarseMatcher(
  pcl::NormalDistributionsTransform<Point, Point> * matcher) const
{
  matcher->setResolution(config_.coarse_resolution_m);
  matcher->setStepSize(config_.coarse_step_size_m);
  matcher->setTransformationEpsilon(
    config_.coarse_transformation_epsilon);
  matcher->setMaximumIterations(config_.coarse_maximum_iterations);
}

void RobustInitializer::configureRefinementMatcher(
  pcl::NormalDistributionsTransform<Point, Point> * matcher) const
{
  matcher->setResolution(config_.refinement_resolution_m);
  matcher->setStepSize(config_.refinement_step_size_m);
  matcher->setTransformationEpsilon(
    config_.refinement_transformation_epsilon);
  matcher->setMaximumIterations(
    config_.refinement_maximum_iterations);
}

RobustInitializer::Result RobustInitializer::search(
  const Request & request)
{
  const auto total_start = std::chrono::steady_clock::now();
  Result result;
  if (!map_ || map_->empty() || !request.scan ||
    request.scan->empty() || !request.prior_pose.matrix().allFinite())
  {
    result.total_ms = elapsedMilliseconds(total_start);
    return result;
  }

  const std::vector<HypothesisOffset> offsets =
    deterministicHypothesisOffsets(
    request.bounds, config_.maximum_hypotheses);
  result.hypotheses = offsets.size();
  const double search_radius =
    config_.local_map_radius_m + request.bounds.translation_span_m;
  Cloud::Ptr target = localMap(
    request.prior_pose.translation(), search_radius,
    config_.maximum_local_map_points);
  result.target_points = target->size();
  if (target->size() < config_.minimum_local_map_points) {
    result.code = DecisionCode::LOCAL_MAP_INSUFFICIENT;
    result.total_ms = elapsedMilliseconds(total_start);
    return result;
  }

  Cloud::Ptr coarse_target =
    downsample(target, config_.coarse_map_leaf_size_m);
  Cloud::Ptr coarse_scan =
    downsample(request.scan, config_.coarse_scan_leaf_size_m);
  coarse_scan = cap(
    coarse_scan, config_.maximum_coarse_scan_points);
  result.scan_points = coarse_scan->size();
  if (coarse_scan->empty()) {
    result.code = DecisionCode::SCAN_EMPTY;
    result.total_ms = elapsedMilliseconds(total_start);
    return result;
  }

  pcl::NormalDistributionsTransform<Point, Point> coarse_matcher;
  configureCoarseMatcher(&coarse_matcher);
  coarse_matcher.setInputTarget(coarse_target);
  coarse_matcher.setInputSource(coarse_scan);
  std::vector<ScoredPose> coarse_candidates;
  const auto coarse_start = std::chrono::steady_clock::now();
  const auto coarse_deadline =
    request.deadline -
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(
      config_.refinement_reserve_ms));
  for (const HypothesisOffset & offset : offsets) {
    if (!beforeDeadline(coarse_deadline)) {
      result.timed_out = true;
      break;
    }
    Cloud aligned;
    coarse_matcher.align(
      aligned,
      applyOffset(request.prior_pose, offset).matrix().cast<float>());
    ++result.evaluated;
    if (!coarse_matcher.hasConverged()) {
      continue;
    }
    const double fitness =
      coarse_matcher.getFitnessScore(config_.fitness_max_range_m);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.matrix() =
      coarse_matcher.getFinalTransformation().cast<double>();
    if (!std::isfinite(fitness) || !pose.matrix().allFinite()) {
      continue;
    }
    ++result.converged;
    ScoredPose candidate;
    candidate.pose = pose;
    candidate.score = fitness;
    coarse_candidates.push_back(candidate);
  }
  result.coarse_ms = elapsedMilliseconds(coarse_start);
  if (coarse_candidates.empty()) {
    result.code = result.timed_out ?
      DecisionCode::INITIALIZATION_SEARCH_TIMEOUT :
      DecisionCode::INITIALIZATION_SEARCH_FAILED;
    result.total_ms = elapsedMilliseconds(total_start);
    return result;
  }

  std::sort(
    coarse_candidates.begin(), coarse_candidates.end(),
    [](const ScoredPose & left, const ScoredPose & right)
    {
      return left.score < right.score;
    });
  std::vector<ScoredPose> refinement_seeds;
  for (const ScoredPose & candidate : coarse_candidates) {
    bool distinct = true;
    for (const ScoredPose & seed : refinement_seeds) {
      const TransformValidation comparison = validateTransformCandidate(
        seed.pose, candidate.pose,
        config_.distinct_translation_m,
        config_.distinct_rotation_deg);
      if (comparison.valid) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      refinement_seeds.push_back(candidate);
    }
    if (refinement_seeds.size() >= config_.refinement_candidates) {
      break;
    }
  }

  Cloud::Ptr refinement_scan =
    downsample(request.scan, config_.refinement_scan_leaf_size_m);
  refinement_scan = cap(
    refinement_scan, config_.maximum_refinement_scan_points);
  pcl::NormalDistributionsTransform<Point, Point> refinement_matcher;
  configureRefinementMatcher(&refinement_matcher);
  refinement_matcher.setInputTarget(target);
  refinement_matcher.setInputSource(refinement_scan);
  std::vector<ScoredPose> refined_candidates;
  const auto refinement_start = std::chrono::steady_clock::now();
  for (const ScoredPose & seed : refinement_seeds) {
    if (!beforeDeadline(request.deadline)) {
      result.timed_out = true;
      break;
    }
    Cloud aligned;
    refinement_matcher.align(
      aligned, seed.pose.matrix().cast<float>());
    if (!refinement_matcher.hasConverged()) {
      continue;
    }
    const double fitness =
      refinement_matcher.getFitnessScore(config_.fitness_max_range_m);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.matrix() =
      refinement_matcher.getFinalTransformation().cast<double>();
    if (!std::isfinite(fitness) || !pose.matrix().allFinite()) {
      continue;
    }
    ScoredPose candidate;
    candidate.pose = pose;
    candidate.score = fitness;
    refined_candidates.push_back(candidate);
    ++result.refined;
  }
  result.refinement_ms = elapsedMilliseconds(refinement_start);
  const HypothesisSelection selection = selectBestHypothesis(
    refined_candidates,
    config_.maximum_fitness_score,
    config_.minimum_score_margin,
    config_.distinct_translation_m,
    config_.distinct_rotation_deg);
  result.best_score = selection.best_score;
  result.second_score = selection.second_score;
  result.score_margin = selection.score_margin;
  result.ambiguous = selection.ambiguous;
  if (selection.valid) {
    result.success = true;
    result.pose = refined_candidates[selection.best_index].pose;
    result.code = DecisionCode::INITIALIZATION_HYPOTHESIS_SELECTED;
  } else if (selection.ambiguous) {
    result.code = DecisionCode::INITIALIZATION_AMBIGUOUS;
  } else if (result.timed_out) {
    result.code = DecisionCode::INITIALIZATION_SEARCH_TIMEOUT;
  } else {
    result.code = DecisionCode::INITIALIZATION_SEARCH_FAILED;
  }
  result.total_ms = elapsedMilliseconds(total_start);
  return result;
}

}  // namespace ndt_localization
