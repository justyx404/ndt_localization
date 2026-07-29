#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include <Eigen/Geometry>

namespace ndt_localization
{

enum class LocalizationState
{
  UNINITIALIZED,
  INITIALIZING,
  TRACKING,
  LOST,
  RELOCALIZING,
};

enum class OdometryStatus
{
  AVAILABLE,
  UNAVAILABLE,
  TOO_OLD,
  TOO_NEW,
  INTERPOLATION_GAP,
};

struct TimedPose
{
  std::int64_t timestamp_ns = 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct OdometryLookup
{
  OdometryStatus status = OdometryStatus::UNAVAILABLE;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

class OdometryBuffer
{
public:
  OdometryBuffer(double duration_seconds, std::size_t maximum_samples);

  void add(std::int64_t timestamp_ns, const Eigen::Isometry3d & pose);
  OdometryLookup lookup(
    std::int64_t timestamp_ns,
    double maximum_interpolation_gap_seconds) const;
  std::size_t size() const;

private:
  std::int64_t duration_ns_;
  std::size_t maximum_samples_;
  std::deque<TimedPose> samples_;
};

struct InitialPoseValidationLimits
{
  double quaternion_norm_tolerance = 1.0e-3;
  double covariance_symmetry_tolerance = 1.0e-6;
  double covariance_psd_tolerance = 1.0e-9;
  double maximum_position_stddev_m = 10.0;
  double maximum_yaw_stddev_deg = 180.0;
};

bool validTimestampNanoseconds(
  std::int64_t timestamp_ns,
  std::int64_t now_ns,
  double maximum_age_seconds,
  double future_tolerance_seconds);

bool initializationScanFollowsPrior(
  std::int64_t scan_timestamp_ns,
  std::int64_t initialization_timestamp_ns);

bool deadlineExpired(
  std::int64_t start_steady_time_ns,
  std::int64_t current_steady_time_ns,
  double deadline_ms);

std::vector<std::size_t> deterministicSampleIndices(
  std::size_t input_size,
  std::size_t maximum_size);

struct InitializationSearchBounds
{
  double translation_span_m = 0.0;
  double yaw_span_deg = 0.0;
};

InitializationSearchBounds initializationSearchBounds(
  double position_stddev_m,
  double yaw_stddev_deg,
  double standard_deviation_multiplier,
  double minimum_translation_span_m,
  double maximum_translation_span_m,
  double minimum_yaw_span_deg,
  double maximum_yaw_span_deg);

Eigen::Isometry3d gravityConstrainedPose(
  const Eigen::Isometry3d & pose_prior,
  const Eigen::Isometry3d & gravity_aligned_odometry);

struct HypothesisOffset
{
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_deg = 0.0;
};

std::vector<HypothesisOffset> deterministicHypothesisOffsets(
  const InitializationSearchBounds & bounds,
  std::size_t maximum_hypotheses);

struct ScoredPose
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double score = 0.0;
};

struct HypothesisSelection
{
  bool valid = false;
  bool ambiguous = false;
  std::size_t best_index = 0;
  std::size_t second_index = 0;
  double best_score = 0.0;
  double second_score = 0.0;
  double score_margin = 0.0;
};

HypothesisSelection selectBestHypothesis(
  const std::vector<ScoredPose> & candidates,
  double maximum_score,
  double minimum_score_margin,
  double distinct_translation_m,
  double distinct_rotation_deg);

bool validInitialPoseData(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const std::array<double, 36> & covariance,
  const InitialPoseValidationLimits & limits);

bool validTransformCandidate(
  const Eigen::Isometry3d & guess,
  const Eigen::Isometry3d & candidate,
  double maximum_translation_delta_m,
  double maximum_rotation_delta_deg,
  double rigidity_tolerance = 1.0e-3);

struct InitializationObservation
{
  bool accepted = false;
  bool confirmed = false;
  std::size_t confirmation_count = 0;
};

class LocalizationStateMachine
{
public:
  explicit LocalizationStateMachine(std::size_t required_confirmations);

  LocalizationState state() const;
  bool hasValidCorrection() const;
  bool getValidCorrection(Eigen::Isometry3d * correction) const;
  Eigen::Isometry3d correction() const;
  Eigen::Isometry3d pendingCorrection() const;
  std::size_t confirmationCount() const;

  void beginInitialization(const Eigen::Isometry3d & prior_correction);
  void beginRelocalization(const Eigen::Isometry3d & prior_correction);
  bool setInitializationCandidate(
    const Eigen::Isometry3d & candidate_correction);
  LocalizationState failInitializationAttempt();
  void rejectInitializationCandidate();
  InitializationObservation observeInitializationCorrection(
    const Eigen::Isometry3d & candidate_correction,
    double maximum_translation_delta_m,
    double maximum_rotation_delta_deg);
  bool applyTrackingCorrection(const Eigen::Isometry3d & correction);
  bool recordTrackingRejection(std::size_t maximum_consecutive_rejections);
  void enterLost();

private:
  LocalizationState state_ = LocalizationState::UNINITIALIZED;
  bool has_valid_correction_ = false;
  Eigen::Isometry3d correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_prior_correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pending_correction_ = Eigen::Isometry3d::Identity();
  std::size_t required_confirmations_;
  std::size_t confirmation_count_ = 0;
  std::size_t consecutive_rejections_ = 0;
  mutable std::mutex mutex_;
};

}  // namespace ndt_localization
