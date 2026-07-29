#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

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

enum class DecisionCode
{
  NONE,
  MAP_UNAVAILABLE,
  MAP_FRAME_INVALID,
  MAP_EMPTY,
  STATE_UNINITIALIZED,
  STATE_LOST,
  ODOMETRY_UNAVAILABLE,
  ODOMETRY_FRAME_INVALID,
  ODOMETRY_STAMP_INVALID,
  ODOMETRY_STALE,
  ODOMETRY_FUTURE,
  ODOMETRY_POSE_INVALID,
  ODOMETRY_TOO_OLD,
  ODOMETRY_TOO_NEW,
  ODOMETRY_INTERPOLATION_GAP,
  SCAN_FRAME_INVALID,
  SCAN_STAMP_INVALID,
  SCAN_STALE,
  SCAN_FUTURE,
  SCAN_PRECEDES_INITIALIZATION,
  SCAN_EMPTY,
  SCAN_SUPERSEDED,
  INITIAL_POSE_FRAME_INVALID,
  INITIAL_POSE_STAMP_INVALID,
  INITIAL_POSE_STALE,
  INITIAL_POSE_FUTURE,
  INITIAL_POSE_POSE_INVALID,
  INITIAL_POSE_COVARIANCE_INVALID,
  INITIAL_POSE_COVARIANCE_AMBIGUOUS,
  INITIAL_POSE_WAITING_FOR_ODOMETRY,
  INITIALIZATION_STARTED,
  INITIALIZATION_CONFIRMATION_PENDING,
  INITIALIZATION_CONFIRMED,
  MATCHER_NOT_CONVERGED,
  RESULT_NON_FINITE,
  RESULT_NOT_RIGID,
  RESULT_TRANSLATION_JUMP,
  RESULT_ROTATION_JUMP,
  TRACKING_ACCEPTED,
  TRACKING_LOST,
};

const char * toString(LocalizationState state);
const char * toString(DecisionCode code);

struct TimedPose
{
  std::int64_t timestamp_ns = 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct OdometryLookup
{
  bool success = false;
  DecisionCode code = DecisionCode::ODOMETRY_UNAVAILABLE;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double before_gap_seconds = 0.0;
  double after_gap_seconds = 0.0;
};

class OdometryBuffer
{
public:
  OdometryBuffer(double duration_seconds, std::size_t maximum_samples);

  void add(std::int64_t timestamp_ns, const Eigen::Isometry3d & pose);
  OdometryLookup lookup(
    std::int64_t timestamp_ns,
    double maximum_interpolation_gap_seconds) const;
  void clear();
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

struct ValidationResult
{
  bool valid = false;
  DecisionCode code = DecisionCode::NONE;
};

ValidationResult validateTimestampNanoseconds(
  std::int64_t timestamp_ns,
  std::int64_t now_ns,
  double maximum_age_seconds,
  double future_tolerance_seconds,
  DecisionCode invalid_code,
  DecisionCode stale_code,
  DecisionCode future_code);

ValidationResult validateInitializationScanTimestamp(
  std::int64_t scan_timestamp_ns,
  std::int64_t initialization_timestamp_ns);

ValidationResult validateInitialPoseData(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const std::array<double, 36> & covariance,
  const InitialPoseValidationLimits & limits);

struct TransformValidation
{
  bool valid = false;
  DecisionCode code = DecisionCode::NONE;
  double translation_delta_m = 0.0;
  double rotation_delta_deg = 0.0;
};

TransformValidation validateTransformCandidate(
  const Eigen::Isometry3d & guess,
  const Eigen::Isometry3d & candidate,
  double maximum_translation_delta_m,
  double maximum_rotation_delta_deg,
  double rigidity_tolerance = 1.0e-3);

struct InitializationObservation
{
  bool accepted = false;
  bool confirmed = false;
  DecisionCode code = DecisionCode::NONE;
  std::size_t confirmation_count = 0;
  double translation_delta_m = 0.0;
  double rotation_delta_deg = 0.0;
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
  std::size_t consecutiveRejections() const;

  void beginInitialization(const Eigen::Isometry3d & prior_correction);
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
