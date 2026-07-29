#include "localization_core.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include <Eigen/Eigenvalues>

namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kPi = 3.14159265358979323846;

bool finiteTransform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite();
}

bool rigidTransform(const Eigen::Isometry3d & transform, double tolerance)
{
  if (!finiteTransform(transform)) {
    return false;
  }
  const Eigen::Matrix4d & matrix = transform.matrix();
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  const double orthonormal_error =
    (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  const bool bottom_row_valid =
    std::abs(matrix(3, 0)) <= tolerance &&
    std::abs(matrix(3, 1)) <= tolerance &&
    std::abs(matrix(3, 2)) <= tolerance &&
    std::abs(matrix(3, 3) - 1.0) <= tolerance;
  return bottom_row_valid &&
         orthonormal_error <= tolerance &&
         std::abs(rotation.determinant() - 1.0) <= tolerance;
}

Eigen::Isometry3d interpolatePose(
  const ndt_localization::TimedPose & before,
  const ndt_localization::TimedPose & after,
  std::int64_t timestamp_ns)
{
  const double span =
    static_cast<double>(after.timestamp_ns - before.timestamp_ns);
  const double fraction =
    static_cast<double>(timestamp_ns - before.timestamp_ns) / span;
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
    before.pose.translation() +
    fraction * (after.pose.translation() - before.pose.translation());
  const Eigen::Quaterniond before_rotation(before.pose.rotation());
  const Eigen::Quaterniond after_rotation(after.pose.rotation());
  result.linear() =
    before_rotation.slerp(fraction, after_rotation).normalized().toRotationMatrix();
  return result;
}

}  // namespace

namespace ndt_localization
{

const char * toString(LocalizationState state)
{
  switch (state) {
    case LocalizationState::UNINITIALIZED:
      return "UNINITIALIZED";
    case LocalizationState::INITIALIZING:
      return "INITIALIZING";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::LOST:
      return "LOST";
    case LocalizationState::RELOCALIZING:
      return "RELOCALIZING";
  }
  return "UNKNOWN";
}

const char * toString(DecisionCode code)
{
  switch (code) {
    case DecisionCode::NONE:
      return "none";
    case DecisionCode::MAP_UNAVAILABLE:
      return "map_unavailable";
    case DecisionCode::MAP_FRAME_INVALID:
      return "map_frame_invalid";
    case DecisionCode::MAP_EMPTY:
      return "map_empty";
    case DecisionCode::STATE_UNINITIALIZED:
      return "state_uninitialized";
    case DecisionCode::STATE_LOST:
      return "state_lost";
    case DecisionCode::ODOMETRY_UNAVAILABLE:
      return "odometry_unavailable";
    case DecisionCode::ODOMETRY_FRAME_INVALID:
      return "odometry_frame_invalid";
    case DecisionCode::ODOMETRY_STAMP_INVALID:
      return "odometry_stamp_invalid";
    case DecisionCode::ODOMETRY_STALE:
      return "odometry_stale";
    case DecisionCode::ODOMETRY_FUTURE:
      return "odometry_future";
    case DecisionCode::ODOMETRY_POSE_INVALID:
      return "odometry_pose_invalid";
    case DecisionCode::ODOMETRY_TOO_OLD:
      return "odometry_too_old";
    case DecisionCode::ODOMETRY_TOO_NEW:
      return "odometry_too_new";
    case DecisionCode::ODOMETRY_INTERPOLATION_GAP:
      return "odometry_interpolation_gap";
    case DecisionCode::SCAN_FRAME_INVALID:
      return "scan_frame_invalid";
    case DecisionCode::SCAN_STAMP_INVALID:
      return "scan_stamp_invalid";
    case DecisionCode::SCAN_STALE:
      return "scan_stale";
    case DecisionCode::SCAN_FUTURE:
      return "scan_future";
    case DecisionCode::SCAN_PRECEDES_INITIALIZATION:
      return "scan_precedes_initialization";
    case DecisionCode::SCAN_EMPTY:
      return "scan_empty";
    case DecisionCode::SCAN_SUPERSEDED:
      return "scan_superseded";
    case DecisionCode::REGISTRATION_TIMEOUT:
      return "registration_timeout";
    case DecisionCode::RESULT_GENERATION_STALE:
      return "result_generation_stale";
    case DecisionCode::LOCAL_MAP_INSUFFICIENT:
      return "local_map_insufficient";
    case DecisionCode::INITIAL_POSE_FRAME_INVALID:
      return "initial_pose_frame_invalid";
    case DecisionCode::INITIAL_POSE_STAMP_INVALID:
      return "initial_pose_stamp_invalid";
    case DecisionCode::INITIAL_POSE_STALE:
      return "initial_pose_stale";
    case DecisionCode::INITIAL_POSE_FUTURE:
      return "initial_pose_future";
    case DecisionCode::INITIAL_POSE_POSE_INVALID:
      return "initial_pose_pose_invalid";
    case DecisionCode::INITIAL_POSE_COVARIANCE_INVALID:
      return "initial_pose_covariance_invalid";
    case DecisionCode::INITIAL_POSE_COVARIANCE_AMBIGUOUS:
      return "initial_pose_covariance_ambiguous";
    case DecisionCode::INITIAL_POSE_WAITING_FOR_ODOMETRY:
      return "initial_pose_waiting_for_odometry";
    case DecisionCode::INITIALIZATION_STARTED:
      return "initialization_started";
    case DecisionCode::INITIALIZATION_CONFIRMATION_PENDING:
      return "initialization_confirmation_pending";
    case DecisionCode::INITIALIZATION_CONFIRMED:
      return "initialization_confirmed";
    case DecisionCode::MATCHER_NOT_CONVERGED:
      return "matcher_not_converged";
    case DecisionCode::RESULT_NON_FINITE:
      return "result_non_finite";
    case DecisionCode::RESULT_NOT_RIGID:
      return "result_not_rigid";
    case DecisionCode::RESULT_TRANSLATION_JUMP:
      return "result_translation_jump";
    case DecisionCode::RESULT_ROTATION_JUMP:
      return "result_rotation_jump";
    case DecisionCode::TRACKING_ACCEPTED:
      return "tracking_accepted";
    case DecisionCode::TRACKING_LOST:
      return "tracking_lost";
  }
  return "unknown";
}

OdometryBuffer::OdometryBuffer(
  double duration_seconds,
  std::size_t maximum_samples)
: duration_ns_(
    static_cast<std::int64_t>(duration_seconds * kNanosecondsPerSecond)),
  maximum_samples_(std::max<std::size_t>(maximum_samples, 2))
{
}

void OdometryBuffer::add(
  std::int64_t timestamp_ns,
  const Eigen::Isometry3d & pose)
{
  const auto position = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_ns,
    [](const TimedPose & sample, std::int64_t timestamp) {
      return sample.timestamp_ns < timestamp;
    });
  if (position != samples_.end() && position->timestamp_ns == timestamp_ns) {
    position->pose = pose;
  } else {
    samples_.insert(position, TimedPose{timestamp_ns, pose});
  }

  if (!samples_.empty()) {
    const std::int64_t oldest_allowed =
      samples_.back().timestamp_ns - duration_ns_;
    while (!samples_.empty() &&
      samples_.front().timestamp_ns < oldest_allowed)
    {
      samples_.pop_front();
    }
  }
  while (samples_.size() > maximum_samples_) {
    samples_.pop_front();
  }
}

OdometryLookup OdometryBuffer::lookup(
  std::int64_t timestamp_ns,
  double maximum_interpolation_gap_seconds) const
{
  OdometryLookup result;
  if (samples_.empty()) {
    return result;
  }
  if (timestamp_ns < samples_.front().timestamp_ns) {
    result.code = DecisionCode::ODOMETRY_TOO_OLD;
    return result;
  }
  if (timestamp_ns > samples_.back().timestamp_ns) {
    result.code = DecisionCode::ODOMETRY_TOO_NEW;
    return result;
  }

  const auto after = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_ns,
    [](const TimedPose & sample, std::int64_t timestamp) {
      return sample.timestamp_ns < timestamp;
    });
  if (after != samples_.end() && after->timestamp_ns == timestamp_ns) {
    result.success = true;
    result.code = DecisionCode::NONE;
    result.pose = after->pose;
    return result;
  }
  if (after == samples_.begin() || after == samples_.end()) {
    result.code = DecisionCode::ODOMETRY_INTERPOLATION_GAP;
    return result;
  }

  const auto before = std::prev(after);
  result.before_gap_seconds =
    static_cast<double>(timestamp_ns - before->timestamp_ns) /
    kNanosecondsPerSecond;
  result.after_gap_seconds =
    static_cast<double>(after->timestamp_ns - timestamp_ns) /
    kNanosecondsPerSecond;
  if (result.before_gap_seconds > maximum_interpolation_gap_seconds ||
    result.after_gap_seconds > maximum_interpolation_gap_seconds)
  {
    result.code = DecisionCode::ODOMETRY_INTERPOLATION_GAP;
    return result;
  }
  result.success = true;
  result.code = DecisionCode::NONE;
  result.pose = interpolatePose(*before, *after, timestamp_ns);
  return result;
}

void OdometryBuffer::clear()
{
  samples_.clear();
}

std::size_t OdometryBuffer::size() const
{
  return samples_.size();
}

ValidationResult validateTimestampNanoseconds(
  std::int64_t timestamp_ns,
  std::int64_t now_ns,
  double maximum_age_seconds,
  double future_tolerance_seconds,
  DecisionCode invalid_code,
  DecisionCode stale_code,
  DecisionCode future_code)
{
  ValidationResult result;
  if (timestamp_ns <= 0 || now_ns <= 0) {
    result.code = invalid_code;
    return result;
  }
  const double age_seconds =
    static_cast<double>(now_ns - timestamp_ns) /
    kNanosecondsPerSecond;
  if (!std::isfinite(age_seconds)) {
    result.code = invalid_code;
    return result;
  }
  if (age_seconds > maximum_age_seconds) {
    result.code = stale_code;
    return result;
  }
  if (age_seconds < -future_tolerance_seconds) {
    result.code = future_code;
    return result;
  }
  result.valid = true;
  result.code = DecisionCode::NONE;
  return result;
}

ValidationResult validateInitializationScanTimestamp(
  std::int64_t scan_timestamp_ns,
  std::int64_t initialization_timestamp_ns)
{
  ValidationResult result;
  if (initialization_timestamp_ns <= 0 ||
    scan_timestamp_ns <= initialization_timestamp_ns)
  {
    result.code = DecisionCode::SCAN_PRECEDES_INITIALIZATION;
    return result;
  }
  result.valid = true;
  result.code = DecisionCode::NONE;
  return result;
}

bool deadlineExpired(
  std::int64_t start_steady_time_ns,
  std::int64_t current_steady_time_ns,
  double deadline_ms)
{
  if (start_steady_time_ns < 0 ||
    current_steady_time_ns < start_steady_time_ns ||
    !std::isfinite(deadline_ms) || deadline_ms < 0.0)
  {
    return true;
  }
  const double elapsed_ms =
    static_cast<double>(
    current_steady_time_ns - start_steady_time_ns) / 1.0e6;
  return elapsed_ms >= deadline_ms;
}

std::vector<std::size_t> deterministicSampleIndices(
  std::size_t input_size,
  std::size_t maximum_size)
{
  if (input_size == 0 || maximum_size == 0) {
    return {};
  }
  const std::size_t output_size = std::min(input_size, maximum_size);
  std::vector<std::size_t> indices;
  indices.reserve(output_size);
  if (output_size == input_size) {
    for (std::size_t index = 0; index < input_size; ++index) {
      indices.push_back(index);
    }
    return indices;
  }

  for (std::size_t index = 0; index < output_size; ++index) {
    indices.push_back(index * input_size / output_size);
  }
  return indices;
}

ValidationResult validateInitialPoseData(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const std::array<double, 36> & covariance,
  const InitialPoseValidationLimits & limits)
{
  ValidationResult result;
  if (!position.allFinite() || !orientation.coeffs().allFinite()) {
    result.code = DecisionCode::INITIAL_POSE_POSE_INVALID;
    return result;
  }
  const double quaternion_norm = orientation.norm();
  if (!std::isfinite(quaternion_norm) ||
    quaternion_norm <= std::numeric_limits<double>::epsilon() ||
    std::abs(quaternion_norm - 1.0) > limits.quaternion_norm_tolerance)
  {
    result.code = DecisionCode::INITIAL_POSE_POSE_INVALID;
    return result;
  }

  Eigen::Matrix<double, 6, 6> covariance_matrix;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      const double value = covariance[row * 6 + column];
      if (!std::isfinite(value)) {
        result.code = DecisionCode::INITIAL_POSE_COVARIANCE_INVALID;
        return result;
      }
      covariance_matrix(row, column) = value;
    }
  }
  if ((covariance_matrix - covariance_matrix.transpose()).cwiseAbs().maxCoeff() >
    limits.covariance_symmetry_tolerance)
  {
    result.code = DecisionCode::INITIAL_POSE_COVARIANCE_INVALID;
    return result;
  }
  for (std::size_t index = 0; index < 6; ++index) {
    if (covariance_matrix(index, index) < -limits.covariance_psd_tolerance) {
      result.code = DecisionCode::INITIAL_POSE_COVARIANCE_INVALID;
      return result;
    }
  }
  const Eigen::Matrix<double, 6, 6> symmetric_covariance =
    0.5 * (covariance_matrix + covariance_matrix.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    symmetric_covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < -limits.covariance_psd_tolerance)
  {
    result.code = DecisionCode::INITIAL_POSE_COVARIANCE_INVALID;
    return result;
  }

  const double maximum_position_variance =
    limits.maximum_position_stddev_m * limits.maximum_position_stddev_m;
  const double maximum_yaw_stddev_rad =
    limits.maximum_yaw_stddev_deg * kPi / 180.0;
  const double maximum_yaw_variance =
    maximum_yaw_stddev_rad * maximum_yaw_stddev_rad;
  if (covariance_matrix(0, 0) > maximum_position_variance ||
    covariance_matrix(1, 1) > maximum_position_variance ||
    covariance_matrix(2, 2) > maximum_position_variance ||
    covariance_matrix(5, 5) > maximum_yaw_variance)
  {
    result.code = DecisionCode::INITIAL_POSE_COVARIANCE_AMBIGUOUS;
    return result;
  }

  result.valid = true;
  result.code = DecisionCode::NONE;
  return result;
}

TransformValidation validateTransformCandidate(
  const Eigen::Isometry3d & guess,
  const Eigen::Isometry3d & candidate,
  double maximum_translation_delta_m,
  double maximum_rotation_delta_deg,
  double rigidity_tolerance)
{
  TransformValidation result;
  if (!finiteTransform(candidate)) {
    result.code = DecisionCode::RESULT_NON_FINITE;
    return result;
  }
  if (!rigidTransform(candidate, rigidity_tolerance)) {
    result.code = DecisionCode::RESULT_NOT_RIGID;
    return result;
  }

  const Eigen::Isometry3d delta = guess.inverse() * candidate;
  result.translation_delta_m = delta.translation().norm();
  const Eigen::AngleAxisd rotation_delta(delta.rotation());
  result.rotation_delta_deg =
    std::abs(rotation_delta.angle()) * 180.0 / kPi;
  if (result.translation_delta_m > maximum_translation_delta_m) {
    result.code = DecisionCode::RESULT_TRANSLATION_JUMP;
    return result;
  }
  if (result.rotation_delta_deg > maximum_rotation_delta_deg) {
    result.code = DecisionCode::RESULT_ROTATION_JUMP;
    return result;
  }
  result.valid = true;
  result.code = DecisionCode::NONE;
  return result;
}

LocalizationStateMachine::LocalizationStateMachine(
  std::size_t required_confirmations)
: required_confirmations_(
    std::max<std::size_t>(required_confirmations, 1))
{
}

LocalizationState LocalizationStateMachine::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool LocalizationStateMachine::hasValidCorrection() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_valid_correction_;
}

bool LocalizationStateMachine::getValidCorrection(
  Eigen::Isometry3d * correction) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_valid_correction_) {
    return false;
  }
  if (correction != nullptr) {
    *correction = correction_;
  }
  return true;
}

Eigen::Isometry3d LocalizationStateMachine::correction() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return correction_;
}

Eigen::Isometry3d LocalizationStateMachine::pendingCorrection() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_correction_;
}

std::size_t LocalizationStateMachine::confirmationCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return confirmation_count_;
}

std::size_t LocalizationStateMachine::consecutiveRejections() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return consecutive_rejections_;
}

void LocalizationStateMachine::beginInitialization(
  const Eigen::Isometry3d & prior_correction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LocalizationState::INITIALIZING;
  has_valid_correction_ = false;
  initial_prior_correction_ = prior_correction;
  pending_correction_ = prior_correction;
  confirmation_count_ = 0;
  consecutive_rejections_ = 0;
}

void LocalizationStateMachine::rejectInitializationCandidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LocalizationState::INITIALIZING) {
    return;
  }
  pending_correction_ = initial_prior_correction_;
  confirmation_count_ = 0;
}

InitializationObservation
LocalizationStateMachine::observeInitializationCorrection(
  const Eigen::Isometry3d & candidate_correction,
  double maximum_translation_delta_m,
  double maximum_rotation_delta_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  InitializationObservation observation;
  if (state_ != LocalizationState::INITIALIZING) {
    observation.code = DecisionCode::STATE_UNINITIALIZED;
    return observation;
  }
  const TransformValidation validation = validateTransformCandidate(
    pending_correction_, candidate_correction,
    maximum_translation_delta_m, maximum_rotation_delta_deg);
  observation.translation_delta_m = validation.translation_delta_m;
  observation.rotation_delta_deg = validation.rotation_delta_deg;
  if (!validation.valid) {
    pending_correction_ = initial_prior_correction_;
    confirmation_count_ = 0;
    observation.code = validation.code;
    return observation;
  }

  pending_correction_ = candidate_correction;
  ++confirmation_count_;
  observation.accepted = true;
  observation.confirmation_count = confirmation_count_;
  if (confirmation_count_ < required_confirmations_) {
    observation.code = DecisionCode::INITIALIZATION_CONFIRMATION_PENDING;
    return observation;
  }

  correction_ = pending_correction_;
  has_valid_correction_ = true;
  state_ = LocalizationState::TRACKING;
  confirmation_count_ = required_confirmations_;
  consecutive_rejections_ = 0;
  observation.confirmed = true;
  observation.code = DecisionCode::INITIALIZATION_CONFIRMED;
  return observation;
}

bool LocalizationStateMachine::applyTrackingCorrection(
  const Eigen::Isometry3d & correction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LocalizationState::TRACKING || !finiteTransform(correction)) {
    return false;
  }
  correction_ = correction;
  has_valid_correction_ = true;
  consecutive_rejections_ = 0;
  return true;
}

bool LocalizationStateMachine::recordTrackingRejection(
  std::size_t maximum_consecutive_rejections)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LocalizationState::TRACKING) {
    return false;
  }
  ++consecutive_rejections_;
  if (maximum_consecutive_rejections > 0 &&
    consecutive_rejections_ >= maximum_consecutive_rejections)
  {
    state_ = LocalizationState::LOST;
    confirmation_count_ = 0;
    consecutive_rejections_ = 0;
    return true;
  }
  return false;
}

void LocalizationStateMachine::enterLost()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LocalizationState::LOST;
  confirmation_count_ = 0;
  consecutive_rejections_ = 0;
}

}  // namespace ndt_localization
