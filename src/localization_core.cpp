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
// Allow optimizer and interpolation noise at the covariance envelope edge.
constexpr double kInitializationAcceptanceSlackFactor = 1.1;

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
    result.status = OdometryStatus::TOO_OLD;
    return result;
  }
  if (timestamp_ns > samples_.back().timestamp_ns) {
    result.status = OdometryStatus::TOO_NEW;
    return result;
  }

  const auto after = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_ns,
    [](const TimedPose & sample, std::int64_t timestamp) {
      return sample.timestamp_ns < timestamp;
    });
  if (after != samples_.end() && after->timestamp_ns == timestamp_ns) {
    result.status = OdometryStatus::AVAILABLE;
    result.pose = after->pose;
    return result;
  }
  if (after == samples_.begin() || after == samples_.end()) {
    result.status = OdometryStatus::INTERPOLATION_GAP;
    return result;
  }

  const auto before = std::prev(after);
  const double before_gap_seconds =
    static_cast<double>(timestamp_ns - before->timestamp_ns) /
    kNanosecondsPerSecond;
  const double after_gap_seconds =
    static_cast<double>(after->timestamp_ns - timestamp_ns) /
    kNanosecondsPerSecond;
  if (before_gap_seconds > maximum_interpolation_gap_seconds ||
    after_gap_seconds > maximum_interpolation_gap_seconds)
  {
    result.status = OdometryStatus::INTERPOLATION_GAP;
    return result;
  }
  result.status = OdometryStatus::AVAILABLE;
  result.pose = interpolatePose(*before, *after, timestamp_ns);
  return result;
}

std::size_t OdometryBuffer::size() const
{
  return samples_.size();
}

bool validTimestampNanoseconds(
  std::int64_t timestamp_ns,
  std::int64_t now_ns,
  double maximum_age_seconds,
  double future_tolerance_seconds)
{
  if (timestamp_ns <= 0 || now_ns <= 0) {
    return false;
  }
  const double age_seconds =
    static_cast<double>(now_ns - timestamp_ns) /
    kNanosecondsPerSecond;
  if (!std::isfinite(age_seconds)) {
    return false;
  }
  if (age_seconds > maximum_age_seconds) {
    return false;
  }
  if (age_seconds < -future_tolerance_seconds) {
    return false;
  }
  return true;
}

bool initializationScanFollowsPrior(
  std::int64_t scan_timestamp_ns,
  std::int64_t initialization_timestamp_ns)
{
  return initialization_timestamp_ns > 0 &&
         scan_timestamp_ns > initialization_timestamp_ns;
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

InitializationSearchBounds initializationSearchBounds(
  double position_stddev_m,
  double yaw_stddev_deg,
  double standard_deviation_multiplier,
  double minimum_translation_span_m,
  double maximum_translation_span_m,
  double minimum_yaw_span_deg,
  double maximum_yaw_span_deg)
{
  InitializationSearchBounds bounds;
  if (!std::isfinite(position_stddev_m) ||
    !std::isfinite(yaw_stddev_deg) ||
    !std::isfinite(standard_deviation_multiplier) ||
    !std::isfinite(minimum_translation_span_m) ||
    !std::isfinite(maximum_translation_span_m) ||
    !std::isfinite(minimum_yaw_span_deg) ||
    !std::isfinite(maximum_yaw_span_deg) ||
    position_stddev_m < 0.0 || yaw_stddev_deg < 0.0 ||
    standard_deviation_multiplier <= 0.0 ||
    minimum_translation_span_m < 0.0 ||
    maximum_translation_span_m < minimum_translation_span_m ||
    minimum_yaw_span_deg < 0.0 ||
    maximum_yaw_span_deg < minimum_yaw_span_deg)
  {
    return bounds;
  }
  bounds.translation_span_m = std::min(
    maximum_translation_span_m,
    std::max(
      minimum_translation_span_m,
      position_stddev_m * standard_deviation_multiplier));
  bounds.yaw_span_deg = std::min(
    maximum_yaw_span_deg,
    std::max(
      minimum_yaw_span_deg,
      yaw_stddev_deg * standard_deviation_multiplier));
  bounds.acceptance_translation_m =
    kInitializationAcceptanceSlackFactor *
    std::min(
      maximum_translation_span_m,
      std::max(
        0.5,
        position_stddev_m * standard_deviation_multiplier));
  bounds.acceptance_yaw_deg =
    kInitializationAcceptanceSlackFactor *
    std::min(
      maximum_yaw_span_deg,
      std::max(
        10.0,
        yaw_stddev_deg * standard_deviation_multiplier));
  return bounds;
}

Eigen::Isometry3d gravityConstrainedPose(
  const Eigen::Isometry3d & pose_prior,
  const Eigen::Isometry3d & gravity_aligned_odometry)
{
  if (!finiteTransform(pose_prior) ||
    !finiteTransform(gravity_aligned_odometry))
  {
    Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
    invalid.translation().setConstant(
      std::numeric_limits<double>::quiet_NaN());
    return invalid;
  }

  const double prior_yaw = std::atan2(
    pose_prior.rotation()(1, 0), pose_prior.rotation()(0, 0));
  const double odometry_yaw = std::atan2(
    gravity_aligned_odometry.rotation()(1, 0),
    gravity_aligned_odometry.rotation()(0, 0));
  const Eigen::AngleAxisd yaw_correction(
    prior_yaw - odometry_yaw, Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d constrained = pose_prior;
  constrained.linear() =
    yaw_correction.toRotationMatrix() *
    gravity_aligned_odometry.rotation();
  return constrained;
}

std::vector<HypothesisOffset> deterministicHypothesisOffsets(
  const InitializationSearchBounds & bounds,
  std::size_t maximum_hypotheses)
{
  if (maximum_hypotheses == 0 ||
    !std::isfinite(bounds.translation_span_m) ||
    !std::isfinite(bounds.yaw_span_deg) ||
    bounds.translation_span_m < 0.0 || bounds.yaw_span_deg < 0.0)
  {
    return {};
  }
  const double translation = bounds.translation_span_m;
  const double half_translation = 0.5 * translation;
  const double yaw = bounds.yaw_span_deg;
  const double half_yaw = 0.5 * yaw;
  const std::array<std::array<double, 2>, 13> positions = {{
      {{0.0, 0.0}},
      {{half_translation, 0.0}},
      {{-half_translation, 0.0}},
      {{0.0, half_translation}},
      {{0.0, -half_translation}},
      {{translation, 0.0}},
      {{-translation, 0.0}},
      {{0.0, translation}},
      {{0.0, -translation}},
      {{half_translation, half_translation}},
      {{half_translation, -half_translation}},
      {{-half_translation, half_translation}},
      {{-half_translation, -half_translation}},
    }};
  const std::array<double, 5> yaw_offsets = {{
      0.0, half_yaw, -half_yaw, yaw, -yaw,
    }};

  std::vector<HypothesisOffset> hypotheses;
  hypotheses.reserve(std::min<std::size_t>(
      maximum_hypotheses, positions.size() * yaw_offsets.size()));
  for (const auto & position : positions) {
    for (const double yaw_offset : yaw_offsets) {
      if (hypotheses.size() >= maximum_hypotheses) {
        return hypotheses;
      }
      HypothesisOffset hypothesis;
      hypothesis.x_m = position[0];
      hypothesis.y_m = position[1];
      hypothesis.yaw_deg = yaw_offset;
      hypotheses.push_back(hypothesis);
    }
  }
  return hypotheses;
}

HypothesisSelection selectBestHypothesis(
  const std::vector<ScoredPose> & candidates,
  double maximum_score,
  double minimum_score_margin,
  double distinct_translation_m,
  double distinct_rotation_deg)
{
  HypothesisSelection selection;
  if (!std::isfinite(maximum_score) || maximum_score < 0.0 ||
    !std::isfinite(minimum_score_margin) ||
    minimum_score_margin < 0.0 ||
    !std::isfinite(distinct_translation_m) ||
    distinct_translation_m < 0.0 ||
    !std::isfinite(distinct_rotation_deg) ||
    distinct_rotation_deg < 0.0)
  {
    return selection;
  }

  std::vector<std::size_t> ordered_indices;
  ordered_indices.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (std::isfinite(candidates[index].score) &&
      rigidTransform(candidates[index].pose, 1.0e-3))
    {
      ordered_indices.push_back(index);
    }
  }
  std::sort(
    ordered_indices.begin(), ordered_indices.end(),
    [&candidates](std::size_t left, std::size_t right)
    {
      return candidates[left].score < candidates[right].score;
    });
  if (ordered_indices.empty()) {
    return selection;
  }

  selection.best_index = ordered_indices.front();
  selection.second_index = selection.best_index;
  selection.best_score = candidates[selection.best_index].score;
  selection.second_score = std::numeric_limits<double>::infinity();
  selection.score_margin = std::numeric_limits<double>::infinity();
  if (selection.best_score > maximum_score) {
    return selection;
  }

  const Eigen::Isometry3d & best_pose =
    candidates[selection.best_index].pose;
  for (std::size_t ordered = 1; ordered < ordered_indices.size(); ++ordered) {
    const std::size_t index = ordered_indices[ordered];
    const Eigen::Isometry3d delta =
      best_pose.inverse() * candidates[index].pose;
    const double translation_delta = delta.translation().norm();
    const double rotation_delta =
      std::abs(Eigen::AngleAxisd(delta.rotation()).angle()) *
      180.0 / kPi;
    if (translation_delta >= distinct_translation_m ||
      rotation_delta >= distinct_rotation_deg)
    {
      selection.second_index = index;
      selection.second_score = candidates[index].score;
      selection.score_margin =
        selection.second_score - selection.best_score;
      selection.ambiguous =
        selection.score_margin < minimum_score_margin;
      break;
    }
  }
  selection.valid = !selection.ambiguous;
  return selection;
}

bool validInitialPoseData(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const std::array<double, 36> & covariance,
  const InitialPoseValidationLimits & limits)
{
  if (!position.allFinite() || !orientation.coeffs().allFinite()) {
    return false;
  }
  const double quaternion_norm = orientation.norm();
  if (!std::isfinite(quaternion_norm) ||
    quaternion_norm <= std::numeric_limits<double>::epsilon() ||
    std::abs(quaternion_norm - 1.0) > limits.quaternion_norm_tolerance)
  {
    return false;
  }

  Eigen::Matrix<double, 6, 6> covariance_matrix;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      const double value = covariance[row * 6 + column];
      if (!std::isfinite(value)) {
        return false;
      }
      covariance_matrix(row, column) = value;
    }
  }
  if ((covariance_matrix - covariance_matrix.transpose()).cwiseAbs().maxCoeff() >
    limits.covariance_symmetry_tolerance)
  {
    return false;
  }
  for (std::size_t index = 0; index < 6; ++index) {
    if (covariance_matrix(index, index) < -limits.covariance_psd_tolerance) {
      return false;
    }
  }
  const Eigen::Matrix<double, 6, 6> symmetric_covariance =
    0.5 * (covariance_matrix + covariance_matrix.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    symmetric_covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < -limits.covariance_psd_tolerance)
  {
    return false;
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
    return false;
  }

  return true;
}

bool validTransformCandidate(
  const Eigen::Isometry3d & guess,
  const Eigen::Isometry3d & candidate,
  double maximum_translation_delta_m,
  double maximum_rotation_delta_deg,
  double rigidity_tolerance)
{
  if (!finiteTransform(candidate)) {
    return false;
  }
  if (!rigidTransform(candidate, rigidity_tolerance)) {
    return false;
  }

  const Eigen::Isometry3d delta = guess.inverse() * candidate;
  const double translation_delta_m = delta.translation().norm();
  const Eigen::AngleAxisd rotation_delta(delta.rotation());
  const double rotation_delta_deg =
    std::abs(rotation_delta.angle()) * 180.0 / kPi;
  if (translation_delta_m > maximum_translation_delta_m) {
    return false;
  }
  if (rotation_delta_deg > maximum_rotation_delta_deg) {
    return false;
  }
  return true;
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

void LocalizationStateMachine::beginRelocalization(
  const Eigen::Isometry3d & prior_correction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LocalizationState::RELOCALIZING;
  initial_prior_correction_ = prior_correction;
  pending_correction_ = prior_correction;
  confirmation_count_ = 0;
  consecutive_rejections_ = 0;
}

bool LocalizationStateMachine::setInitializationCandidate(
  const Eigen::Isometry3d & candidate_correction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if ((state_ != LocalizationState::INITIALIZING &&
    state_ != LocalizationState::RELOCALIZING) ||
    !finiteTransform(candidate_correction))
  {
    return false;
  }
  initial_prior_correction_ = candidate_correction;
  pending_correction_ = candidate_correction;
  confirmation_count_ = 0;
  return true;
}

LocalizationState LocalizationStateMachine::failInitializationAttempt()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == LocalizationState::INITIALIZING) {
    state_ = LocalizationState::UNINITIALIZED;
    has_valid_correction_ = false;
  } else if (state_ == LocalizationState::RELOCALIZING) {
    state_ = LocalizationState::LOST;
  }
  confirmation_count_ = 0;
  consecutive_rejections_ = 0;
  return state_;
}

void LocalizationStateMachine::rejectInitializationCandidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LocalizationState::INITIALIZING &&
    state_ != LocalizationState::RELOCALIZING)
  {
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
  if (state_ != LocalizationState::INITIALIZING &&
    state_ != LocalizationState::RELOCALIZING)
  {
    return observation;
  }
  if (!validTransformCandidate(
      initial_prior_correction_, candidate_correction,
      maximum_translation_delta_m, maximum_rotation_delta_deg))
  {
    pending_correction_ = initial_prior_correction_;
    confirmation_count_ = 0;
    return observation;
  }

  ++confirmation_count_;
  observation.accepted = true;
  observation.confirmation_count = confirmation_count_;
  if (confirmation_count_ < required_confirmations_) {
    return observation;
  }

  pending_correction_ = candidate_correction;
  correction_ = candidate_correction;
  has_valid_correction_ = true;
  state_ = LocalizationState::TRACKING;
  confirmation_count_ = required_confirmations_;
  consecutive_rejections_ = 0;
  observation.confirmed = true;
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
