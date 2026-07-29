#include "localization.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace
{

diagnostic_msgs::msg::KeyValue keyValue(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template<typename T>
diagnostic_msgs::msg::KeyValue numericKeyValue(
  const std::string & key,
  T value)
{
  return keyValue(key, std::to_string(value));
}

std::string normalizedFrame(const std::string & frame)
{
  const std::size_t first_character = frame.find_first_not_of('/');
  if (first_character == std::string::npos) {
    return "";
  }
  return frame.substr(first_character);
}

bool validPose(
  const geometry_msgs::msg::Pose & pose,
  double quaternion_norm_tolerance)
{
  const Eigen::Vector3d position(
    pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite()) {
    return false;
  }
  const double quaternion_norm = orientation.norm();
  return std::isfinite(quaternion_norm) &&
         quaternion_norm > std::numeric_limits<double>::epsilon() &&
         std::abs(quaternion_norm - 1.0) <= quaternion_norm_tolerance;
}

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  transform.linear() = orientation.normalized().toRotationMatrix();
  return transform;
}

double elapsedMilliseconds(
  const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

}  // namespace

LocalizationNode::LocalizationNode()
: Node("localization_node")
{
  RCLCPP_INFO(
    this->get_logger(), "Initializing FAST-LIO Localization Node ...");

  this->declare_parameter<std::string>("odom_frame_id", "camera_init");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("map_frame_id", "map");

  this->declare_parameter<double>("localization.ndt_resolution", 1.0);
  this->declare_parameter<double>("localization.ndt_step_size", 0.1);
  this->declare_parameter<double>("localization.ndt_trans_epsilon", 0.01);
  this->declare_parameter<int>("localization.ndt_max_iter", 30);
  this->declare_parameter<double>("localization.ndt_map_leaf_size", 0.0);
  this->declare_parameter<double>("localization.ndt_scan_leaf_size", 0.0);
  this->declare_parameter<bool>("localization.ndt_log_runtime", false);
  this->declare_parameter<bool>(
    "localization.ndt_compute_fitness_score", false);
  this->declare_parameter<bool>(
    "localization.publish_scan_diagnostics", true);

  this->declare_parameter<double>(
    "localization.odom_buffer_duration_seconds", 3.0);
  this->declare_parameter<int>(
    "localization.odom_buffer_max_samples", 500);
  this->declare_parameter<double>(
    "localization.max_odom_interpolation_gap_seconds", 0.15);
  this->declare_parameter<double>(
    "localization.max_odometry_age_seconds", 0.5);
  this->declare_parameter<double>(
    "localization.max_scan_age_seconds", 0.5);
  this->declare_parameter<double>(
    "localization.max_initial_pose_age_seconds", 1.0);
  this->declare_parameter<double>(
    "localization.future_tolerance_seconds", 0.1);
  this->declare_parameter<double>(
    "localization.quaternion_norm_tolerance", 1.0e-3);
  this->declare_parameter<double>(
    "localization.covariance_symmetry_tolerance", 1.0e-6);
  this->declare_parameter<double>(
    "localization.covariance_psd_tolerance", 1.0e-9);
  this->declare_parameter<double>(
    "localization.max_initial_position_stddev_m", 10.0);
  this->declare_parameter<double>(
    "localization.max_initial_yaw_stddev_deg", 180.0);
  this->declare_parameter<int>(
    "localization.initialization_confirmation_scans", 3);
  this->declare_parameter<double>(
    "localization.max_result_translation_delta_m", 1.0);
  this->declare_parameter<double>(
    "localization.max_result_rotation_delta_deg", 20.0);
  this->declare_parameter<double>(
    "localization.max_confirmation_translation_delta_m", 0.5);
  this->declare_parameter<double>(
    "localization.max_confirmation_rotation_delta_deg", 10.0);
  this->declare_parameter<int>(
    "localization.max_consecutive_rejections", 10);

  this->get_parameter("odom_frame_id", this->odom_frame_id_);
  this->get_parameter("base_frame_id", this->base_frame_id_);
  this->get_parameter("map_frame_id", this->global_frame_id_);
  this->get_parameter(
    "localization.ndt_resolution", this->ndt_resolution_);
  this->get_parameter(
    "localization.ndt_step_size", this->ndt_step_size_);
  this->get_parameter(
    "localization.ndt_trans_epsilon", this->ndt_trans_epsilon_);
  this->get_parameter(
    "localization.ndt_max_iter", this->ndt_max_iter_);
  this->get_parameter(
    "localization.ndt_map_leaf_size", this->ndt_map_leaf_size_);
  this->get_parameter(
    "localization.ndt_scan_leaf_size", this->ndt_scan_leaf_size_);
  this->get_parameter(
    "localization.ndt_log_runtime", this->ndt_log_runtime_);
  this->get_parameter(
    "localization.ndt_compute_fitness_score",
    this->ndt_compute_fitness_score_);
  this->get_parameter(
    "localization.publish_scan_diagnostics",
    this->publish_scan_diagnostics_);
  this->get_parameter(
    "localization.odom_buffer_duration_seconds",
    this->odometry_buffer_duration_seconds_);
  this->get_parameter(
    "localization.odom_buffer_max_samples",
    this->odometry_buffer_max_samples_);
  this->get_parameter(
    "localization.max_odom_interpolation_gap_seconds",
    this->maximum_odometry_interpolation_gap_seconds_);
  this->get_parameter(
    "localization.max_odometry_age_seconds",
    this->maximum_odometry_age_seconds_);
  this->get_parameter(
    "localization.max_scan_age_seconds",
    this->maximum_scan_age_seconds_);
  this->get_parameter(
    "localization.max_initial_pose_age_seconds",
    this->maximum_initial_pose_age_seconds_);
  this->get_parameter(
    "localization.future_tolerance_seconds",
    this->future_tolerance_seconds_);
  this->get_parameter(
    "localization.quaternion_norm_tolerance",
    this->quaternion_norm_tolerance_);
  this->get_parameter(
    "localization.covariance_symmetry_tolerance",
    this->covariance_symmetry_tolerance_);
  this->get_parameter(
    "localization.covariance_psd_tolerance",
    this->covariance_psd_tolerance_);
  this->get_parameter(
    "localization.max_initial_position_stddev_m",
    this->maximum_initial_position_stddev_m_);
  this->get_parameter(
    "localization.max_initial_yaw_stddev_deg",
    this->maximum_initial_yaw_stddev_deg_);
  this->get_parameter(
    "localization.initialization_confirmation_scans",
    this->initialization_confirmation_scans_);
  this->get_parameter(
    "localization.max_result_translation_delta_m",
    this->maximum_result_translation_delta_m_);
  this->get_parameter(
    "localization.max_result_rotation_delta_deg",
    this->maximum_result_rotation_delta_deg_);
  this->get_parameter(
    "localization.max_confirmation_translation_delta_m",
    this->maximum_confirmation_translation_delta_m_);
  this->get_parameter(
    "localization.max_confirmation_rotation_delta_deg",
    this->maximum_confirmation_rotation_delta_deg_);
  this->get_parameter(
    "localization.max_consecutive_rejections",
    this->maximum_consecutive_rejections_);

  this->global_frame_id_ = normalizedFrame(this->global_frame_id_);
  this->odom_frame_id_ = normalizedFrame(this->odom_frame_id_);
  this->base_frame_id_ = normalizedFrame(this->base_frame_id_);
  if (this->global_frame_id_.empty() ||
    this->odom_frame_id_.empty() ||
    this->base_frame_id_.empty())
  {
    throw std::invalid_argument("localization frame IDs cannot be empty");
  }
  if (this->odometry_buffer_duration_seconds_ <= 0.0 ||
    this->odometry_buffer_max_samples_ < 2 ||
    this->maximum_odometry_interpolation_gap_seconds_ <= 0.0 ||
    this->maximum_odometry_age_seconds_ < 0.0 ||
    this->maximum_scan_age_seconds_ < 0.0 ||
    this->maximum_initial_pose_age_seconds_ < 0.0 ||
    this->future_tolerance_seconds_ < 0.0 ||
    this->quaternion_norm_tolerance_ < 0.0 ||
    this->covariance_symmetry_tolerance_ < 0.0 ||
    this->covariance_psd_tolerance_ < 0.0 ||
    this->maximum_initial_position_stddev_m_ < 0.0 ||
    this->maximum_initial_yaw_stddev_deg_ < 0.0 ||
    this->initialization_confirmation_scans_ < 1 ||
    this->maximum_result_translation_delta_m_ <= 0.0 ||
    this->maximum_result_rotation_delta_deg_ <= 0.0 ||
    this->maximum_confirmation_translation_delta_m_ <= 0.0 ||
    this->maximum_confirmation_rotation_delta_deg_ <= 0.0 ||
    this->maximum_consecutive_rejections_ < 1)
  {
    throw std::invalid_argument("invalid Phase 1 localization parameters");
  }

  this->odometry_buffer_ =
    std::make_unique<ndt_localization::OdometryBuffer>(
    this->odometry_buffer_duration_seconds_,
    static_cast<std::size_t>(this->odometry_buffer_max_samples_));
  this->state_machine_ =
    std::make_unique<ndt_localization::LocalizationStateMachine>(
    static_cast<std::size_t>(this->initialization_confirmation_scans_));
  this->tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->global_map_.reset(new pcl::PointCloud<PointType>());

  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  this->map_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/global_map", map_qos,
    std::bind(
      &LocalizationNode::mapCallback, this, std::placeholders::_1));
  this->odometry_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions odometry_options;
  odometry_options.callback_group = this->odometry_callback_group_;
  rclcpp::QoS odometry_qos(100);
  odometry_qos.best_effort();
  odometry_qos.durability_volatile();
  this->odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry_lio", odometry_qos,
    std::bind(
      &LocalizationNode::odomCallback, this, std::placeholders::_1),
    odometry_options);
  this->scan_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered_body", 10,
    std::bind(
      &LocalizationNode::scanCallback, this, std::placeholders::_1));
  this->initial_pose_sub_ =
    this->create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1,
    std::bind(
      &LocalizationNode::initialPoseCallback, this,
      std::placeholders::_1));

  this->pub_odom_ =
    this->create_publisher<nav_msgs::msg::Odometry>("/odometry_map", 10);
  this->diagnostic_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/localization/scan_diagnostics", 10);
}

LocalizationNode::~LocalizationNode()
{
}

void LocalizationNode::mapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (this->map_initialized_) {
    return;
  }
  if (normalizedFrame(msg->header.frame_id) != this->global_frame_id_) {
    RCLCPP_ERROR(
      this->get_logger(), "Rejected map with frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), this->global_frame_id_.c_str());
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::MAP_FRAME_INVALID);
    return;
  }
  if (msg->width == 0 || msg->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Rejected empty global map");
    this->publishStateDiagnostic(
      msg->header.stamp, ndt_localization::DecisionCode::MAP_EMPTY);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received global map with %u points",
    msg->width * msg->height);
  pcl::fromROSMsg(*msg, *this->global_map_);
  this->raw_map_points_ = this->global_map_->size();
  this->global_map_ = this->downsampleCloud(
    this->global_map_, this->ndt_map_leaf_size_);
  if (this->global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Rejected map with no usable points");
    this->publishStateDiagnostic(
      msg->header.stamp, ndt_localization::DecisionCode::MAP_EMPTY);
    return;
  }

  this->ndt_.setResolution(this->ndt_resolution_);
  this->ndt_.setStepSize(this->ndt_step_size_);
  this->ndt_.setTransformationEpsilon(this->ndt_trans_epsilon_);
  this->ndt_.setMaximumIterations(this->ndt_max_iter_);
  this->ndt_.setInputTarget(this->global_map_);
  this->map_initialized_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "NDT target initialized with %zu points (%zu before filtering)",
    this->global_map_->size(), this->raw_map_points_);
}

pcl::PointCloud<PointType>::Ptr LocalizationNode::downsampleCloud(
  const pcl::PointCloud<PointType>::Ptr & cloud,
  double leaf_size)
{
  if (leaf_size <= 0.0 || cloud->empty()) {
    return cloud;
  }
  pcl::PointCloud<PointType>::Ptr filtered(
    new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*filtered);
  return filtered;
}

ndt_localization::DecisionCode LocalizationNode::validateTimestamp(
  const builtin_interfaces::msg::Time & stamp,
  double maximum_age_seconds,
  ndt_localization::DecisionCode invalid_code,
  ndt_localization::DecisionCode stale_code,
  ndt_localization::DecisionCode future_code,
  double * age_ms) const
{
  if (stamp.sec <= 0 || stamp.nanosec >= 1000000000u) {
    return invalid_code;
  }
  const rclcpp::Time message_time(stamp);
  const std::int64_t now_ns = this->now().nanoseconds();
  const double age_seconds =
    static_cast<double>(now_ns - message_time.nanoseconds()) / 1.0e9;
  if (age_ms != nullptr) {
    *age_ms = age_seconds * 1000.0;
  }
  return ndt_localization::validateTimestampNanoseconds(
    message_time.nanoseconds(), now_ns, maximum_age_seconds,
    this->future_tolerance_seconds_, invalid_code, stale_code,
    future_code).code;
}

void LocalizationNode::odomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (normalizedFrame(msg->header.frame_id) != this->odom_frame_id_ ||
    normalizedFrame(msg->child_frame_id) != this->base_frame_id_)
  {
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::ODOMETRY_FRAME_INVALID);
    return;
  }
  double age_ms = 0.0;
  const ndt_localization::DecisionCode timestamp_result =
    this->validateTimestamp(
    msg->header.stamp, this->maximum_odometry_age_seconds_,
    ndt_localization::DecisionCode::ODOMETRY_STAMP_INVALID,
    ndt_localization::DecisionCode::ODOMETRY_STALE,
    ndt_localization::DecisionCode::ODOMETRY_FUTURE,
    &age_ms);
  if (timestamp_result != ndt_localization::DecisionCode::NONE) {
    this->publishStateDiagnostic(msg->header.stamp, timestamp_result);
    return;
  }
  if (!validPose(msg->pose.pose, this->quaternion_norm_tolerance_)) {
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::ODOMETRY_POSE_INVALID);
    return;
  }

  const Eigen::Isometry3d odom_to_base =
    poseToIsometry(msg->pose.pose);
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    this->odometry_buffer_->add(
      rclcpp::Time(msg->header.stamp).nanoseconds(), odom_to_base);
  }
  this->tryStartPendingInitialization();
  this->processPendingScan();

  Eigen::Isometry3d valid_correction = Eigen::Isometry3d::Identity();
  if (this->state_machine_->getValidCorrection(&valid_correction)) {
    this->publishMapPrediction(*msg, odom_to_base, valid_correction);
  }
}

void LocalizationNode::publishMapPrediction(
  const nav_msgs::msg::Odometry & odometry,
  const Eigen::Isometry3d & odom_to_base,
  const Eigen::Isometry3d & map_to_odom)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odometry.header.stamp;
  transform.header.frame_id = this->global_frame_id_;
  transform.child_frame_id = this->odom_frame_id_;
  transform.transform = tf2::eigenToTransform(map_to_odom).transform;
  this->tf_broadcaster_->sendTransform(transform);

  nav_msgs::msg::Odometry map_odometry = odometry;
  map_odometry.header.frame_id = this->global_frame_id_;
  map_odometry.child_frame_id = this->base_frame_id_;
  const Eigen::Isometry3d map_to_base = map_to_odom * odom_to_base;
  map_odometry.pose.pose = tf2::toMsg(map_to_base);

  const Eigen::Matrix3d rotation = map_to_odom.rotation();
  Eigen::Matrix<double, 6, 6> odom_covariance =
    Eigen::Matrix<double, 6, 6>::Zero();
  for (std::size_t index = 0; index < 36; ++index) {
    odom_covariance(index / 6, index % 6) =
      odometry.pose.covariance[index];
  }
  Eigen::Matrix<double, 6, 6> map_covariance =
    Eigen::Matrix<double, 6, 6>::Zero();
  map_covariance.block<3, 3>(0, 0) =
    rotation * odom_covariance.block<3, 3>(0, 0) *
    rotation.transpose();
  map_covariance.block<3, 3>(3, 3) =
    rotation * odom_covariance.block<3, 3>(3, 3) *
    rotation.transpose();
  map_covariance.block<3, 3>(0, 3) =
    rotation * odom_covariance.block<3, 3>(0, 3) *
    rotation.transpose();
  map_covariance.block<3, 3>(3, 0) =
    rotation * odom_covariance.block<3, 3>(3, 0) *
    rotation.transpose();
  for (std::size_t index = 0; index < 36; ++index) {
    map_odometry.pose.covariance[index] =
      map_covariance(index / 6, index % 6);
  }
  this->pub_odom_->publish(map_odometry);
}

void LocalizationNode::scanCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> scan_lock(this->scan_mutex_);
  const auto callback_start = std::chrono::steady_clock::now();
  ScanMetrics metrics;
  metrics.raw_scan_points =
    static_cast<std::size_t>(msg->width) * msg->height;

  const auto reject_scan =
    [this, &metrics, &callback_start, &msg](
    ndt_localization::DecisionCode code,
    bool registration_rejection)
    {
      if (this->state_machine_->state() ==
        ndt_localization::LocalizationState::INITIALIZING)
      {
        this->state_machine_->rejectInitializationCandidate();
      } else if (registration_rejection &&
        this->state_machine_->state() ==
        ndt_localization::LocalizationState::TRACKING)
      {
        if (this->state_machine_->recordTrackingRejection(
            static_cast<std::size_t>(
              this->maximum_consecutive_rejections_)))
        {
          this->publishStateDiagnostic(
            msg->header.stamp,
            ndt_localization::DecisionCode::TRACKING_LOST);
        }
      }
      metrics.decision = code;
      metrics.total_ms = elapsedMilliseconds(callback_start);
      this->publishScanDiagnostic(msg->header.stamp, metrics);
    };

  if (normalizedFrame(msg->header.frame_id) != this->base_frame_id_) {
    reject_scan(
      ndt_localization::DecisionCode::SCAN_FRAME_INVALID, false);
    return;
  }
  const ndt_localization::DecisionCode timestamp_result =
    this->validateTimestamp(
    msg->header.stamp, this->maximum_scan_age_seconds_,
    ndt_localization::DecisionCode::SCAN_STAMP_INVALID,
    ndt_localization::DecisionCode::SCAN_STALE,
    ndt_localization::DecisionCode::SCAN_FUTURE,
    &metrics.input_age_ms);
  if (timestamp_result != ndt_localization::DecisionCode::NONE) {
    reject_scan(timestamp_result, false);
    return;
  }
  if (!this->map_initialized_) {
    reject_scan(
      ndt_localization::DecisionCode::MAP_UNAVAILABLE, false);
    return;
  }
  if (this->state_machine_->state() ==
    ndt_localization::LocalizationState::UNINITIALIZED)
  {
    reject_scan(
      ndt_localization::DecisionCode::STATE_UNINITIALIZED, false);
    return;
  }
  if (this->state_machine_->state() ==
    ndt_localization::LocalizationState::LOST ||
    this->state_machine_->state() ==
    ndt_localization::LocalizationState::RELOCALIZING)
  {
    reject_scan(ndt_localization::DecisionCode::STATE_LOST, false);
    return;
  }
  if (this->state_machine_->state() ==
    ndt_localization::LocalizationState::INITIALIZING)
  {
    const ndt_localization::ValidationResult sequence_validation =
      ndt_localization::validateInitializationScanTimestamp(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      this->initialization_stamp_ns_);
    if (!sequence_validation.valid) {
      reject_scan(sequence_validation.code, false);
      return;
    }
  }

  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      this->maximum_odometry_interpolation_gap_seconds_);
  }
  metrics.odometry_before_gap_ms =
    synchronized_odometry.before_gap_seconds * 1000.0;
  metrics.odometry_after_gap_ms =
    synchronized_odometry.after_gap_seconds * 1000.0;
  if (!synchronized_odometry.success) {
    if (synchronized_odometry.code ==
      ndt_localization::DecisionCode::ODOMETRY_TOO_NEW)
    {
      {
        std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
        if (this->pending_scan_ &&
          rclcpp::Time(this->pending_scan_->header.stamp) <
          rclcpp::Time(msg->header.stamp))
        {
          if (this->state_machine_->state() ==
            ndt_localization::LocalizationState::INITIALIZING)
          {
            this->state_machine_->rejectInitializationCandidate();
          }
          ScanMetrics superseded_metrics;
          superseded_metrics.decision =
            ndt_localization::DecisionCode::SCAN_SUPERSEDED;
          superseded_metrics.raw_scan_points =
            static_cast<std::size_t>(this->pending_scan_->width) *
            this->pending_scan_->height;
          superseded_metrics.input_age_ms =
            (this->now() -
            rclcpp::Time(this->pending_scan_->header.stamp)).seconds() *
            1000.0;
          this->publishScanDiagnostic(
            this->pending_scan_->header.stamp, superseded_metrics);
        }
        if (!this->pending_scan_ ||
          rclcpp::Time(this->pending_scan_->header.stamp) <=
          rclcpp::Time(msg->header.stamp))
        {
          this->pending_scan_ = msg;
        }
      }
      return;
    }
    reject_scan(synchronized_odometry.code, false);
    return;
  }
  if (metrics.raw_scan_points == 0) {
    reject_scan(ndt_localization::DecisionCode::SCAN_EMPTY, false);
    return;
  }

  const auto conversion_start = std::chrono::steady_clock::now();
  pcl::PointCloud<PointType>::Ptr scan(
    new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *scan);
  scan = this->downsampleCloud(scan, this->ndt_scan_leaf_size_);
  metrics.filtered_scan_points = scan->size();
  metrics.conversion_ms = elapsedMilliseconds(conversion_start);
  if (scan->empty()) {
    reject_scan(ndt_localization::DecisionCode::SCAN_EMPTY, false);
    return;
  }

  const Eigen::Isometry3d correction_guess =
    this->state_machine_->state() ==
    ndt_localization::LocalizationState::INITIALIZING ?
    this->state_machine_->pendingCorrection() :
    this->state_machine_->correction();
  const Eigen::Isometry3d pose_guess =
    correction_guess * synchronized_odometry.pose;

  const auto matcher_start = std::chrono::steady_clock::now();
  this->ndt_.setInputSource(scan);
  pcl::PointCloud<PointType>::Ptr aligned_cloud(
    new pcl::PointCloud<PointType>());
  this->ndt_.align(
    *aligned_cloud, pose_guess.matrix().cast<float>());
  metrics.matcher_ms = elapsedMilliseconds(matcher_start);
  metrics.converged = this->ndt_.hasConverged();
  metrics.iterations = this->ndt_.getFinalNumIteration();
  if (metrics.converged && this->ndt_compute_fitness_score_) {
    metrics.fitness_score = this->ndt_.getFitnessScore();
  }
  if (!metrics.converged) {
    reject_scan(
      ndt_localization::DecisionCode::MATCHER_NOT_CONVERGED, true);
    return;
  }

  const auto validation_start = std::chrono::steady_clock::now();
  Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
  optimized_pose.matrix() =
    this->ndt_.getFinalTransformation().cast<double>();
  const ndt_localization::TransformValidation pose_validation =
    ndt_localization::validateTransformCandidate(
    pose_guess, optimized_pose,
    this->maximum_result_translation_delta_m_,
    this->maximum_result_rotation_delta_deg_);
  metrics.translation_delta_m =
    pose_validation.translation_delta_m;
  metrics.rotation_delta_deg =
    pose_validation.rotation_delta_deg;
  if (!pose_validation.valid) {
    metrics.validation_ms = elapsedMilliseconds(validation_start);
    reject_scan(pose_validation.code, true);
    return;
  }

  const Eigen::Isometry3d candidate_correction =
    optimized_pose * synchronized_odometry.pose.inverse();
  if (this->state_machine_->state() ==
    ndt_localization::LocalizationState::INITIALIZING)
  {
    const ndt_localization::InitializationObservation observation =
      this->state_machine_->observeInitializationCorrection(
      candidate_correction,
      this->maximum_confirmation_translation_delta_m_,
      this->maximum_confirmation_rotation_delta_deg_);
    if (!observation.accepted) {
      metrics.translation_delta_m = observation.translation_delta_m;
      metrics.rotation_delta_deg = observation.rotation_delta_deg;
      metrics.validation_ms = elapsedMilliseconds(validation_start);
      reject_scan(observation.code, true);
      return;
    }
    metrics.decision = observation.code;
    metrics.accepted = observation.confirmed;
    if (observation.confirmed) {
      this->publishStateDiagnostic(
        msg->header.stamp,
        ndt_localization::DecisionCode::INITIALIZATION_CONFIRMED);
    }
  } else {
    if (!this->state_machine_->applyTrackingCorrection(
        candidate_correction))
    {
      metrics.validation_ms = elapsedMilliseconds(validation_start);
      reject_scan(
        ndt_localization::DecisionCode::STATE_LOST, false);
      return;
    }
    metrics.decision =
      ndt_localization::DecisionCode::TRACKING_ACCEPTED;
    metrics.accepted = true;
  }
  metrics.validation_ms = elapsedMilliseconds(validation_start);
  metrics.total_ms = elapsedMilliseconds(callback_start);
  this->publishScanDiagnostic(msg->header.stamp, metrics);

  if (this->ndt_log_runtime_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "NDT: %.3f ms, points: %zu -> %zu, iterations: %d, "
      "state: %s, decision: %s",
      metrics.matcher_ms, metrics.raw_scan_points,
      metrics.filtered_scan_points, metrics.iterations,
      ndt_localization::toString(this->state_machine_->state()),
      ndt_localization::toString(metrics.decision));
  }
}

void LocalizationNode::publishScanDiagnostic(
  const builtin_interfaces::msg::Time & stamp,
  const ScanMetrics & metrics)
{
  if (!this->publish_scan_diagnostics_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = metrics.accepted ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.name = "ndt_localization/scan";
  status.hardware_id = "localization_node";
  status.message = ndt_localization::toString(metrics.decision);
  status.values = {
    keyValue("decision", status.message),
    keyValue("accepted", metrics.accepted ? "true" : "false"),
    keyValue("converged", metrics.converged ? "true" : "false"),
    keyValue(
      "state",
      ndt_localization::toString(this->state_machine_->state())),
    keyValue(
      "correction_valid",
      this->state_machine_->hasValidCorrection() ? "true" : "false"),
    numericKeyValue(
      "confirmation_count",
      this->state_machine_->confirmationCount()),
    numericKeyValue(
      "consecutive_rejections",
      this->state_machine_->consecutiveRejections()),
    numericKeyValue("conversion_ms", metrics.conversion_ms),
    numericKeyValue("local_map_ms", metrics.local_map_ms),
    numericKeyValue("matcher_ms", metrics.matcher_ms),
    numericKeyValue("validation_ms", metrics.validation_ms),
    numericKeyValue("total_ms", metrics.total_ms),
    numericKeyValue("input_age_ms", metrics.input_age_ms),
    numericKeyValue(
      "odometry_before_gap_ms", metrics.odometry_before_gap_ms),
    numericKeyValue(
      "odometry_after_gap_ms", metrics.odometry_after_gap_ms),
    numericKeyValue(
      "translation_delta_m", metrics.translation_delta_m),
    numericKeyValue(
      "rotation_delta_deg", metrics.rotation_delta_deg),
    numericKeyValue(
      "scan_points_raw", metrics.raw_scan_points),
    numericKeyValue(
      "scan_points_filtered", metrics.filtered_scan_points),
    numericKeyValue("map_points_raw", this->raw_map_points_),
    numericKeyValue("target_points", this->global_map_->size()),
    numericKeyValue("iterations", metrics.iterations),
    numericKeyValue("fitness_score", metrics.fitness_score),
  };
  message.status.push_back(std::move(status));
  this->diagnostic_pub_->publish(message);
}

void LocalizationNode::publishStateDiagnostic(
  const builtin_interfaces::msg::Time & stamp,
  ndt_localization::DecisionCode code)
{
  if (!this->publish_scan_diagnostics_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  const bool healthy =
    code == ndt_localization::DecisionCode::INITIALIZATION_STARTED ||
    code == ndt_localization::DecisionCode::INITIALIZATION_CONFIRMED ||
    code == ndt_localization::DecisionCode::TRACKING_ACCEPTED;
  const bool waiting =
    code ==
    ndt_localization::DecisionCode::INITIAL_POSE_WAITING_FOR_ODOMETRY;
  status.level = healthy ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    waiting ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.name = "ndt_localization/state";
  status.hardware_id = "localization_node";
  status.message = ndt_localization::toString(code);
  std::size_t buffer_size = 0;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    buffer_size = this->odometry_buffer_->size();
  }
  status.values = {
    keyValue("reason", status.message),
    keyValue(
      "state",
      ndt_localization::toString(this->state_machine_->state())),
    keyValue(
      "correction_valid",
      this->state_machine_->hasValidCorrection() ? "true" : "false"),
    numericKeyValue(
      "confirmation_count",
      this->state_machine_->confirmationCount()),
    numericKeyValue("odometry_buffer_samples", buffer_size),
  };
  message.status.push_back(std::move(status));
  this->diagnostic_pub_->publish(message);
  if (healthy || waiting) {
    RCLCPP_INFO(
      this->get_logger(), "State event: %s (%s)",
      ndt_localization::toString(code),
      ndt_localization::toString(this->state_machine_->state()));
  } else {
    RCLCPP_WARN(
      this->get_logger(), "State rejection: %s (%s)",
      ndt_localization::toString(code),
      ndt_localization::toString(this->state_machine_->state()));
  }
}

void LocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> scan_lock(this->scan_mutex_);
  if (!this->map_initialized_) {
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::MAP_UNAVAILABLE);
    return;
  }
  if (normalizedFrame(msg->header.frame_id) != this->global_frame_id_) {
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::INITIAL_POSE_FRAME_INVALID);
    return;
  }
  double age_ms = 0.0;
  const ndt_localization::DecisionCode timestamp_result =
    this->validateTimestamp(
    msg->header.stamp, this->maximum_initial_pose_age_seconds_,
    ndt_localization::DecisionCode::INITIAL_POSE_STAMP_INVALID,
    ndt_localization::DecisionCode::INITIAL_POSE_STALE,
    ndt_localization::DecisionCode::INITIAL_POSE_FUTURE,
    &age_ms);
  if (timestamp_result != ndt_localization::DecisionCode::NONE) {
    this->publishStateDiagnostic(msg->header.stamp, timestamp_result);
    return;
  }

  std::array<double, 36> covariance;
  std::copy(
    msg->pose.covariance.begin(), msg->pose.covariance.end(),
    covariance.begin());
  const Eigen::Vector3d position(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);
  const Eigen::Quaterniond orientation(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  ndt_localization::InitialPoseValidationLimits limits;
  limits.quaternion_norm_tolerance =
    this->quaternion_norm_tolerance_;
  limits.covariance_symmetry_tolerance =
    this->covariance_symmetry_tolerance_;
  limits.covariance_psd_tolerance =
    this->covariance_psd_tolerance_;
  limits.maximum_position_stddev_m =
    this->maximum_initial_position_stddev_m_;
  limits.maximum_yaw_stddev_deg =
    this->maximum_initial_yaw_stddev_deg_;
  const ndt_localization::ValidationResult pose_validation =
    ndt_localization::validateInitialPoseData(
    position, orientation, covariance, limits);
  if (!pose_validation.valid) {
    this->publishStateDiagnostic(
      msg->header.stamp, pose_validation.code);
    return;
  }

  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      this->maximum_odometry_interpolation_gap_seconds_);
  }
  const Eigen::Isometry3d initial_pose =
    poseToIsometry(msg->pose.pose);
  if (!synchronized_odometry.success &&
    (synchronized_odometry.code ==
    ndt_localization::DecisionCode::ODOMETRY_UNAVAILABLE ||
    synchronized_odometry.code ==
    ndt_localization::DecisionCode::ODOMETRY_TOO_NEW))
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      this->initial_pose_waiting_for_odometry_ = true;
      this->pending_initial_pose_stamp_ = msg->header.stamp;
      this->pending_initial_pose_ = initial_pose;
    }
    this->publishStateDiagnostic(
      msg->header.stamp,
      ndt_localization::DecisionCode::INITIAL_POSE_WAITING_FOR_ODOMETRY);
    return;
  }
  if (!synchronized_odometry.success) {
    this->publishStateDiagnostic(
      msg->header.stamp, synchronized_odometry.code);
    return;
  }
  this->beginInitialization(
    msg->header.stamp, initial_pose, synchronized_odometry.pose);
}

void LocalizationNode::beginInitialization(
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Isometry3d & initial_pose,
  const Eigen::Isometry3d & synchronized_odometry)
{
  const Eigen::Isometry3d prior_correction =
    initial_pose * synchronized_odometry.inverse();
  this->initialization_stamp_ns_ = rclcpp::Time(stamp).nanoseconds();
  this->state_machine_->beginInitialization(prior_correction);
  {
    std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
    this->initial_pose_waiting_for_odometry_ = false;
  }
  this->publishStateDiagnostic(
    stamp, ndt_localization::DecisionCode::INITIALIZATION_STARTED);
  RCLCPP_INFO(
    this->get_logger(),
    "Accepted initial-pose prior at %.6f; waiting for %d confirmations",
    rclcpp::Time(stamp).seconds(),
    this->initialization_confirmation_scans_);
}

void LocalizationNode::tryStartPendingInitialization()
{
  std::lock_guard<std::mutex> scan_lock(this->scan_mutex_);
  builtin_interfaces::msg::Time pending_stamp;
  Eigen::Isometry3d pending_pose = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
    if (!this->initial_pose_waiting_for_odometry_) {
      return;
    }
    pending_stamp = this->pending_initial_pose_stamp_;
    pending_pose = this->pending_initial_pose_;
  }
  const ndt_localization::DecisionCode timestamp_result =
    this->validateTimestamp(
    pending_stamp, this->maximum_initial_pose_age_seconds_,
    ndt_localization::DecisionCode::INITIAL_POSE_STAMP_INVALID,
    ndt_localization::DecisionCode::INITIAL_POSE_STALE,
    ndt_localization::DecisionCode::INITIAL_POSE_FUTURE,
    nullptr);
  if (timestamp_result != ndt_localization::DecisionCode::NONE) {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      if (rclcpp::Time(this->pending_initial_pose_stamp_) ==
        rclcpp::Time(pending_stamp))
      {
        this->initial_pose_waiting_for_odometry_ = false;
      }
    }
    this->publishStateDiagnostic(pending_stamp, timestamp_result);
    return;
  }
  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(pending_stamp).nanoseconds(),
      this->maximum_odometry_interpolation_gap_seconds_);
  }
  if (synchronized_odometry.success) {
    this->beginInitialization(
      pending_stamp,
      pending_pose,
      synchronized_odometry.pose);
  } else if (
    synchronized_odometry.code ==
    ndt_localization::DecisionCode::ODOMETRY_TOO_OLD)
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      if (rclcpp::Time(this->pending_initial_pose_stamp_) ==
        rclcpp::Time(pending_stamp))
      {
        this->initial_pose_waiting_for_odometry_ = false;
      }
    }
    this->publishStateDiagnostic(
      pending_stamp, synchronized_odometry.code);
  }
}

void LocalizationNode::processPendingScan()
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending;
  {
    std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
    if (!this->pending_scan_) {
      return;
    }
    pending = this->pending_scan_;
    this->pending_scan_.reset();
  }
  this->scanCallback(pending);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
