#include "localization.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Validated tracking matcher and deadline mechanics.
constexpr double kNdtStepSizeM = 0.1;
constexpr double kNdtTransformationEpsilon = 0.005;
constexpr int kNdtMaximumIterations = 15;
constexpr double kNdtScanLeafSizeM = 0.0;
constexpr std::size_t kMinimumLocalMapPoints = 1000;
constexpr double kDeadlineWatchdogMarginMs = 1.0;

// Input synchronization and numerical validation policy.
constexpr double kOdometryBufferDurationSeconds = 3.0;
constexpr std::size_t kOdometryBufferMaximumSamples = 500;
constexpr double kMaximumOdometryInterpolationGapSeconds = 0.15;
constexpr double kMaximumOdometryAgeSeconds = 0.5;
constexpr double kMaximumScanAgeSeconds = 0.5;
constexpr double kMaximumInitialPoseAgeSeconds = 1.0;
constexpr double kFutureToleranceSeconds = 0.1;
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr double kCovarianceSymmetryTolerance = 1.0e-6;
constexpr double kCovariancePsdTolerance = 1.0e-9;

// Initialization confirmation, search-envelope, and retry mechanics.
constexpr double kMaximumConfirmationTranslationDeltaM = 0.5;
constexpr double kMaximumConfirmationRotationDeltaDeg = 10.0;
constexpr double kInitializationConfirmationReserveMs = 350.0;
constexpr double kInitializationStandardDeviationMultiplier = 2.5;
constexpr double kInitializationMinimumTranslationSpanM = 1.0;
constexpr double kInitializationMinimumYawSpanDeg = 15.0;
constexpr double kRelocalizationRetryDelayMs = 500.0;

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

std::int64_t steadyNanoseconds(
  const std::chrono::steady_clock::time_point & time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    time.time_since_epoch()).count();
}

}  // namespace

LocalizationNode::LocalizationNode()
: Node("localization_node")
{
  RCLCPP_INFO(
    this->get_logger(), "Initializing FAST-LIO Localization Node ...");

  this->declare_parameter<std::string>("odom_frame_id", "odom_lidar");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("map_frame_id", "map");

  // Public deployment interface. Optimizer mechanics, numerical tolerances,
  // and queue policy are validated implementation constants above.
  this->declare_parameter<double>("localization.ndt_resolution", 1.0);
  this->declare_parameter<double>("localization.ndt_map_leaf_size", 0.15);
  this->declare_parameter<double>("localization.local_map_radius_m", 35.0);
  this->declare_parameter<int>(
    "localization.max_local_map_points", 120000);
  this->declare_parameter<int>("localization.max_scan_points", 4000);
  this->declare_parameter<double>(
    "localization.registration_deadline_ms", 80.0);
  this->declare_parameter<double>(
    "localization.max_result_translation_delta_m", 1.0);
  this->declare_parameter<double>(
    "localization.max_result_rotation_delta_deg", 20.0);
  this->declare_parameter<int>(
    "localization.max_consecutive_rejections", 10);
  this->declare_parameter<double>(
    "localization.initialization_timeout_ms", 2000.0);
  this->declare_parameter<double>(
    "localization.initialization_max_translation_span_m", 10.0);
  this->declare_parameter<double>(
    "localization.initialization_max_yaw_span_deg", 180.0);
  this->declare_parameter<int>(
    "localization.initialization_confirmation_scans", 3);
  this->declare_parameter<double>(
    "localization.initialization_max_fitness_score", 0.5);
  this->declare_parameter<double>(
    "localization.initialization_min_score_margin", 0.01);
  this->declare_parameter<double>(
    "localization.recovery_translation_span_m", 5.0);
  this->declare_parameter<double>(
    "localization.recovery_yaw_span_deg", 90.0);

  this->get_parameter("odom_frame_id", this->odom_frame_id_);
  this->get_parameter("base_frame_id", this->base_frame_id_);
  this->get_parameter("map_frame_id", this->global_frame_id_);
  this->get_parameter(
    "localization.ndt_resolution", this->ndt_resolution_);
  this->get_parameter(
    "localization.ndt_map_leaf_size", this->ndt_map_leaf_size_);
  this->get_parameter(
    "localization.local_map_radius_m", this->local_map_radius_m_);
  this->get_parameter(
    "localization.max_local_map_points",
    this->maximum_local_map_points_);
  this->get_parameter(
    "localization.max_scan_points", this->maximum_scan_points_);
  this->get_parameter(
    "localization.registration_deadline_ms",
    this->registration_deadline_ms_);
  this->get_parameter(
    "localization.max_result_translation_delta_m",
    this->maximum_result_translation_delta_m_);
  this->get_parameter(
    "localization.max_result_rotation_delta_deg",
    this->maximum_result_rotation_delta_deg_);
  this->get_parameter(
    "localization.max_consecutive_rejections",
    this->maximum_consecutive_rejections_);
  this->get_parameter(
    "localization.initialization_timeout_ms",
    this->initialization_timeout_ms_);
  this->get_parameter(
    "localization.initialization_max_translation_span_m",
    this->initialization_maximum_translation_span_m_);
  this->get_parameter(
    "localization.initialization_max_yaw_span_deg",
    this->initialization_maximum_yaw_span_deg_);
  this->get_parameter(
    "localization.initialization_confirmation_scans",
    this->initialization_confirmation_scans_);
  this->get_parameter(
    "localization.initialization_max_fitness_score",
    this->initialization_maximum_fitness_score_);
  this->get_parameter(
    "localization.initialization_min_score_margin",
    this->initialization_minimum_score_margin_);
  this->get_parameter(
    "localization.recovery_translation_span_m",
    this->recovery_translation_span_m_);
  this->get_parameter(
    "localization.recovery_yaw_span_deg",
    this->recovery_yaw_span_deg_);

  this->global_frame_id_ = normalizedFrame(this->global_frame_id_);
  this->odom_frame_id_ = normalizedFrame(this->odom_frame_id_);
  this->base_frame_id_ = normalizedFrame(this->base_frame_id_);
  if (this->global_frame_id_.empty() ||
    this->odom_frame_id_.empty() ||
    this->base_frame_id_.empty())
  {
    throw std::invalid_argument("localization frame IDs cannot be empty");
  }
  ndt_localization::RobustInitializer::Config initializer_config;
  if (this->initialization_confirmation_scans_ < 1 ||
    this->maximum_result_translation_delta_m_ <= 0.0 ||
    this->maximum_result_rotation_delta_deg_ <= 0.0 ||
    this->maximum_consecutive_rejections_ < 1 ||
    this->ndt_resolution_ <= 0.0 ||
    this->ndt_map_leaf_size_ < 0.0 ||
    this->local_map_radius_m_ <= 0.0 ||
    this->maximum_local_map_points_ < 1 ||
    static_cast<std::size_t>(this->maximum_local_map_points_) <
    kMinimumLocalMapPoints ||
    this->maximum_scan_points_ < 1 ||
    this->registration_deadline_ms_ <= kDeadlineWatchdogMarginMs ||
    this->initialization_timeout_ms_ <=
    kInitializationConfirmationReserveMs +
    initializer_config.refinement_reserve_ms ||
    this->initialization_maximum_translation_span_m_ <
    kInitializationMinimumTranslationSpanM ||
    this->initialization_maximum_yaw_span_deg_ <
    kInitializationMinimumYawSpanDeg ||
    this->recovery_translation_span_m_ <= 0.0 ||
    this->recovery_yaw_span_deg_ <= 0.0 ||
    this->initialization_maximum_fitness_score_ < 0.0 ||
    this->initialization_minimum_score_margin_ < 0.0)
  {
    throw std::invalid_argument("invalid localization parameters");
  }

  this->odometry_buffer_ =
    std::make_unique<ndt_localization::OdometryBuffer>(
    kOdometryBufferDurationSeconds, kOdometryBufferMaximumSamples);
  this->state_machine_ =
    std::make_unique<ndt_localization::LocalizationStateMachine>(
    static_cast<std::size_t>(this->initialization_confirmation_scans_));
  initializer_config.local_map_radius_m = this->local_map_radius_m_;
  initializer_config.maximum_local_map_points =
    static_cast<std::size_t>(this->maximum_local_map_points_);
  initializer_config.minimum_local_map_points = kMinimumLocalMapPoints;
  initializer_config.refinement_scan_leaf_size_m = kNdtScanLeafSizeM;
  initializer_config.maximum_refinement_scan_points =
    static_cast<std::size_t>(this->maximum_scan_points_);
  initializer_config.refinement_resolution_m = this->ndt_resolution_;
  initializer_config.refinement_step_size_m = kNdtStepSizeM;
  initializer_config.refinement_transformation_epsilon =
    kNdtTransformationEpsilon;
  initializer_config.refinement_maximum_iterations =
    kNdtMaximumIterations;
  initializer_config.maximum_fitness_score =
    this->initialization_maximum_fitness_score_;
  initializer_config.minimum_score_margin =
    this->initialization_minimum_score_margin_;
  this->robust_initializer_ =
    std::make_unique<ndt_localization::RobustInitializer>(
    initializer_config);
  this->tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->map_kdtree_.reset(new pcl::KdTreeFLANN<PointType>(true));

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
  rclcpp::QoS scan_qos(1);
  scan_qos.reliable();
  scan_qos.durability_volatile();
  this->scan_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered_body", scan_qos,
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
  this->relocalization_service_ =
    this->create_service<std_srvs::srv::Trigger>(
    "/localization/trigger_relocalization",
    std::bind(
      &LocalizationNode::triggerRelocalizationCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  this->registration_worker_ =
    std::thread(&LocalizationNode::registrationWorkerLoop, this);
  this->initialization_worker_ =
    std::thread(&LocalizationNode::initializationWorkerLoop, this);
  this->deadline_worker_ =
    std::thread(&LocalizationNode::deadlineWorkerLoop, this);
}

LocalizationNode::~LocalizationNode()
{
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    this->stop_registration_threads_ = true;
  }
  this->registration_cv_.notify_all();
  if (this->registration_worker_.joinable()) {
    this->registration_worker_.join();
  }
  if (this->initialization_worker_.joinable()) {
    this->initialization_worker_.join();
  }
  if (this->deadline_worker_.joinable()) {
    this->deadline_worker_.join();
  }
}

void LocalizationNode::mapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (this->map_initialized_.load(std::memory_order_acquire)) {
    return;
  }
  if (normalizedFrame(msg->header.frame_id) != this->global_frame_id_) {
    RCLCPP_ERROR(
      this->get_logger(), "Rejected map with frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), this->global_frame_id_.c_str());
    return;
  }
  if (msg->width == 0 || msg->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Rejected empty global map");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received global map with %u points",
    msg->width * msg->height);
  pcl::PointCloud<PointType>::Ptr map(
    new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *map);
  this->raw_map_points_ = map->size();
  this->global_map_ = ndt_localization::voxelDownsample(
    map, this->ndt_map_leaf_size_);
  if (this->global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Rejected map with no usable points");
    return;
  }

  this->ndt_.setResolution(this->ndt_resolution_);
  this->ndt_.setStepSize(kNdtStepSizeM);
  this->ndt_.setTransformationEpsilon(kNdtTransformationEpsilon);
  this->ndt_.setMaximumIterations(kNdtMaximumIterations);
  this->map_kdtree_->setInputCloud(this->global_map_);
  this->robust_initializer_->setMap(this->global_map_);
  this->map_initialized_.store(true, std::memory_order_release);
  RCLCPP_INFO(
    this->get_logger(),
    "NDT map index initialized with %zu points (%zu before filtering)",
    this->global_map_->size(), this->raw_map_points_);
}

bool LocalizationNode::validTimestamp(
  const builtin_interfaces::msg::Time & stamp,
  double maximum_age_seconds) const
{
  if (stamp.sec <= 0 || stamp.nanosec >= 1000000000u) {
    return false;
  }
  const rclcpp::Time message_time(stamp);
  return ndt_localization::validTimestampNanoseconds(
    message_time.nanoseconds(), this->now().nanoseconds(),
    maximum_age_seconds, kFutureToleranceSeconds);
}

void LocalizationNode::odomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (normalizedFrame(msg->header.frame_id) != this->odom_frame_id_ ||
    normalizedFrame(msg->child_frame_id) != this->base_frame_id_)
  {
    return;
  }
  if (!this->validTimestamp(
      msg->header.stamp, kMaximumOdometryAgeSeconds))
  {
    return;
  }
  if (!validPose(msg->pose.pose, kQuaternionNormTolerance)) {
    return;
  }

  const Eigen::Isometry3d odom_to_base =
    poseToIsometry(msg->pose.pose);
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    this->odometry_buffer_->add(
      rclcpp::Time(msg->header.stamp).nanoseconds(), odom_to_base);
  }
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    ++this->odometry_sequence_;
  }
  this->registration_cv_.notify_all();
  this->tryStartPendingInitialization();

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
  const auto received_at = std::chrono::steady_clock::now();

  if (normalizedFrame(msg->header.frame_id) != this->base_frame_id_) {
    this->rejectScan();
    return;
  }
  if (!this->validTimestamp(msg->header.stamp, kMaximumScanAgeSeconds)) {
    this->rejectScan();
    return;
  }
  if (!this->map_initialized_.load(std::memory_order_acquire)) {
    this->rejectScan();
    return;
  }
  if (msg->width == 0 || msg->height == 0) {
    this->rejectScan();
    return;
  }

  ndt_localization::LocalizationState state =
    this->state_machine_->state();
  if (state == ndt_localization::LocalizationState::UNINITIALIZED) {
    this->rejectScan();
    return;
  }
  if (state == ndt_localization::LocalizationState::LOST) {
    if (!this->startRelocalization(msg->header.stamp)) {
      this->rejectScan();
      return;
    }
    state = this->state_machine_->state();
  }
  if (state == ndt_localization::LocalizationState::INITIALIZING ||
    state == ndt_localization::LocalizationState::RELOCALIZING)
  {
    if (!ndt_localization::initializationScanFollowsPrior(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      this->initialization_stamp_ns_.load(std::memory_order_acquire)))
    {
      this->rejectScan();
      return;
    }
    bool search_required = false;
    {
      std::lock_guard<std::mutex> lock(this->registration_mutex_);
      search_required =
        this->initialization_attempt_active_ &&
        this->initialization_search_required_;
    }
    if (search_required) {
      this->enqueueInitializationScan(msg);
    } else {
      this->enqueueScan(msg, received_at);
    }
    return;
  }
  this->enqueueScan(msg, received_at);
}

void LocalizationNode::rejectScan()
{
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    this->invalidateRegistrationWorkLocked();
    this->applyRejectionStateLocked(false);
  }
  this->registration_cv_.notify_all();
}

bool LocalizationNode::applyRejectionStateLocked(
  bool registration_rejection)
{
  const ndt_localization::LocalizationState state =
    this->state_machine_->state();
  if (state == ndt_localization::LocalizationState::INITIALIZING ||
    state == ndt_localization::LocalizationState::RELOCALIZING)
  {
    this->state_machine_->rejectInitializationCandidate();
    return false;
  }
  if (registration_rejection &&
    state == ndt_localization::LocalizationState::TRACKING)
  {
    return this->state_machine_->recordTrackingRejection(
      static_cast<std::size_t>(
        this->maximum_consecutive_rejections_));
  }
  return false;
}

void LocalizationNode::enqueueScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  const std::chrono::steady_clock::time_point & received_at)
{
  std::shared_ptr<ScanTask> task = std::make_shared<ScanTask>();
  task->message = msg;
  task->received_at = received_at;
  task->deadline = received_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(
      this->registration_deadline_ms_ -
      kDeadlineWatchdogMarginMs));
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    task->generation = ++this->latest_scan_generation_;
    const auto supersede =
      [this](const std::shared_ptr<ScanTask> & old_task)
      {
        if (!old_task || old_task->decided) {
          return;
        }
        old_task->decided = true;
        this->applyRejectionStateLocked(false);
      };
    supersede(this->pending_scan_task_);
    supersede(this->active_scan_task_);
    this->pending_scan_task_ = task;
  }
  this->registration_cv_.notify_all();
}

void LocalizationNode::enqueueInitializationScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::shared_ptr<InitializationTask> task =
    std::make_shared<InitializationTask>();
  task->message = msg;
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (this->initialization_attempt_active_ &&
      this->initialization_search_required_)
    {
      task->generation = this->initialization_generation_;
      task->bounds = this->initialization_search_bounds_;
      task->recovery = this->initialization_recovery_;
      task->search_deadline =
        this->initialization_attempt_deadline_ -
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(
          kInitializationConfirmationReserveMs));
      this->pending_initialization_task_ = task;
      queued = true;
    }
  }

  if (queued) {
    this->registration_cv_.notify_all();
  }
}

void
LocalizationNode::invalidateRegistrationWorkLocked()
{
  ++this->latest_scan_generation_;
  const auto invalidate =
    [](const std::shared_ptr<ScanTask> & task)
    {
      if (!task || task->decided) {
        return;
      }
      task->decided = true;
    };
  invalidate(this->pending_scan_task_);
  invalidate(this->active_scan_task_);
  this->pending_scan_task_.reset();
}

void LocalizationNode::registrationWorkerLoop()
{
  while (true) {
    std::shared_ptr<ScanTask> task;
    {
      std::unique_lock<std::mutex> lock(this->registration_mutex_);
      this->registration_cv_.wait(
        lock,
        [this]()
        {
          return this->stop_registration_threads_ ||
                 (this->pending_scan_task_ &&
                  (!this->pending_scan_task_->waiting_for_odometry ||
                   this->odometry_sequence_ >=
                   this->pending_scan_task_->required_odometry_sequence));
        });
      if (this->stop_registration_threads_) {
        return;
      }
      task = this->pending_scan_task_;
      this->pending_scan_task_.reset();
      this->active_scan_task_ = task;
    }
    this->processScanTask(task);
  }
}

void LocalizationNode::initializationWorkerLoop()
{
  while (true) {
    std::shared_ptr<InitializationTask> task;
    {
      std::unique_lock<std::mutex> lock(this->registration_mutex_);
      this->registration_cv_.wait(
        lock,
        [this]()
        {
          return this->stop_registration_threads_ ||
                 this->pending_initialization_task_ != nullptr;
        });
      if (this->stop_registration_threads_) {
        return;
      }
      task = this->pending_initialization_task_;
      this->pending_initialization_task_.reset();
      this->active_initialization_task_ = task;
    }
    this->processInitializationTask(task);
  }
}

void LocalizationNode::processInitializationTask(
  const std::shared_ptr<InitializationTask> & task)
{
  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(task->message->header.stamp).nanoseconds(),
      kMaximumOdometryInterpolationGapSeconds);
  }
  if (synchronized_odometry.status !=
    ndt_localization::OdometryStatus::AVAILABLE)
  {
    {
      std::lock_guard<std::mutex> lock(this->registration_mutex_);
      if (this->active_initialization_task_ == task) {
        this->active_initialization_task_.reset();
      }
    }
    this->registration_cv_.notify_all();
    return;
  }

  Eigen::Isometry3d prior_correction = Eigen::Isometry3d::Identity();
  bool current = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    current =
      this->initialization_attempt_active_ &&
      this->initialization_search_required_ &&
      task->generation == this->initialization_generation_;
    if (current) {
      prior_correction = this->state_machine_->pendingCorrection();
    }
  }
  if (!current) {
    {
      std::lock_guard<std::mutex> lock(this->registration_mutex_);
      if (this->active_initialization_task_ == task) {
        this->active_initialization_task_.reset();
      }
    }
    return;
  }

  pcl::PointCloud<PointType>::Ptr scan(
    new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*task->message, *scan);
  ndt_localization::RobustInitializer::Request request;
  request.scan = scan;
  request.prior_pose = prior_correction * synchronized_odometry.pose;
  request.bounds = task->bounds;
  request.deadline = task->search_deadline;
  Eigen::Isometry3d selected_pose = Eigen::Isometry3d::Identity();
  const bool search_succeeded =
    this->robust_initializer_->search(request, &selected_pose);
  const Eigen::Isometry3d candidate_correction =
    selected_pose * synchronized_odometry.pose.inverse();
  const bool candidate_valid =
    search_succeeded && ndt_localization::validTransformCandidate(
    request.prior_pose, selected_pose,
    task->bounds.translation_span_m +
    this->maximum_result_translation_delta_m_,
    task->bounds.yaw_span_deg +
    this->maximum_result_rotation_delta_deg_);
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (!this->initialization_attempt_active_ ||
      task->generation != this->initialization_generation_ ||
      std::chrono::steady_clock::now() >=
      this->initialization_attempt_deadline_)
    {
      // Superseded search result.
    } else if (candidate_valid &&
      this->state_machine_->setInitializationCandidate(
        candidate_correction))
    {
      this->initialization_search_required_ = false;
      this->pending_initialization_task_.reset();
      this->initialization_stamp_ns_.store(
        rclcpp::Time(task->message->header.stamp).nanoseconds(),
        std::memory_order_release);
    } else {
      this->initialization_attempt_active_ = false;
      this->initialization_search_required_ = false;
      this->pending_initialization_task_.reset();
      ++this->initialization_generation_;
      this->state_machine_->failInitializationAttempt();
      if (task->recovery) {
        this->next_relocalization_attempt_ =
          std::chrono::steady_clock::now() +
          std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(
              kRelocalizationRetryDelayMs));
      }
    }
    if (this->active_initialization_task_ == task) {
      this->active_initialization_task_.reset();
    }
  }
  this->registration_cv_.notify_all();
}

void LocalizationNode::finalizeRejectedTask(
  const std::shared_ptr<ScanTask> & task,
  bool registration_rejection)
{
  bool entered_lost = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (task->decided) {
      if (this->active_scan_task_ == task) {
        this->active_scan_task_.reset();
      }
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (ndt_localization::deadlineExpired(
        steadyNanoseconds(task->received_at), steadyNanoseconds(now),
        this->registration_deadline_ms_))
    {
      registration_rejection = true;
    } else if (task->generation != this->latest_scan_generation_) {
      registration_rejection = false;
    }
    task->decided = true;
    if (this->active_scan_task_ == task) {
      this->active_scan_task_.reset();
    }
    if (this->pending_scan_task_ == task) {
      this->pending_scan_task_.reset();
    }
    entered_lost =
      this->applyRejectionStateLocked(registration_rejection);
  }
  this->registration_cv_.notify_all();
  if (entered_lost) {
    this->startRelocalization(task->message->header.stamp);
  }
}

LocalizationNode::RegistrationInputStatus
LocalizationNode::prepareRegistrationInput(
  const std::shared_ptr<ScanTask> & task,
  const Eigen::Isometry3d & correction_guess,
  const ndt_localization::OdometryLookup & synchronized_odometry,
  RegistrationInput * input)
{
  pcl::PointCloud<PointType>::Ptr converted_scan(
    new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*task->message, *converted_scan);
  input->scan = ndt_localization::voxelDownsample(
    converted_scan, kNdtScanLeafSizeM);
  input->scan = ndt_localization::deterministicallyCap(
    input->scan, static_cast<std::size_t>(this->maximum_scan_points_));
  if (input->scan->empty()) {
    return RegistrationInputStatus::INVALID_SCAN;
  }

  input->pose_guess =
    correction_guess * synchronized_odometry.pose;
  input->target = ndt_localization::radiusSubmap(
    this->global_map_, this->map_kdtree_,
    input->pose_guess.translation(), this->local_map_radius_m_,
    static_cast<std::size_t>(this->maximum_local_map_points_));
  if (input->target->size() < kMinimumLocalMapPoints) {
    return RegistrationInputStatus::REGISTRATION_REJECTED;
  }
  if (std::chrono::steady_clock::now() >= task->deadline) {
    return RegistrationInputStatus::REGISTRATION_REJECTED;
  }
  return RegistrationInputStatus::READY;
}

bool LocalizationNode::runRegistration(
  const RegistrationInput & input,
  Eigen::Isometry3d * optimized_pose)
{
  this->ndt_.setInputTarget(input.target);
  this->ndt_.setInputSource(input.scan);
  pcl::PointCloud<PointType> aligned_cloud;
  this->ndt_.align(
    aligned_cloud, input.pose_guess.matrix().cast<float>());
  if (!this->ndt_.hasConverged()) {
    return false;
  }
  optimized_pose->matrix() =
    this->ndt_.getFinalTransformation().cast<double>();
  return true;
}

void LocalizationNode::commitRegistrationResult(
  const std::shared_ptr<ScanTask> & task,
  const Eigen::Isometry3d & candidate_correction)
{
  bool entered_lost = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    const auto now = std::chrono::steady_clock::now();
    const bool public_deadline_expired =
      ndt_localization::deadlineExpired(
      steadyNanoseconds(task->received_at), steadyNanoseconds(now),
      this->registration_deadline_ms_);
    const ndt_localization::LocalizationState decision_state =
      this->state_machine_->state();
    const bool is_initializing =
      decision_state == ndt_localization::LocalizationState::INITIALIZING;
    const bool is_relocalizing =
      decision_state == ndt_localization::LocalizationState::RELOCALIZING;
    const bool is_tracking =
      decision_state == ndt_localization::LocalizationState::TRACKING;
    if (task->decided) {
      // A newer task or the watchdog already decided this task.
    } else if (public_deadline_expired) {
      task->decided = true;
      entered_lost = this->applyRejectionStateLocked(true);
    } else if (task->generation != this->latest_scan_generation_) {
      task->decided = true;
      this->applyRejectionStateLocked(false);
    } else if (is_initializing || is_relocalizing) {
      const ndt_localization::InitializationObservation observation =
        this->state_machine_->observeInitializationCorrection(
        candidate_correction,
        kMaximumConfirmationTranslationDeltaM,
        kMaximumConfirmationRotationDeltaDeg);
      task->decided = true;
      if (observation.confirmed) {
        this->initialization_attempt_active_ = false;
        this->initialization_search_required_ = false;
        this->pending_initialization_task_.reset();
        ++this->initialization_generation_;
      }
    } else if (is_tracking) {
      task->decided = true;
      this->state_machine_->applyTrackingCorrection(
        candidate_correction);
    } else {
      task->decided = true;
    }
    if (this->active_scan_task_ == task) {
      this->active_scan_task_.reset();
    }
  }
  this->registration_cv_.notify_all();
  if (entered_lost) {
    this->startRelocalization(task->message->header.stamp);
  }
}

void LocalizationNode::processScanTask(
  const std::shared_ptr<ScanTask> & task)
{
  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(task->message->header.stamp).nanoseconds(),
      kMaximumOdometryInterpolationGapSeconds);
  }
  if (synchronized_odometry.status ==
    ndt_localization::OdometryStatus::TOO_NEW)
  {
    bool requeued = false;
    {
      std::lock_guard<std::mutex> lock(this->registration_mutex_);
      if (!task->decided &&
        task->generation == this->latest_scan_generation_ &&
        std::chrono::steady_clock::now() < task->deadline)
      {
        task->waiting_for_odometry = true;
        task->required_odometry_sequence = this->odometry_sequence_ + 1;
        if (this->active_scan_task_ == task) {
          this->active_scan_task_.reset();
        }
        this->pending_scan_task_ = task;
        requeued = true;
      }
    }
    if (requeued) {
      this->registration_cv_.notify_all();
      return;
    }
  }
  if (synchronized_odometry.status !=
    ndt_localization::OdometryStatus::AVAILABLE)
  {
    this->finalizeRejectedTask(task, false);
    return;
  }

  ndt_localization::LocalizationState processing_state;
  Eigen::Isometry3d correction_guess = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (task->decided) {
      if (this->active_scan_task_ == task) {
        this->active_scan_task_.reset();
      }
      return;
    }
    processing_state = this->state_machine_->state();
    correction_guess =
      (processing_state ==
      ndt_localization::LocalizationState::INITIALIZING ||
      processing_state ==
      ndt_localization::LocalizationState::RELOCALIZING) ?
      this->state_machine_->pendingCorrection() :
      this->state_machine_->correction();
  }
  if (processing_state ==
    ndt_localization::LocalizationState::UNINITIALIZED)
  {
    this->finalizeRejectedTask(task, false);
    return;
  }
  if (processing_state == ndt_localization::LocalizationState::LOST) {
    this->finalizeRejectedTask(task, false);
    return;
  }

  RegistrationInput registration_input;
  const RegistrationInputStatus preparation_result =
    this->prepareRegistrationInput(
    task, correction_guess, synchronized_odometry,
    &registration_input);
  if (preparation_result != RegistrationInputStatus::READY) {
    const bool registration_rejection =
      preparation_result ==
      RegistrationInputStatus::REGISTRATION_REJECTED;
    this->finalizeRejectedTask(task, registration_rejection);
    return;
  }

  Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
  const bool converged = this->runRegistration(
    registration_input, &optimized_pose);

  bool task_already_decided = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (task->decided) {
      if (this->active_scan_task_ == task) {
        this->active_scan_task_.reset();
      }
      task_already_decided = true;
    }
  }
  if (task_already_decided) {
    this->registration_cv_.notify_all();
    return;
  }
  if (!converged) {
    this->finalizeRejectedTask(task, true);
    return;
  }

  if (!ndt_localization::validTransformCandidate(
    registration_input.pose_guess, optimized_pose,
    this->maximum_result_translation_delta_m_,
    this->maximum_result_rotation_delta_deg_))
  {
    this->finalizeRejectedTask(task, true);
    return;
  }
  const Eigen::Isometry3d candidate_correction =
    optimized_pose * synchronized_odometry.pose.inverse();
  this->commitRegistrationResult(task, candidate_correction);
}

void LocalizationNode::deadlineWorkerLoop()
{
  while (true) {
    std::vector<std::pair<std::shared_ptr<ScanTask>, bool>> expired;
    {
      std::unique_lock<std::mutex> lock(this->registration_mutex_);
      if (this->stop_registration_threads_) {
        return;
      }
      std::chrono::steady_clock::time_point earliest =
        std::chrono::steady_clock::time_point::max();
      for (const auto & task :
        {this->pending_scan_task_, this->active_scan_task_})
      {
        if (task && !task->decided) {
          earliest = std::min(earliest, task->deadline);
        }
      }
      if (this->initialization_attempt_active_) {
        earliest = std::min(
          earliest, this->initialization_attempt_deadline_);
      }
      if (earliest == std::chrono::steady_clock::time_point::max()) {
        this->registration_cv_.wait(lock);
        continue;
      }
      if (std::chrono::steady_clock::now() < earliest) {
        this->registration_cv_.wait_until(lock, earliest);
        continue;
      }
      const auto expire =
        [this, &expired](std::shared_ptr<ScanTask> & task)
        {
          if (!task || task->decided ||
            std::chrono::steady_clock::now() < task->deadline)
          {
            return;
          }
          task->decided = true;
          const bool entered_lost =
            this->applyRejectionStateLocked(true);
          expired.emplace_back(task, entered_lost);
          task.reset();
        };
      expire(this->pending_scan_task_);
      expire(this->active_scan_task_);
      if (this->initialization_attempt_active_ &&
        std::chrono::steady_clock::now() >=
        this->initialization_attempt_deadline_)
      {
        const bool recovery_expired = this->initialization_recovery_;
        this->initialization_attempt_active_ = false;
        this->initialization_search_required_ = false;
        ++this->initialization_generation_;
        this->pending_initialization_task_.reset();
        this->active_initialization_task_.reset();
        this->state_machine_->failInitializationAttempt();
        this->invalidateRegistrationWorkLocked();
        if (recovery_expired) {
          this->next_relocalization_attempt_ =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(
              kRelocalizationRetryDelayMs));
        }
      }
    }
    this->registration_cv_.notify_all();
    for (const auto & item : expired) {
      if (item.second) {
        this->startRelocalization(
          item.first->message->header.stamp);
      }
    }
  }
}

void LocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!this->map_initialized_.load(std::memory_order_acquire)) {
    return;
  }
  if (normalizedFrame(msg->header.frame_id) != this->global_frame_id_) {
    return;
  }
  if (!this->validTimestamp(
      msg->header.stamp, kMaximumInitialPoseAgeSeconds))
  {
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
    kQuaternionNormTolerance;
  limits.covariance_symmetry_tolerance =
    kCovarianceSymmetryTolerance;
  limits.covariance_psd_tolerance =
    kCovariancePsdTolerance;
  limits.maximum_position_stddev_m =
    this->initialization_maximum_translation_span_m_;
  limits.maximum_yaw_stddev_deg =
    this->initialization_maximum_yaw_span_deg_;
  if (!ndt_localization::validInitialPoseData(
      position, orientation, covariance, limits))
  {
    return;
  }
  const double position_stddev_m = std::sqrt(
    std::max(
      covariance[0],
      std::max(covariance[7], covariance[14])));
  const double yaw_stddev_deg =
    std::sqrt(covariance[35]) * 180.0 / kPi;
  const ndt_localization::InitializationSearchBounds search_bounds =
    ndt_localization::initializationSearchBounds(
    position_stddev_m,
    yaw_stddev_deg,
    kInitializationStandardDeviationMultiplier,
    kInitializationMinimumTranslationSpanM,
    this->initialization_maximum_translation_span_m_,
    kInitializationMinimumYawSpanDeg,
    this->initialization_maximum_yaw_span_deg_);

  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      kMaximumOdometryInterpolationGapSeconds);
  }
  const Eigen::Isometry3d initial_pose =
    poseToIsometry(msg->pose.pose);
  if (synchronized_odometry.status ==
    ndt_localization::OdometryStatus::UNAVAILABLE ||
    synchronized_odometry.status ==
    ndt_localization::OdometryStatus::TOO_NEW)
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      this->initial_pose_waiting_for_odometry_ = true;
      this->pending_initial_pose_stamp_ = msg->header.stamp;
      this->pending_initial_pose_ = initial_pose;
      this->pending_initialization_search_bounds_ = search_bounds;
    }
    return;
  }
  if (synchronized_odometry.status !=
    ndt_localization::OdometryStatus::AVAILABLE)
  {
    return;
  }
  this->beginInitialization(
    msg->header.stamp, initial_pose, synchronized_odometry.pose,
    search_bounds);
}

void LocalizationNode::beginInitialization(
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Isometry3d & initial_pose,
  const Eigen::Isometry3d & synchronized_odometry,
  const ndt_localization::InitializationSearchBounds & bounds)
{
  const Eigen::Isometry3d constrained_initial_pose =
    ndt_localization::gravityConstrainedPose(
    initial_pose, synchronized_odometry);
  const Eigen::Isometry3d prior_correction =
    constrained_initial_pose * synchronized_odometry.inverse();
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    this->invalidateRegistrationWorkLocked();
    this->initialization_stamp_ns_.store(
      rclcpp::Time(stamp).nanoseconds(), std::memory_order_release);
    ++this->initialization_generation_;
    this->initialization_attempt_active_ = true;
    this->initialization_search_required_ = true;
    this->initialization_recovery_ = false;
    this->initialization_search_bounds_ = bounds;
    this->initialization_attempt_deadline_ =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(
        this->initialization_timeout_ms_));
    this->pending_initialization_task_.reset();
    this->active_initialization_task_.reset();
    this->state_machine_->beginInitialization(prior_correction);
  }
  {
    std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
    this->initial_pose_waiting_for_odometry_ = false;
  }
  this->registration_cv_.notify_all();
  RCLCPP_INFO(
    this->get_logger(),
    "Accepted initial-pose prior at %.6f; searching %.2f m / %.1f deg "
    "before %d confirmations",
    rclcpp::Time(stamp).seconds(),
    bounds.translation_span_m, bounds.yaw_span_deg,
    this->initialization_confirmation_scans_);
}

bool LocalizationNode::startRelocalization(
  const builtin_interfaces::msg::Time & stamp,
  bool ignore_retry_delay)
{
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!ignore_retry_delay && now < this->next_relocalization_attempt_) {
      return false;
    }
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    if (!this->state_machine_->getValidCorrection(&correction)) {
      return false;
    }
    this->invalidateRegistrationWorkLocked();
    ++this->initialization_generation_;
    this->initialization_attempt_active_ = true;
    this->initialization_search_required_ = true;
    this->initialization_recovery_ = true;
    this->initialization_search_bounds_.translation_span_m =
      this->recovery_translation_span_m_;
    this->initialization_search_bounds_.yaw_span_deg =
      this->recovery_yaw_span_deg_;
    this->initialization_attempt_deadline_ =
      now +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(
        this->initialization_timeout_ms_));
    this->initialization_stamp_ns_.store(
      rclcpp::Time(stamp).nanoseconds(), std::memory_order_release);
    this->pending_initialization_task_.reset();
    this->active_initialization_task_.reset();
    this->state_machine_->beginRelocalization(correction);
  }
  this->registration_cv_.notify_all();
  return true;
}

void LocalizationNode::triggerRelocalizationCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!this->state_machine_->hasValidCorrection()) {
    response->success = false;
    response->message = "no validated correction is available";
    return;
  }
  this->state_machine_->enterLost();
  const builtin_interfaces::msg::Time stamp = this->now();
  response->success = this->startRelocalization(stamp, true);
  response->message = response->success ?
    "bounded background relocalization started" :
    "no validated correction is available";
}

void LocalizationNode::tryStartPendingInitialization()
{
  builtin_interfaces::msg::Time pending_stamp;
  Eigen::Isometry3d pending_pose;
  ndt_localization::InitializationSearchBounds pending_bounds;
  {
    std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
    if (!this->initial_pose_waiting_for_odometry_) {
      return;
    }
    pending_stamp = this->pending_initial_pose_stamp_;
    pending_pose = this->pending_initial_pose_;
    pending_bounds = this->pending_initialization_search_bounds_;
  }
  if (!this->validTimestamp(
      pending_stamp, kMaximumInitialPoseAgeSeconds))
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      if (rclcpp::Time(this->pending_initial_pose_stamp_) ==
        rclcpp::Time(pending_stamp))
      {
        this->initial_pose_waiting_for_odometry_ = false;
      }
    }
    return;
  }
  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(pending_stamp).nanoseconds(),
      kMaximumOdometryInterpolationGapSeconds);
  }
  if (synchronized_odometry.status ==
    ndt_localization::OdometryStatus::AVAILABLE)
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      if (!this->initial_pose_waiting_for_odometry_ ||
        rclcpp::Time(this->pending_initial_pose_stamp_) !=
        rclcpp::Time(pending_stamp))
      {
        return;
      }
      this->initial_pose_waiting_for_odometry_ = false;
    }
    this->beginInitialization(
      pending_stamp,
      pending_pose,
      synchronized_odometry.pose,
      pending_bounds);
  } else if (
    synchronized_odometry.status ==
    ndt_localization::OdometryStatus::TOO_OLD)
  {
    {
      std::lock_guard<std::mutex> pending_lock(this->pending_mutex_);
      if (rclcpp::Time(this->pending_initial_pose_stamp_) ==
        rclcpp::Time(pending_stamp))
      {
        this->initial_pose_waiting_for_odometry_ = false;
      }
    }
  }
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
