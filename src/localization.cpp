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

constexpr double kPi = 3.14159265358979323846;

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

double durationMilliseconds(
  const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
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

  this->declare_parameter<double>("localization.ndt_resolution", 1.0);
  this->declare_parameter<double>("localization.ndt_step_size", 0.1);
  this->declare_parameter<double>("localization.ndt_trans_epsilon", 0.005);
  this->declare_parameter<int>("localization.ndt_max_iter", 15);
  this->declare_parameter<double>("localization.ndt_map_leaf_size", 0.15);
  this->declare_parameter<double>("localization.ndt_scan_leaf_size", 0.0);
  this->declare_parameter<double>("localization.local_map_radius_m", 35.0);
  this->declare_parameter<int>(
    "localization.max_local_map_points", 120000);
  this->declare_parameter<int>(
    "localization.min_local_map_points", 1000);
  this->declare_parameter<int>("localization.max_scan_points", 4000);
  this->declare_parameter<double>(
    "localization.registration_deadline_ms", 80.0);
  this->declare_parameter<double>(
    "localization.deadline_watchdog_margin_ms", 1.0);
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
  this->declare_parameter<double>(
    "localization.initialization_timeout_ms", 2000.0);
  this->declare_parameter<double>(
    "localization.initialization_confirmation_reserve_ms", 350.0);
  this->declare_parameter<double>(
    "localization.initialization_stddev_multiplier", 2.5);
  this->declare_parameter<double>(
    "localization.initialization_min_translation_span_m", 1.0);
  this->declare_parameter<double>(
    "localization.initialization_max_translation_span_m", 10.0);
  this->declare_parameter<double>(
    "localization.initialization_min_yaw_span_deg", 15.0);
  this->declare_parameter<double>(
    "localization.initialization_max_yaw_span_deg", 180.0);
  this->declare_parameter<double>(
    "localization.recovery_translation_span_m", 5.0);
  this->declare_parameter<double>(
    "localization.recovery_yaw_span_deg", 90.0);
  this->declare_parameter<double>(
    "localization.relocalization_retry_delay_ms", 500.0);
  this->declare_parameter<int>(
    "localization.initialization_max_hypotheses", 65);
  this->declare_parameter<double>(
    "localization.initialization_coarse_map_leaf_size_m", 0.5);
  this->declare_parameter<double>(
    "localization.initialization_coarse_scan_leaf_size_m", 0.4);
  this->declare_parameter<int>(
    "localization.initialization_max_coarse_scan_points", 2000);
  this->declare_parameter<double>(
    "localization.initialization_coarse_resolution_m", 2.0);
  this->declare_parameter<double>(
    "localization.initialization_coarse_step_size_m", 0.2);
  this->declare_parameter<double>(
    "localization.initialization_coarse_trans_epsilon", 0.05);
  this->declare_parameter<int>(
    "localization.initialization_coarse_max_iter", 8);
  this->declare_parameter<int>(
    "localization.initialization_refinement_candidates", 3);
  this->declare_parameter<double>(
    "localization.initialization_refinement_reserve_ms", 250.0);
  this->declare_parameter<double>(
    "localization.initialization_fitness_max_range_m", 2.0);
  this->declare_parameter<double>(
    "localization.initialization_max_fitness_score", 0.5);
  this->declare_parameter<double>(
    "localization.initialization_min_score_margin", 0.01);
  this->declare_parameter<double>(
    "localization.initialization_distinct_translation_m", 0.5);
  this->declare_parameter<double>(
    "localization.initialization_distinct_rotation_deg", 5.0);

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
    "localization.local_map_radius_m", this->local_map_radius_m_);
  this->get_parameter(
    "localization.max_local_map_points",
    this->maximum_local_map_points_);
  this->get_parameter(
    "localization.min_local_map_points",
    this->minimum_local_map_points_);
  this->get_parameter(
    "localization.max_scan_points", this->maximum_scan_points_);
  this->get_parameter(
    "localization.registration_deadline_ms",
    this->registration_deadline_ms_);
  this->get_parameter(
    "localization.deadline_watchdog_margin_ms",
    this->deadline_watchdog_margin_ms_);
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
  this->get_parameter(
    "localization.initialization_timeout_ms",
    this->initialization_timeout_ms_);
  this->get_parameter(
    "localization.initialization_confirmation_reserve_ms",
    this->initialization_confirmation_reserve_ms_);
  this->get_parameter(
    "localization.initialization_stddev_multiplier",
    this->initialization_stddev_multiplier_);
  this->get_parameter(
    "localization.initialization_min_translation_span_m",
    this->initialization_minimum_translation_span_m_);
  this->get_parameter(
    "localization.initialization_max_translation_span_m",
    this->initialization_maximum_translation_span_m_);
  this->get_parameter(
    "localization.initialization_min_yaw_span_deg",
    this->initialization_minimum_yaw_span_deg_);
  this->get_parameter(
    "localization.initialization_max_yaw_span_deg",
    this->initialization_maximum_yaw_span_deg_);
  this->get_parameter(
    "localization.recovery_translation_span_m",
    this->recovery_translation_span_m_);
  this->get_parameter(
    "localization.recovery_yaw_span_deg",
    this->recovery_yaw_span_deg_);
  this->get_parameter(
    "localization.relocalization_retry_delay_ms",
    this->relocalization_retry_delay_ms_);
  this->get_parameter(
    "localization.initialization_max_hypotheses",
    this->initialization_maximum_hypotheses_);
  this->get_parameter(
    "localization.initialization_coarse_map_leaf_size_m",
    this->initialization_coarse_map_leaf_size_m_);
  this->get_parameter(
    "localization.initialization_coarse_scan_leaf_size_m",
    this->initialization_coarse_scan_leaf_size_m_);
  this->get_parameter(
    "localization.initialization_max_coarse_scan_points",
    this->initialization_maximum_coarse_scan_points_);
  this->get_parameter(
    "localization.initialization_coarse_resolution_m",
    this->initialization_coarse_resolution_m_);
  this->get_parameter(
    "localization.initialization_coarse_step_size_m",
    this->initialization_coarse_step_size_m_);
  this->get_parameter(
    "localization.initialization_coarse_trans_epsilon",
    this->initialization_coarse_transformation_epsilon_);
  this->get_parameter(
    "localization.initialization_coarse_max_iter",
    this->initialization_coarse_maximum_iterations_);
  this->get_parameter(
    "localization.initialization_refinement_candidates",
    this->initialization_refinement_candidates_);
  this->get_parameter(
    "localization.initialization_refinement_reserve_ms",
    this->initialization_refinement_reserve_ms_);
  this->get_parameter(
    "localization.initialization_fitness_max_range_m",
    this->initialization_fitness_max_range_m_);
  this->get_parameter(
    "localization.initialization_max_fitness_score",
    this->initialization_maximum_fitness_score_);
  this->get_parameter(
    "localization.initialization_min_score_margin",
    this->initialization_minimum_score_margin_);
  this->get_parameter(
    "localization.initialization_distinct_translation_m",
    this->initialization_distinct_translation_m_);
  this->get_parameter(
    "localization.initialization_distinct_rotation_deg",
    this->initialization_distinct_rotation_deg_);

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
    this->maximum_consecutive_rejections_ < 1 ||
    this->ndt_max_iter_ < 1 ||
    this->ndt_map_leaf_size_ < 0.0 ||
    this->ndt_scan_leaf_size_ < 0.0 ||
    this->local_map_radius_m_ <= 0.0 ||
    this->maximum_local_map_points_ < 1 ||
    this->minimum_local_map_points_ < 1 ||
    this->minimum_local_map_points_ > this->maximum_local_map_points_ ||
    this->maximum_scan_points_ < 1 ||
    this->registration_deadline_ms_ <= 0.0 ||
    this->deadline_watchdog_margin_ms_ < 0.0 ||
    this->deadline_watchdog_margin_ms_ >=
    this->registration_deadline_ms_ ||
    this->initialization_timeout_ms_ <= 0.0 ||
    this->initialization_confirmation_reserve_ms_ < 0.0 ||
    this->initialization_refinement_reserve_ms_ < 0.0 ||
    this->initialization_confirmation_reserve_ms_ +
    this->initialization_refinement_reserve_ms_ >=
    this->initialization_timeout_ms_ ||
    this->initialization_stddev_multiplier_ <= 0.0 ||
    this->initialization_minimum_translation_span_m_ < 0.0 ||
    this->initialization_maximum_translation_span_m_ <
    this->initialization_minimum_translation_span_m_ ||
    this->initialization_minimum_yaw_span_deg_ < 0.0 ||
    this->initialization_maximum_yaw_span_deg_ <
    this->initialization_minimum_yaw_span_deg_ ||
    this->recovery_translation_span_m_ <= 0.0 ||
    this->recovery_yaw_span_deg_ <= 0.0 ||
    this->relocalization_retry_delay_ms_ < 0.0 ||
    this->initialization_maximum_hypotheses_ < 1 ||
    this->initialization_coarse_map_leaf_size_m_ < 0.0 ||
    this->initialization_coarse_scan_leaf_size_m_ < 0.0 ||
    this->initialization_maximum_coarse_scan_points_ < 1 ||
    this->initialization_coarse_resolution_m_ <= 0.0 ||
    this->initialization_coarse_step_size_m_ <= 0.0 ||
    this->initialization_coarse_transformation_epsilon_ <= 0.0 ||
    this->initialization_coarse_maximum_iterations_ < 1 ||
    this->initialization_refinement_candidates_ < 1 ||
    this->initialization_fitness_max_range_m_ <= 0.0 ||
    this->initialization_maximum_fitness_score_ < 0.0 ||
    this->initialization_minimum_score_margin_ < 0.0 ||
    this->initialization_distinct_translation_m_ <= 0.0 ||
    this->initialization_distinct_rotation_deg_ <= 0.0)
  {
    throw std::invalid_argument("invalid localization parameters");
  }

  this->odometry_buffer_ =
    std::make_unique<ndt_localization::OdometryBuffer>(
    this->odometry_buffer_duration_seconds_,
    static_cast<std::size_t>(this->odometry_buffer_max_samples_));
  this->state_machine_ =
    std::make_unique<ndt_localization::LocalizationStateMachine>(
    static_cast<std::size_t>(this->initialization_confirmation_scans_));
  ndt_localization::RobustInitializer::Config initializer_config;
  initializer_config.local_map_radius_m = this->local_map_radius_m_;
  initializer_config.maximum_local_map_points =
    static_cast<std::size_t>(this->maximum_local_map_points_);
  initializer_config.minimum_local_map_points =
    static_cast<std::size_t>(this->minimum_local_map_points_);
  initializer_config.maximum_hypotheses =
    static_cast<std::size_t>(this->initialization_maximum_hypotheses_);
  initializer_config.coarse_map_leaf_size_m =
    this->initialization_coarse_map_leaf_size_m_;
  initializer_config.coarse_scan_leaf_size_m =
    this->initialization_coarse_scan_leaf_size_m_;
  initializer_config.maximum_coarse_scan_points =
    static_cast<std::size_t>(
    this->initialization_maximum_coarse_scan_points_);
  initializer_config.coarse_resolution_m =
    this->initialization_coarse_resolution_m_;
  initializer_config.coarse_step_size_m =
    this->initialization_coarse_step_size_m_;
  initializer_config.coarse_transformation_epsilon =
    this->initialization_coarse_transformation_epsilon_;
  initializer_config.coarse_maximum_iterations =
    this->initialization_coarse_maximum_iterations_;
  initializer_config.refinement_scan_leaf_size_m =
    this->ndt_scan_leaf_size_;
  initializer_config.maximum_refinement_scan_points =
    static_cast<std::size_t>(this->maximum_scan_points_);
  initializer_config.refinement_resolution_m = this->ndt_resolution_;
  initializer_config.refinement_step_size_m = this->ndt_step_size_;
  initializer_config.refinement_transformation_epsilon =
    this->ndt_trans_epsilon_;
  initializer_config.refinement_maximum_iterations = this->ndt_max_iter_;
  initializer_config.refinement_candidates =
    static_cast<std::size_t>(this->initialization_refinement_candidates_);
  initializer_config.refinement_reserve_ms =
    this->initialization_refinement_reserve_ms_;
  initializer_config.fitness_max_range_m =
    this->initialization_fitness_max_range_m_;
  initializer_config.maximum_fitness_score =
    this->initialization_maximum_fitness_score_;
  initializer_config.minimum_score_margin =
    this->initialization_minimum_score_margin_;
  initializer_config.distinct_translation_m =
    this->initialization_distinct_translation_m_;
  initializer_config.distinct_rotation_deg =
    this->initialization_distinct_rotation_deg_;
  this->robust_initializer_ =
    std::make_unique<ndt_localization::RobustInitializer>(
    initializer_config);
  this->tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->global_map_.reset(new pcl::PointCloud<PointType>());
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
  this->diagnostic_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/localization/scan_diagnostics", 10);
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
  this->map_kdtree_->setInputCloud(this->global_map_);
  this->robust_initializer_->setMap(this->global_map_);
  this->map_initialized_.store(true, std::memory_order_release);
  RCLCPP_INFO(
    this->get_logger(),
    "NDT map index initialized with %zu points (%zu before filtering)",
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

pcl::PointCloud<PointType>::Ptr LocalizationNode::capCloud(
  const pcl::PointCloud<PointType>::Ptr & cloud,
  std::size_t maximum_points)
{
  if (cloud->size() <= maximum_points) {
    return cloud;
  }
  pcl::PointCloud<PointType>::Ptr capped(
    new pcl::PointCloud<PointType>());
  capped->reserve(maximum_points);
  capped->is_dense = cloud->is_dense;
  for (const std::size_t index :
    ndt_localization::deterministicSampleIndices(
      cloud->size(), maximum_points))
  {
    capped->push_back((*cloud)[index]);
  }
  return capped;
}

pcl::PointCloud<PointType>::Ptr LocalizationNode::buildLocalMap(
  const Eigen::Vector3d & center,
  ScanMetrics * metrics)
{
  const auto start = std::chrono::steady_clock::now();
  PointType query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = static_cast<float>(center.z());
  std::vector<int> indices;
  std::vector<float> squared_distances;
  this->map_kdtree_->radiusSearch(
    query, this->local_map_radius_m_, indices, squared_distances,
    static_cast<unsigned int>(this->maximum_local_map_points_));

  pcl::PointCloud<PointType>::Ptr local_map(
    new pcl::PointCloud<PointType>());
  local_map->reserve(indices.size());
  local_map->is_dense = this->global_map_->is_dense;
  for (const int index : indices) {
    local_map->push_back(
      (*this->global_map_)[static_cast<std::size_t>(index)]);
  }
  metrics->target_points = local_map->size();
  metrics->local_map_ms = elapsedMilliseconds(start);
  return local_map;
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
  ScanMetrics metrics;
  metrics.raw_scan_points =
    static_cast<std::size_t>(msg->width) * msg->height;

  if (normalizedFrame(msg->header.frame_id) != this->base_frame_id_) {
    this->publishImmediateScanRejection(
      msg, metrics, ndt_localization::DecisionCode::SCAN_FRAME_INVALID);
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
    this->publishImmediateScanRejection(msg, metrics, timestamp_result);
    return;
  }
  if (!this->map_initialized_.load(std::memory_order_acquire)) {
    this->publishImmediateScanRejection(
      msg, metrics, ndt_localization::DecisionCode::MAP_UNAVAILABLE);
    return;
  }
  if (metrics.raw_scan_points == 0) {
    this->publishImmediateScanRejection(
      msg, metrics, ndt_localization::DecisionCode::SCAN_EMPTY);
    return;
  }

  ndt_localization::LocalizationState state =
    this->state_machine_->state();
  if (state == ndt_localization::LocalizationState::UNINITIALIZED) {
    this->publishImmediateScanRejection(
      msg, metrics, ndt_localization::DecisionCode::STATE_UNINITIALIZED);
    return;
  }
  if (state == ndt_localization::LocalizationState::LOST) {
    if (!this->startRelocalization(msg->header.stamp)) {
      this->publishImmediateScanRejection(
        msg, metrics, ndt_localization::DecisionCode::STATE_LOST);
      return;
    }
    state = this->state_machine_->state();
  }
  if (state == ndt_localization::LocalizationState::INITIALIZING ||
    state == ndt_localization::LocalizationState::RELOCALIZING)
  {
    const ndt_localization::ValidationResult sequence_validation =
      ndt_localization::validateInitializationScanTimestamp(
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      this->initialization_stamp_ns_.load(std::memory_order_acquire));
    if (!sequence_validation.valid) {
      this->publishImmediateScanRejection(
        msg, metrics, sequence_validation.code);
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
      this->enqueueInitializationScan(msg, received_at);
    } else {
      this->enqueueScan(msg, received_at);
    }
    return;
  }
  this->enqueueScan(msg, received_at);
}

void LocalizationNode::publishImmediateScanRejection(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  ScanMetrics metrics,
  ndt_localization::DecisionCode code)
{
  bool entered_lost = false;
  std::vector<std::shared_ptr<ScanTask>> invalidated;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    invalidated = this->invalidateRegistrationWorkLocked();
    entered_lost = this->applyRejectionStateLocked(false);
  }
  this->publishSupersededTasks(invalidated);
  this->registration_cv_.notify_all();
  metrics.decision = code;
  metrics.total_ms = 0.0;
  this->publishScanDiagnostic(msg->header.stamp, metrics);
  if (entered_lost) {
    this->publishStateDiagnostic(
      msg->header.stamp, ndt_localization::DecisionCode::TRACKING_LOST);
  }
}

bool LocalizationNode::applyRejectionStateLocked(
  bool registration_rejection)
{
  if (this->state_machine_->state() ==
    ndt_localization::LocalizationState::INITIALIZING ||
    this->state_machine_->state() ==
    ndt_localization::LocalizationState::RELOCALIZING)
  {
    this->state_machine_->rejectInitializationCandidate();
    return false;
  }
  if (registration_rejection &&
    this->state_machine_->state() ==
    ndt_localization::LocalizationState::TRACKING)
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
      this->deadline_watchdog_margin_ms_));
  std::vector<std::shared_ptr<ScanTask>> superseded;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    task->generation = ++this->latest_scan_generation_;
    const auto supersede =
      [this, &superseded](const std::shared_ptr<ScanTask> & old_task)
      {
        if (!old_task || old_task->decided) {
          return;
        }
        old_task->decided = true;
        old_task->decision =
          ndt_localization::DecisionCode::SCAN_SUPERSEDED;
        superseded.push_back(old_task);
        this->applyRejectionStateLocked(false);
      };
    supersede(this->pending_scan_task_);
    supersede(this->active_scan_task_);
    this->pending_scan_task_ = task;
  }
  this->publishSupersededTasks(superseded);
  this->registration_cv_.notify_all();
}

void LocalizationNode::enqueueInitializationScan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  const std::chrono::steady_clock::time_point & received_at)
{
  std::shared_ptr<InitializationTask> task =
    std::make_shared<InitializationTask>();
  task->message = msg;
  task->received_at = received_at;
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
          this->initialization_confirmation_reserve_ms_));
      this->pending_initialization_task_ = task;
      queued = true;
    }
  }

  ScanMetrics metrics;
  metrics.raw_scan_points =
    static_cast<std::size_t>(msg->width) * msg->height;
  metrics.generation = task->generation;
  metrics.input_age_ms =
    (this->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
  metrics.total_ms = elapsedMilliseconds(received_at);
  metrics.decision = queued ?
    ndt_localization::DecisionCode::INITIALIZATION_SEARCH_PENDING :
    ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
  this->publishScanDiagnostic(msg->header.stamp, metrics);
  if (queued) {
    this->registration_cv_.notify_all();
  }
}

void LocalizationNode::publishSupersededTasks(
  const std::vector<std::shared_ptr<ScanTask>> & tasks)
{
  for (const auto & task : tasks) {
    ScanMetrics metrics;
    metrics.decision = task->decision;
    metrics.generation = task->generation;
    metrics.raw_scan_points =
      static_cast<std::size_t>(task->message->width) *
      task->message->height;
    metrics.total_ms = elapsedMilliseconds(task->received_at);
    metrics.queue_wait_ms =
      task->started_at == std::chrono::steady_clock::time_point() ?
      metrics.total_ms :
      durationMilliseconds(task->received_at, task->started_at);
    metrics.input_age_ms =
      (this->now() -
      rclcpp::Time(task->message->header.stamp)).seconds() * 1000.0;
    this->publishScanDiagnostic(task->message->header.stamp, metrics);
  }
}

std::vector<std::shared_ptr<LocalizationNode::ScanTask>>
LocalizationNode::invalidateRegistrationWorkLocked()
{
  ++this->latest_scan_generation_;
  std::vector<std::shared_ptr<ScanTask>> invalidated;
  const auto invalidate =
    [&invalidated](const std::shared_ptr<ScanTask> & task)
    {
      if (!task || task->decided) {
        return;
      }
      task->decided = true;
      task->decision =
        ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
      invalidated.push_back(task);
    };
  invalidate(this->pending_scan_task_);
  invalidate(this->active_scan_task_);
  this->pending_scan_task_.reset();
  return invalidated;
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
      if (task->started_at ==
        std::chrono::steady_clock::time_point())
      {
        task->started_at = std::chrono::steady_clock::now();
      }
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
  InitializationMetrics metrics;
  metrics.generation = task->generation;
  metrics.recovery = task->recovery;
  metrics.translation_span_m = task->bounds.translation_span_m;
  metrics.yaw_span_deg = task->bounds.yaw_span_deg;

  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(task->message->header.stamp).nanoseconds(),
      this->maximum_odometry_interpolation_gap_seconds_);
  }
  if (!synchronized_odometry.success) {
    {
      std::lock_guard<std::mutex> lock(this->registration_mutex_);
      if (this->active_initialization_task_ == task) {
        this->active_initialization_task_.reset();
      }
    }
    metrics.decision = synchronized_odometry.code;
    metrics.total_ms = elapsedMilliseconds(task->received_at);
    this->publishInitializationDiagnostic(
      task->message->header.stamp, metrics);
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
    metrics.decision =
      ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
    metrics.total_ms = elapsedMilliseconds(task->received_at);
    this->publishInitializationDiagnostic(
      task->message->header.stamp, metrics);
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
  const ndt_localization::RobustInitializer::Result result =
    this->robust_initializer_->search(request);
  metrics.success = result.success;
  metrics.ambiguous = result.ambiguous;
  metrics.hypotheses = result.hypotheses;
  metrics.evaluated = result.evaluated;
  metrics.converged = result.converged;
  metrics.refined = result.refined;
  metrics.scan_points = result.scan_points;
  metrics.target_points = result.target_points;
  metrics.best_score = result.best_score;
  metrics.second_score = result.second_score;
  metrics.score_margin = result.score_margin;
  metrics.coarse_ms = result.coarse_ms;
  metrics.refinement_ms = result.refinement_ms;
  metrics.total_ms = elapsedMilliseconds(task->received_at);

  const Eigen::Isometry3d candidate_correction =
    result.pose * synchronized_odometry.pose.inverse();
  const ndt_localization::TransformValidation candidate_validation =
    ndt_localization::validateTransformCandidate(
    request.prior_pose, result.pose,
    task->bounds.translation_span_m +
    this->maximum_result_translation_delta_m_,
    task->bounds.yaw_span_deg +
    this->maximum_result_rotation_delta_deg_);
  bool selected = false;
  bool failed = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (!this->initialization_attempt_active_ ||
      task->generation != this->initialization_generation_ ||
      std::chrono::steady_clock::now() >=
      this->initialization_attempt_deadline_)
    {
      metrics.decision =
        ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
    } else if (result.success && candidate_validation.valid &&
      this->state_machine_->setInitializationCandidate(
        candidate_correction))
    {
      selected = true;
      this->initialization_search_required_ = false;
      this->pending_initialization_task_.reset();
      this->initialization_stamp_ns_.store(
        rclcpp::Time(task->message->header.stamp).nanoseconds(),
        std::memory_order_release);
      metrics.decision = task->recovery ?
        ndt_localization::DecisionCode::RELOCALIZATION_HYPOTHESIS_SELECTED :
        ndt_localization::DecisionCode::INITIALIZATION_HYPOTHESIS_SELECTED;
    } else {
      failed = true;
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
            this->relocalization_retry_delay_ms_));
      }
      if (!candidate_validation.valid && result.success) {
        metrics.decision = candidate_validation.code;
      } else if (task->recovery) {
        if (result.ambiguous) {
          metrics.decision =
            ndt_localization::DecisionCode::RELOCALIZATION_AMBIGUOUS;
        } else if (result.timed_out) {
          metrics.decision =
            ndt_localization::DecisionCode::RELOCALIZATION_SEARCH_TIMEOUT;
        } else {
          metrics.decision =
            ndt_localization::DecisionCode::RELOCALIZATION_SEARCH_FAILED;
        }
      } else {
        metrics.decision = result.code;
      }
    }
    if (this->active_initialization_task_ == task) {
      this->active_initialization_task_.reset();
    }
  }
  this->registration_cv_.notify_all();
  metrics.success = selected;
  this->publishInitializationDiagnostic(
    task->message->header.stamp, metrics);
  if (selected || failed) {
    this->publishStateDiagnostic(
      task->message->header.stamp, metrics.decision);
  }
}

bool LocalizationNode::finalizeRejectedTask(
  const std::shared_ptr<ScanTask> & task,
  ScanMetrics metrics,
  ndt_localization::DecisionCode code,
  bool registration_rejection)
{
  bool entered_lost = false;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    if (task->decided) {
      if (this->active_scan_task_ == task) {
        this->active_scan_task_.reset();
      }
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (ndt_localization::deadlineExpired(
        steadyNanoseconds(task->received_at), steadyNanoseconds(now),
        this->registration_deadline_ms_))
    {
      code = ndt_localization::DecisionCode::REGISTRATION_TIMEOUT;
      registration_rejection = true;
      metrics.deadline_exceeded = true;
    } else if (task->generation != this->latest_scan_generation_) {
      code = ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
      registration_rejection = false;
    }
    task->decided = true;
    task->decision = code;
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
  metrics.decision = code;
  metrics.generation = task->generation;
  metrics.total_ms = elapsedMilliseconds(task->received_at);
  metrics.input_age_ms =
    (this->now() -
    rclcpp::Time(task->message->header.stamp)).seconds() * 1000.0;
  this->publishScanDiagnostic(task->message->header.stamp, metrics);
  if (entered_lost) {
    this->publishStateDiagnostic(
      task->message->header.stamp,
      ndt_localization::DecisionCode::TRACKING_LOST);
    this->startRelocalization(task->message->header.stamp);
  }
  return true;
}

void LocalizationNode::processScanTask(
  const std::shared_ptr<ScanTask> & task)
{
  ScanMetrics metrics;
  metrics.generation = task->generation;
  metrics.raw_scan_points =
    static_cast<std::size_t>(task->message->width) *
    task->message->height;
  metrics.queue_wait_ms =
    durationMilliseconds(task->received_at, task->started_at);

  ndt_localization::OdometryLookup synchronized_odometry;
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    synchronized_odometry = this->odometry_buffer_->lookup(
      rclcpp::Time(task->message->header.stamp).nanoseconds(),
      this->maximum_odometry_interpolation_gap_seconds_);
  }
  metrics.odometry_before_gap_ms =
    synchronized_odometry.before_gap_seconds * 1000.0;
  metrics.odometry_after_gap_ms =
    synchronized_odometry.after_gap_seconds * 1000.0;
  if (!synchronized_odometry.success &&
    synchronized_odometry.code ==
    ndt_localization::DecisionCode::ODOMETRY_TOO_NEW)
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
  if (!synchronized_odometry.success) {
    this->finalizeRejectedTask(
      task, metrics, synchronized_odometry.code, false);
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
    this->finalizeRejectedTask(
      task, metrics, ndt_localization::DecisionCode::STATE_UNINITIALIZED,
      false);
    return;
  }
  if (processing_state == ndt_localization::LocalizationState::LOST) {
    this->finalizeRejectedTask(
      task, metrics, ndt_localization::DecisionCode::STATE_LOST, false);
    return;
  }

  const auto conversion_start = std::chrono::steady_clock::now();
  pcl::PointCloud<PointType>::Ptr scan(
    new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*task->message, *scan);
  scan = this->downsampleCloud(scan, this->ndt_scan_leaf_size_);
  metrics.filtered_scan_points = scan->size();
  scan = this->capCloud(
    scan, static_cast<std::size_t>(this->maximum_scan_points_));
  metrics.capped_scan_points = scan->size();
  metrics.conversion_ms = elapsedMilliseconds(conversion_start);
  if (scan->empty()) {
    this->finalizeRejectedTask(
      task, metrics, ndt_localization::DecisionCode::SCAN_EMPTY, false);
    return;
  }

  const Eigen::Isometry3d pose_guess =
    correction_guess * synchronized_odometry.pose;
  pcl::PointCloud<PointType>::Ptr local_map =
    this->buildLocalMap(pose_guess.translation(), &metrics);
  if (local_map->size() <
    static_cast<std::size_t>(this->minimum_local_map_points_))
  {
    this->finalizeRejectedTask(
      task, metrics,
      ndt_localization::DecisionCode::LOCAL_MAP_INSUFFICIENT, true);
    return;
  }
  if (std::chrono::steady_clock::now() >= task->deadline) {
    this->finalizeRejectedTask(
      task, metrics,
      ndt_localization::DecisionCode::REGISTRATION_TIMEOUT, true);
    return;
  }

  const auto matcher_start = std::chrono::steady_clock::now();
  this->ndt_.setInputTarget(local_map);
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
    this->publishLateResultDiagnostic(
      *task, metrics.matcher_ms,
      elapsedMilliseconds(task->received_at));
    return;
  }
  if (!metrics.converged) {
    const bool published = this->finalizeRejectedTask(
      task, metrics,
      ndt_localization::DecisionCode::MATCHER_NOT_CONVERGED, true);
    if (published &&
      task->decision ==
      ndt_localization::DecisionCode::REGISTRATION_TIMEOUT)
    {
      this->publishLateResultDiagnostic(
        *task, metrics.matcher_ms,
        elapsedMilliseconds(task->received_at));
    }
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
    const bool published = this->finalizeRejectedTask(
      task, metrics, pose_validation.code, true);
    if (published &&
      task->decision ==
      ndt_localization::DecisionCode::REGISTRATION_TIMEOUT)
    {
      this->publishLateResultDiagnostic(
        *task, metrics.matcher_ms,
        elapsedMilliseconds(task->received_at));
    }
    return;
  }
  const Eigen::Isometry3d candidate_correction =
    optimized_pose * synchronized_odometry.pose.inverse();
  bool publish_scan = false;
  bool publish_late = false;
  bool entered_lost = false;
  bool initialization_confirmed = false;
  ndt_localization::DecisionCode confirmation_code =
    ndt_localization::DecisionCode::NONE;
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
      publish_late = true;
    } else if (public_deadline_expired) {
      task->decided = true;
      task->decision =
        ndt_localization::DecisionCode::REGISTRATION_TIMEOUT;
      metrics.deadline_exceeded = true;
      entered_lost = this->applyRejectionStateLocked(true);
      publish_scan = true;
      publish_late = true;
    } else if (task->generation != this->latest_scan_generation_) {
      task->decided = true;
      task->decision =
        ndt_localization::DecisionCode::RESULT_GENERATION_STALE;
      this->applyRejectionStateLocked(false);
      publish_scan = true;
      publish_late = true;
    } else if (is_initializing || is_relocalizing) {
      const ndt_localization::InitializationObservation observation =
        this->state_machine_->observeInitializationCorrection(
        candidate_correction,
        this->maximum_confirmation_translation_delta_m_,
        this->maximum_confirmation_rotation_delta_deg_);
      task->decided = true;
      task->decision = observation.code;
      metrics.translation_delta_m = observation.translation_delta_m;
      metrics.rotation_delta_deg = observation.rotation_delta_deg;
      metrics.accepted = observation.confirmed;
      initialization_confirmed = observation.confirmed;
      confirmation_code = observation.code;
      if (observation.confirmed) {
        this->initialization_attempt_active_ = false;
        this->initialization_search_required_ = false;
        this->pending_initialization_task_.reset();
        ++this->initialization_generation_;
      }
      publish_scan = true;
    } else if (is_tracking) {
      task->decided = true;
      if (this->state_machine_->applyTrackingCorrection(
          candidate_correction))
      {
        task->decision =
          ndt_localization::DecisionCode::TRACKING_ACCEPTED;
        metrics.accepted = true;
      } else {
        task->decision = ndt_localization::DecisionCode::STATE_LOST;
      }
      publish_scan = true;
    } else {
      task->decided = true;
      task->decision = ndt_localization::DecisionCode::STATE_LOST;
      publish_scan = true;
    }
    if (this->active_scan_task_ == task) {
      this->active_scan_task_.reset();
    }
  }
  this->registration_cv_.notify_all();
  metrics.validation_ms = elapsedMilliseconds(validation_start);
  metrics.decision = task->decision;
  metrics.total_ms = elapsedMilliseconds(task->received_at);
  metrics.input_age_ms =
    (this->now() -
    rclcpp::Time(task->message->header.stamp)).seconds() * 1000.0;
  if (publish_scan) {
    this->publishScanDiagnostic(task->message->header.stamp, metrics);
  }
  if (publish_late) {
    this->publishLateResultDiagnostic(
      *task, metrics.matcher_ms, metrics.total_ms);
  }
  if (initialization_confirmed) {
    this->publishStateDiagnostic(
      task->message->header.stamp,
      confirmation_code);
  }
  if (entered_lost) {
    this->publishStateDiagnostic(
      task->message->header.stamp,
      ndt_localization::DecisionCode::TRACKING_LOST);
    this->startRelocalization(task->message->header.stamp);
  }

  if (this->ndt_log_runtime_) {
    const std::string generation_string =
      std::to_string(metrics.generation);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "NDT generation %s: %.3f ms, points: %zu -> %zu -> %zu, "
      "target: %zu, state: %s, decision: %s",
      generation_string.c_str(),
      metrics.matcher_ms, metrics.raw_scan_points,
      metrics.filtered_scan_points, metrics.capped_scan_points,
      metrics.target_points,
      ndt_localization::toString(this->state_machine_->state()),
      ndt_localization::toString(metrics.decision));
  }
}

void LocalizationNode::deadlineWorkerLoop()
{
  while (true) {
    std::vector<std::pair<std::shared_ptr<ScanTask>, bool>> expired;
    std::vector<std::shared_ptr<ScanTask>> invalidated;
    bool initialization_expired = false;
    bool recovery_expired = false;
    InitializationMetrics initialization_metrics;
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
          task->decision =
            ndt_localization::DecisionCode::REGISTRATION_TIMEOUT;
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
        initialization_expired = true;
        recovery_expired = this->initialization_recovery_;
        initialization_metrics.generation =
          this->initialization_generation_;
        initialization_metrics.recovery = recovery_expired;
        initialization_metrics.translation_span_m =
          this->initialization_search_bounds_.translation_span_m;
        initialization_metrics.yaw_span_deg =
          this->initialization_search_bounds_.yaw_span_deg;
        initialization_metrics.total_ms = this->initialization_timeout_ms_;
        initialization_metrics.decision = recovery_expired ?
          ndt_localization::DecisionCode::RELOCALIZATION_SEARCH_TIMEOUT :
          ndt_localization::DecisionCode::INITIALIZATION_SEARCH_TIMEOUT;
        this->initialization_attempt_active_ = false;
        this->initialization_search_required_ = false;
        ++this->initialization_generation_;
        this->pending_initialization_task_.reset();
        this->active_initialization_task_.reset();
        this->state_machine_->failInitializationAttempt();
        invalidated = this->invalidateRegistrationWorkLocked();
        if (recovery_expired) {
          this->next_relocalization_attempt_ =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(
              this->relocalization_retry_delay_ms_));
        }
      }
    }
    this->registration_cv_.notify_all();
    this->publishSupersededTasks(invalidated);
    if (initialization_expired) {
      const builtin_interfaces::msg::Time stamp = this->now();
      this->publishInitializationDiagnostic(
        stamp, initialization_metrics);
      this->publishStateDiagnostic(
        stamp, initialization_metrics.decision);
    }
    for (const auto & item : expired) {
      const auto & task = item.first;
      ScanMetrics metrics;
      metrics.decision =
        ndt_localization::DecisionCode::REGISTRATION_TIMEOUT;
      metrics.generation = task->generation;
      metrics.deadline_exceeded =
        ndt_localization::deadlineExpired(
        steadyNanoseconds(task->received_at),
        steadyNanoseconds(std::chrono::steady_clock::now()),
        this->registration_deadline_ms_);
      metrics.raw_scan_points =
        static_cast<std::size_t>(task->message->width) *
        task->message->height;
      metrics.total_ms = elapsedMilliseconds(task->received_at);
      metrics.queue_wait_ms =
        task->started_at == std::chrono::steady_clock::time_point() ?
        metrics.total_ms :
        durationMilliseconds(task->received_at, task->started_at);
      metrics.input_age_ms =
        (this->now() -
        rclcpp::Time(task->message->header.stamp)).seconds() * 1000.0;
      this->publishScanDiagnostic(
        task->message->header.stamp, metrics);
      if (item.second) {
        this->publishStateDiagnostic(
          task->message->header.stamp,
          ndt_localization::DecisionCode::TRACKING_LOST);
        this->startRelocalization(task->message->header.stamp);
      }
    }
  }
}

void LocalizationNode::publishLateResultDiagnostic(
  const ScanTask & task,
  double matcher_ms,
  double completion_ms)
{
  if (!this->publish_scan_diagnostics_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = task.message->header.stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.name = "ndt_localization/late_result";
  status.hardware_id = "localization_node";
  status.message =
    ndt_localization::toString(
    ndt_localization::DecisionCode::RESULT_GENERATION_STALE);
  status.values = {
    keyValue(
      "reason", ndt_localization::toString(task.decision)),
    numericKeyValue("generation", task.generation),
    numericKeyValue("matcher_ms", matcher_ms),
    numericKeyValue("completion_ms", completion_ms),
  };
  message.status.push_back(std::move(status));
  this->diagnostic_pub_->publish(message);
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
    numericKeyValue("queue_wait_ms", metrics.queue_wait_ms),
    numericKeyValue("input_age_ms", metrics.input_age_ms),
    numericKeyValue(
      "decision_deadline_ms", this->registration_deadline_ms_),
    keyValue(
      "deadline_exceeded",
      metrics.deadline_exceeded ? "true" : "false"),
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
    numericKeyValue(
      "scan_points_capped", metrics.capped_scan_points),
    numericKeyValue("map_points_raw", this->raw_map_points_),
    numericKeyValue("target_points", metrics.target_points),
    numericKeyValue("generation", metrics.generation),
    numericKeyValue("iterations", metrics.iterations),
    numericKeyValue("fitness_score", metrics.fitness_score),
  };
  message.status.push_back(std::move(status));
  this->diagnostic_pub_->publish(message);
}

void LocalizationNode::publishInitializationDiagnostic(
  const builtin_interfaces::msg::Time & stamp,
  const InitializationMetrics & metrics)
{
  if (!this->publish_scan_diagnostics_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = metrics.success ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    metrics.ambiguous ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.name = "ndt_localization/initialization_search";
  status.hardware_id = "localization_node";
  status.message = ndt_localization::toString(metrics.decision);
  status.values = {
    keyValue("decision", status.message),
    keyValue("success", metrics.success ? "true" : "false"),
    keyValue("ambiguous", metrics.ambiguous ? "true" : "false"),
    keyValue("recovery", metrics.recovery ? "true" : "false"),
    keyValue(
      "state",
      ndt_localization::toString(this->state_machine_->state())),
    numericKeyValue("generation", metrics.generation),
    numericKeyValue("hypotheses", metrics.hypotheses),
    numericKeyValue("evaluated", metrics.evaluated),
    numericKeyValue("converged", metrics.converged),
    numericKeyValue("refined", metrics.refined),
    numericKeyValue("scan_points", metrics.scan_points),
    numericKeyValue("target_points", metrics.target_points),
    numericKeyValue("best_score", metrics.best_score),
    numericKeyValue("second_score", metrics.second_score),
    numericKeyValue("score_margin", metrics.score_margin),
    numericKeyValue("coarse_ms", metrics.coarse_ms),
    numericKeyValue("refinement_ms", metrics.refinement_ms),
    numericKeyValue("total_ms", metrics.total_ms),
    numericKeyValue(
      "translation_span_m", metrics.translation_span_m),
    numericKeyValue("yaw_span_deg", metrics.yaw_span_deg),
    numericKeyValue(
      "initialization_deadline_ms", this->initialization_timeout_ms_),
  };
  message.status.push_back(std::move(status));
  this->diagnostic_pub_->publish(message);
  RCLCPP_INFO(
    this->get_logger(),
    "Initialization search generation %s: %s, %zu/%zu converged, "
    "%zu refined, score %.6f, margin %.6f, %.3f ms",
    std::to_string(metrics.generation).c_str(),
    ndt_localization::toString(metrics.decision),
    metrics.converged, metrics.evaluated, metrics.refined,
    metrics.best_score, metrics.score_margin, metrics.total_ms);
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
    code ==
    ndt_localization::DecisionCode::INITIALIZATION_HYPOTHESIS_SELECTED ||
    code == ndt_localization::DecisionCode::INITIALIZATION_CONFIRMED ||
    code == ndt_localization::DecisionCode::RELOCALIZATION_STARTED ||
    code ==
    ndt_localization::DecisionCode::RELOCALIZATION_HYPOTHESIS_SELECTED ||
    code == ndt_localization::DecisionCode::RELOCALIZATION_CONFIRMED ||
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
  if (!this->map_initialized_.load(std::memory_order_acquire)) {
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
    this->initialization_stddev_multiplier_,
    this->initialization_minimum_translation_span_m_,
    this->initialization_maximum_translation_span_m_,
    this->initialization_minimum_yaw_span_deg_,
    this->initialization_maximum_yaw_span_deg_);

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
      this->pending_initialization_search_bounds_ = search_bounds;
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
  std::vector<std::shared_ptr<ScanTask>> invalidated;
  {
    std::lock_guard<std::mutex> lock(this->registration_mutex_);
    invalidated = this->invalidateRegistrationWorkLocked();
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
  this->publishSupersededTasks(invalidated);
  this->registration_cv_.notify_all();
  this->publishStateDiagnostic(
    stamp, ndt_localization::DecisionCode::INITIALIZATION_STARTED);
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
  std::vector<std::shared_ptr<ScanTask>> invalidated;
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
    invalidated = this->invalidateRegistrationWorkLocked();
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
  this->publishSupersededTasks(invalidated);
  this->registration_cv_.notify_all();
  this->publishStateDiagnostic(
    stamp, ndt_localization::DecisionCode::RELOCALIZATION_STARTED);
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
  Eigen::Isometry3d pending_pose = Eigen::Isometry3d::Identity();
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
