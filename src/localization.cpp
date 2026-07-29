#include "localization.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace
{
diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template<typename T>
diagnostic_msgs::msg::KeyValue numericKeyValue(const std::string& key, T value)
{
  return keyValue(key, std::to_string(value));
}
}  // namespace

LocalizationNode::LocalizationNode() : Node("localization_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing FAST-LIO Localization Node ...");

  // Parameters
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
  this->declare_parameter<bool>("localization.ndt_compute_fitness_score", false);
  this->declare_parameter<bool>("localization.publish_scan_diagnostics", true);

  this->get_parameter("odom_frame_id", this->odom_frame_id_);
  this->get_parameter("base_frame_id", this->base_frame_id_);
  this->get_parameter("map_frame_id", this->global_frame_id_);
  this->get_parameter("localization.ndt_resolution", this->ndt_resolution_);
  this->get_parameter("localization.ndt_step_size", this->ndt_step_size_);
  this->get_parameter("localization.ndt_trans_epsilon", this->ndt_trans_epsilon_);
  this->get_parameter("localization.ndt_max_iter", this->ndt_max_iter_);
  this->get_parameter("localization.ndt_map_leaf_size", this->ndt_map_leaf_size_);
  this->get_parameter("localization.ndt_scan_leaf_size", this->ndt_scan_leaf_size_);
  this->get_parameter("localization.ndt_log_runtime", this->ndt_log_runtime_);
  this->get_parameter(
      "localization.ndt_compute_fitness_score", this->ndt_compute_fitness_score_);
  this->get_parameter(
      "localization.publish_scan_diagnostics", this->publish_scan_diagnostics_);

  // TF
  this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize State
  this->map_to_odom_ = Eigen::Matrix4f::Identity();
  this->odom_to_base_ = Eigen::Matrix4f::Identity();
  this->global_map_.reset(new pcl::PointCloud<PointType>());

  // Map Subscription (Latched)
  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  this->map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/global_map", qos_profile, std::bind(&LocalizationNode::mapCallback, this, std::placeholders::_1));

  // Odom: High frequency
  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry_lio", 10, std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));

  // Scan: Use undistorted body frame cloud from FAST-LIO
  // Note: /cloud_registered_body is usually cleaner for matching
  this->scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_body", 10, std::bind(&LocalizationNode::scanCallback, this, std::placeholders::_1));

  // Initial Pose
  this->initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // Publisher
  this->pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_map", 10);
  this->scan_diagnostic_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization/scan_diagnostics", 10);
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (this->map_initialized_) return;

  RCLCPP_INFO(this->get_logger(), "Received Global Map from topic. Points: %d", msg->width * msg->height);

  pcl::fromROSMsg(*msg, *this->global_map_);
  const std::size_t raw_points = this->global_map_->size();
  this->raw_map_points_ = raw_points;
  this->global_map_ = this->downsampleCloud(this->global_map_, this->ndt_map_leaf_size_);

  RCLCPP_INFO(this->get_logger(), "Map received with %zu points. NDT target has %zu points. Building NDT...",
              raw_points, this->global_map_->size());

  // NDT Setup
  this->ndt_.setResolution(this->ndt_resolution_);
  this->ndt_.setStepSize(this->ndt_step_size_);
  this->ndt_.setTransformationEpsilon(this->ndt_trans_epsilon_);
  this->ndt_.setMaximumIterations(this->ndt_max_iter_);

  this->ndt_.setInputTarget(this->global_map_);
  this->map_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "NDT Target Map Set.");
}

pcl::PointCloud<PointType>::Ptr LocalizationNode::downsampleCloud(
    const pcl::PointCloud<PointType>::Ptr& cloud, double leaf_size)
{
  if (leaf_size <= 0.0 || cloud->empty()) {
    return cloud;
  }

  pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*filtered);
  return filtered;
}

void LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->latest_odom_ = *msg;
  this->has_odom_ = true;

  // Update odom_to_base
  Eigen::Isometry3d odom_to_base_d;
  tf2::fromMsg(msg->pose.pose, odom_to_base_d);
  this->odom_to_base_ = odom_to_base_d.cast<float>().matrix();

  // Publish TF immediately using the latest known map_to_odom correction
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = this->global_frame_id_;
  tf_msg.child_frame_id = this->odom_frame_id_; // map -> camera_init

  Eigen::Matrix4f map_to_odom_curr = this->map_to_odom_;
  Eigen::Isometry3d map_to_odom_d(map_to_odom_curr.cast<double>());
  tf_msg.transform = tf2::eigenToTransform(map_to_odom_d).transform;

  this->tf_broadcaster_->sendTransform(tf_msg);

  // Publish Odometry in Map Frame
  nav_msgs::msg::Odometry odom_map = *msg;
  odom_map.header.frame_id = this->global_frame_id_;
  odom_map.child_frame_id = this->base_frame_id_;

  // Transform Pose (T_map_base = T_map_odom * T_odom_base)
  Eigen::Matrix4f map_pose_curr = map_to_odom_curr * this->odom_to_base_;
  Eigen::Isometry3d map_pose_d(map_pose_curr.cast<double>());

  // Fill Pose
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(map_pose_d);
  odom_map.pose.pose = pose_msg;

  // Rotate Covariance (P_map = R * P_odom * R^T)
  Eigen::Matrix3d R = map_to_odom_d.rotation();

  // Copy covariance to Eigen matrix for easy manipulation
  Eigen::Matrix<double, 6, 6> P_odom = Eigen::Matrix<double, 6, 6>::Zero();
  for(int i=0; i<36; i++) P_odom(i/6, i%6) = msg->pose.covariance[i];

  Eigen::Matrix<double, 6, 6> P_map = Eigen::Matrix<double, 6, 6>::Zero();

  // Rotate Position Covariance
  P_map.block<3,3>(0,0) = R * P_odom.block<3,3>(0,0) * R.transpose();
  // Rotate Orientation Covariance
  P_map.block<3,3>(3,3) = R * P_odom.block<3,3>(3,3) * R.transpose();
  // Rotate Cross-Covariance
  P_map.block<3,3>(0,3) = R * P_odom.block<3,3>(0,3) * R.transpose();
  P_map.block<3,3>(3,0) = R * P_odom.block<3,3>(3,0) * R.transpose();

  for(int i=0; i<36; i++) odom_map.pose.covariance[i] = P_map(i/6, i%6);

  this->pub_odom_->publish(odom_map);
}

void LocalizationNode::scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();
  const auto scan_stamp = rclcpp::Time(msg->header.stamp);
  const double input_age_ms = (this->now() - scan_stamp).seconds() * 1000.0;

  if (!this->map_initialized_) {
    const double total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - callback_start).count();
    this->publishScanDiagnostic(
        msg->header.stamp, "map_unavailable", false, 0.0, 0.0, 0.0, total_ms,
        input_age_ms, msg->width * msg->height, 0, 0, false,
        std::numeric_limits<double>::quiet_NaN());
    return;
  }

  if (!this->has_odom_) {
    const double total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - callback_start).count();
    this->publishScanDiagnostic(
        msg->header.stamp, "odometry_unavailable", false, 0.0, 0.0, 0.0, total_ms,
        input_age_ms, msg->width * msg->height, 0, 0, false,
        std::numeric_limits<double>::quiet_NaN());
    return;
  }

  // If we haven't received an initial pose yet, we can't localize
  // (unless we assume start at 0,0,0, but usually we wait)
  if (!this->initial_pose_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial pose...");
      const double total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - callback_start).count();
      this->publishScanDiagnostic(
          msg->header.stamp, "initial_pose_unavailable", false, 0.0, 0.0, 0.0, total_ms,
          input_age_ms, msg->width * msg->height, 0, 0, false,
          std::numeric_limits<double>::quiet_NaN());
      return;
  }

  const auto conversion_start = std::chrono::steady_clock::now();
  pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *scan);
  const std::size_t raw_scan_points = scan->size();
  scan = this->downsampleCloud(scan, this->ndt_scan_leaf_size_);
  const auto conversion_end = std::chrono::steady_clock::now();
  const double conversion_ms =
      std::chrono::duration<double, std::milli>(conversion_end - conversion_start).count();

  // 1. Predict current pose in Map Frame
  // T_map_base_guess = T_map_odom * T_odom_base
  Eigen::Matrix4f odom_to_base_curr;
  {
      std::lock_guard<std::mutex> lock(this->mutex_);
      odom_to_base_curr = this->odom_to_base_;
  }
  Eigen::Matrix4f guess_pose = this->map_to_odom_ * odom_to_base_curr;

  // 2. Align
  const auto align_start = std::chrono::steady_clock::now();
  this->ndt_.setInputSource(scan);
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>());
  this->ndt_.align(*output_cloud, guess_pose);
  const auto align_end = std::chrono::steady_clock::now();
  const double align_ms =
      std::chrono::duration<double, std::milli>(align_end - align_start).count();

  // 3. Update Correction
  const auto validation_start = std::chrono::steady_clock::now();
  const bool converged = this->ndt_.hasConverged();
  const double fitness_score =
      converged && this->ndt_compute_fitness_score_ ?
      this->ndt_.getFitnessScore() :
      std::numeric_limits<double>::quiet_NaN();
  if (converged) {
      Eigen::Matrix4f T_map_base_opt = this->ndt_.getFinalTransformation();

      // Recalculate T_map_odom = T_map_base_opt * T_odom_base^-1
      this->map_to_odom_ = T_map_base_opt * odom_to_base_curr.inverse();

      RCLCPP_DEBUG(this->get_logger(), "NDT Converged. Score: %.4f", fitness_score);
  } else {
      RCLCPP_WARN(this->get_logger(), "NDT Diverged!");
  }
  const auto validation_end = std::chrono::steady_clock::now();
  const double validation_ms =
      std::chrono::duration<double, std::milli>(validation_end - validation_start).count();
  const double total_ms =
      std::chrono::duration<double, std::milli>(validation_end - callback_start).count();

  this->publishScanDiagnostic(
      msg->header.stamp, converged ? "accepted_baseline" : "matcher_not_converged",
      converged, conversion_ms, align_ms, validation_ms, total_ms, input_age_ms,
      raw_scan_points, scan->size(), this->ndt_.getFinalNumIteration(), converged,
      fitness_score);

  if (this->ndt_log_runtime_) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "NDT align: %.3f ms, scan points: %zu -> %zu, iterations: %d, converged: %s",
          align_ms, raw_scan_points, scan->size(), this->ndt_.getFinalNumIteration(),
          converged ? "true" : "false");
  }
}

void LocalizationNode::publishScanDiagnostic(
    const builtin_interfaces::msg::Time& stamp,
    const std::string& decision,
    bool accepted,
    double conversion_ms,
    double matcher_ms,
    double validation_ms,
    double total_ms,
    double input_age_ms,
    std::size_t raw_scan_points,
    std::size_t filtered_scan_points,
    int iterations,
    bool converged,
    double fitness_score)
{
  if (!this->publish_scan_diagnostics_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = accepted ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.name = "ndt_localization/scan";
  status.hardware_id = "localization_node";
  status.message = decision;
  status.values = {
    keyValue("decision", decision),
    keyValue("accepted", accepted ? "true" : "false"),
    numericKeyValue("conversion_ms", conversion_ms),
    numericKeyValue("local_map_ms", 0.0),
    numericKeyValue("matcher_ms", matcher_ms),
    numericKeyValue("validation_ms", validation_ms),
    numericKeyValue("total_ms", total_ms),
    numericKeyValue("input_age_ms", input_age_ms),
    numericKeyValue("scan_points_raw", raw_scan_points),
    numericKeyValue("scan_points_filtered", filtered_scan_points),
    numericKeyValue("map_points_raw", this->raw_map_points_),
    numericKeyValue("target_points", this->global_map_->size()),
    numericKeyValue("iterations", iterations),
    keyValue("converged", converged ? "true" : "false"),
    numericKeyValue("fitness_score", fitness_score),
  };
  message.status.push_back(std::move(status));
  this->scan_diagnostic_pub_->publish(message);
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Initial Pose.");

  Eigen::Isometry3d initial_pose_d;
  tf2::fromMsg(msg->pose.pose, initial_pose_d);
  Eigen::Matrix4f initial_pose = initial_pose_d.cast<float>().matrix(); // T_map_base

  // Calculate T_map_odom = T_map_base * T_odom_base^-1
  Eigen::Matrix4f odom_to_base_curr;
  {
      std::lock_guard<std::mutex> lock(this->mutex_);
      odom_to_base_curr = this->odom_to_base_;
  }

  this->map_to_odom_ = initial_pose * odom_to_base_curr.inverse();
  this->initial_pose_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Localization Reset.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
