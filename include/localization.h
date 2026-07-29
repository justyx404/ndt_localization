#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_core.h"
#include "nav_msgs/msg/odometry.hpp"
#include "point_cloud_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "robust_initializer.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <pcl/registration/ndt.h>

#include <tf2_ros/transform_broadcaster.h>

using PointType = ndt_localization::Point;

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode();
  ~LocalizationNode();

private:
  enum class RegistrationInputStatus
  {
    READY,
    INVALID_SCAN,
    REGISTRATION_REJECTED,
  };

  struct ScanTask
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::uint64_t generation = 0;
    std::chrono::steady_clock::time_point received_at;
    std::chrono::steady_clock::time_point deadline;
    bool waiting_for_odometry = false;
    std::uint64_t required_odometry_sequence = 0;
    bool decided = false;
  };

  struct RegistrationInput
  {
    pcl::PointCloud<PointType>::ConstPtr scan;
    pcl::PointCloud<PointType>::ConstPtr target;
    Eigen::Isometry3d pose_guess = Eigen::Isometry3d::Identity();
  };

  struct InitializationTask
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::uint64_t generation = 0;
    std::chrono::steady_clock::time_point search_deadline;
    ndt_localization::InitializationSearchBounds bounds;
    bool recovery = false;
  };

  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  bool validTimestamp(
    const builtin_interfaces::msg::Time & stamp,
    double maximum_age_seconds) const;
  void publishMapPrediction(
    const nav_msgs::msg::Odometry & odometry,
    const Eigen::Isometry3d & odom_to_base,
    const Eigen::Isometry3d & map_to_odom);
  void rejectScan();
  void enqueueScan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const std::chrono::steady_clock::time_point & received_at);
  void registrationWorkerLoop();
  void initializationWorkerLoop();
  void deadlineWorkerLoop();
  void processScanTask(const std::shared_ptr<ScanTask> & task);
  RegistrationInputStatus prepareRegistrationInput(
    const std::shared_ptr<ScanTask> & task,
    const Eigen::Isometry3d & correction_guess,
    const ndt_localization::OdometryLookup & synchronized_odometry,
    RegistrationInput * input);
  bool runRegistration(
    const RegistrationInput & input,
    Eigen::Isometry3d * optimized_pose);
  void commitRegistrationResult(
    const std::shared_ptr<ScanTask> & task,
    const Eigen::Isometry3d & candidate_correction);
  void processInitializationTask(
    const std::shared_ptr<InitializationTask> & task);
  void enqueueInitializationScan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void finalizeRejectedTask(
    const std::shared_ptr<ScanTask> & task,
    bool registration_rejection);
  void invalidateRegistrationWorkLocked();
  bool applyRejectionStateLocked(bool registration_rejection);
  void beginInitialization(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Isometry3d & initial_pose,
    const Eigen::Isometry3d & synchronized_odometry,
    const ndt_localization::InitializationSearchBounds & bounds);
  bool startRelocalization(
    const builtin_interfaces::msg::Time & stamp,
    bool ignore_retry_delay = false);
  void triggerRelocalizationCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void tryStartPendingInitialization();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    relocalization_service_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  pcl::PointCloud<PointType>::ConstPtr global_map_;
  pcl::KdTreeFLANN<PointType>::Ptr map_kdtree_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
  std::unique_ptr<ndt_localization::RobustInitializer>
    robust_initializer_;

  std::atomic<bool> map_initialized_{false};
  std::size_t raw_map_points_ = 0;
  std::mutex odometry_mutex_;
  std::mutex pending_mutex_;
  std::mutex registration_mutex_;
  std::condition_variable registration_cv_;
  std::thread registration_worker_;
  std::thread initialization_worker_;
  std::thread deadline_worker_;
  bool stop_registration_threads_ = false;
  std::uint64_t latest_scan_generation_ = 0;
  std::uint64_t odometry_sequence_ = 0;
  std::shared_ptr<ScanTask> pending_scan_task_;
  std::shared_ptr<ScanTask> active_scan_task_;
  std::uint64_t initialization_generation_ = 0;
  bool initialization_attempt_active_ = false;
  bool initialization_search_required_ = false;
  bool initialization_recovery_ = false;
  std::chrono::steady_clock::time_point initialization_attempt_deadline_;
  std::chrono::steady_clock::time_point next_relocalization_attempt_;
  ndt_localization::InitializationSearchBounds initialization_search_bounds_;
  std::shared_ptr<InitializationTask> pending_initialization_task_;
  std::shared_ptr<InitializationTask> active_initialization_task_;
  std::unique_ptr<ndt_localization::OdometryBuffer> odometry_buffer_;
  std::unique_ptr<ndt_localization::LocalizationStateMachine> state_machine_;
  bool initial_pose_waiting_for_odometry_ = false;
  builtin_interfaces::msg::Time pending_initial_pose_stamp_;
  Eigen::Isometry3d pending_initial_pose_ = Eigen::Isometry3d::Identity();
  ndt_localization::InitializationSearchBounds
    pending_initialization_search_bounds_;
  std::atomic<std::int64_t> initialization_stamp_ns_{0};

  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double ndt_resolution_;
  double ndt_map_leaf_size_;
  double local_map_radius_m_;
  int maximum_local_map_points_;
  int maximum_scan_points_;
  double registration_deadline_ms_;
  double initialization_timeout_ms_;
  double initialization_maximum_translation_span_m_;
  double initialization_maximum_yaw_span_deg_;
  double recovery_translation_span_m_;
  double recovery_yaw_span_deg_;
  double initialization_maximum_fitness_score_;
  double initialization_minimum_score_margin_;
  int initialization_confirmation_scans_;
  double maximum_result_translation_delta_m_;
  double maximum_result_rotation_delta_deg_;
  int maximum_consecutive_rejections_;
};
