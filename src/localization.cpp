#include "localization.h"

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

  this->get_parameter("odom_frame_id", this->odom_frame_id_);
  this->get_parameter("base_frame_id", this->base_frame_id_);
  this->get_parameter("map_frame_id", this->global_frame_id_);
  this->get_parameter("localization.ndt_resolution", this->ndt_resolution_);
  this->get_parameter("localization.ndt_step_size", this->ndt_step_size_);
  this->get_parameter("localization.ndt_trans_epsilon", this->ndt_trans_epsilon_);
  this->get_parameter("localization.ndt_max_iter", this->ndt_max_iter_);

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
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (this->map_initialized_) return;

  RCLCPP_INFO(this->get_logger(), "Received Global Map from topic. Points: %d", msg->width * msg->height);

  pcl::fromROSMsg(*msg, *this->global_map_);

  RCLCPP_INFO(this->get_logger(), "Map received with %zu points. Building NDT...", this->global_map_->size());

  // NDT Setup
  this->ndt_.setResolution(this->ndt_resolution_);
  this->ndt_.setStepSize(this->ndt_step_size_);
  this->ndt_.setTransformationEpsilon(this->ndt_trans_epsilon_);
  this->ndt_.setMaximumIterations(this->ndt_max_iter_);

  this->ndt_.setInputTarget(this->global_map_);
  this->map_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "NDT Target Map Set.");
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
  if (!this->map_initialized_ || !this->has_odom_) return;

  // If we haven't received an initial pose yet, we can't localize
  // (unless we assume start at 0,0,0, but usually we wait)
  if (!this->initial_pose_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial pose...");
      return; 
  }

  pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *scan);

  // 1. Predict current pose in Map Frame
  // T_map_base_guess = T_map_odom * T_odom_base
  Eigen::Matrix4f odom_to_base_curr;
  {
      std::lock_guard<std::mutex> lock(this->mutex_);
      odom_to_base_curr = this->odom_to_base_;
  }
  Eigen::Matrix4f guess_pose = this->map_to_odom_ * odom_to_base_curr;

  // 2. Align
  this->ndt_.setInputSource(scan);
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>());
  this->ndt_.align(*output_cloud, guess_pose);

  // 3. Update Correction
  if (this->ndt_.hasConverged()) {
      Eigen::Matrix4f T_map_base_opt = this->ndt_.getFinalTransformation();

      // Recalculate T_map_odom = T_map_base_opt * T_odom_base^-1
      this->map_to_odom_ = T_map_base_opt * odom_to_base_curr.inverse();

      RCLCPP_DEBUG(this->get_logger(), "NDT Converged. Score: %.4f", this->ndt_.getFitnessScore());
  } else {
      RCLCPP_WARN(this->get_logger(), "NDT Diverged!");
  }
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
