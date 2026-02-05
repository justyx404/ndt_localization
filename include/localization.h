#pragma once

#include <mutex>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using PointType = pcl::PointXYZ;

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode();
  ~LocalizationNode();

private:
  // Callbacks
  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Data
  pcl::PointCloud<PointType>::Ptr global_map_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;

  // State
  Eigen::Matrix4f map_to_odom_;   // map -> odom
  Eigen::Matrix4f odom_to_base_;  // (current) odom -> base_link
  bool map_initialized_ = false;
  bool initial_pose_received_ = false;

  // Buffers
  std::mutex mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_ = false;

  // Parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_trans_epsilon_;
  int ndt_max_iter_;
};
