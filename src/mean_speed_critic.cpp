/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/mean_speed_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{

void MeanSpeedCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".target_speed",
    rclcpp::ParameterValue(1.2));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".target_speed",
    target_speed_);
  target_speed_ = std::max(0.0, target_speed_);
}

bool MeanSpeedCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  current_pose_ = pose;
  return true;
}

double MeanSpeedCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.empty() || trajectory.time_offsets.empty()) {
    return target_speed_;
  }

  double traveled_distance = 0.0;
  geometry_msgs::msg::Pose2D previous_pose = current_pose_;
  for (const auto & pose : trajectory.poses) {
    traveled_distance += std::hypot(
      pose.x - previous_pose.x, pose.y - previous_pose.y);
    previous_pose = pose;
  }
  const double duration_seconds =
    rclcpp::Duration(trajectory.time_offsets.back()).seconds();
  if (duration_seconds <= 0.0) {
    return target_speed_;
  }
  const double mean_speed = traveled_distance / duration_seconds;
  return std::max(0.0, target_speed_ - mean_speed);
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::MeanSpeedCritic,
  dwb_core::TrajectoryCritic)
