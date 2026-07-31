/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{

void HorizonObstacleFootprintCritic::onInit()
{
  dwb_critics::ObstacleFootprintCritic::onInit();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".score_time_horizon",
    rclcpp::ParameterValue(1.25));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".score_time_horizon",
    score_time_horizon_);
  score_time_horizon_ = std::max(0.0, score_time_horizon_);
}

double HorizonObstacleFootprintCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  const bool has_timing =
    trajectory.time_offsets.size() == trajectory.poses.size();
  double score = 0.0;
  std::size_t scored_pose_count = 0u;
  for (std::size_t index = 0u; index < trajectory.poses.size(); ++index) {
    const bool within_score_horizon =
      !has_timing ||
      rclcpp::Duration(trajectory.time_offsets[index]).seconds() <=
      score_time_horizon_;
    if (!within_score_horizon) {
      break;
    }
    score += scorePose(trajectory.poses[index]);
    ++scored_pose_count;
  }
  return scored_pose_count == 0u ?
         0.0 : score / static_cast<double>(scored_pose_count);
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::HorizonObstacleFootprintCritic,
  dwb_core::TrajectoryCritic)
