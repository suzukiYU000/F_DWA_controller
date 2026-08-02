/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/path_subgoal_dist_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "f_dwa_controller/path_subgoal.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void PathSubgoalDistCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".allow_forward_overshoot",
    rclcpp::ParameterValue(false));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    lookahead_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".allow_forward_overshoot",
    allow_forward_overshoot_);
  lookahead_distance_ = std::max(0.0, lookahead_distance_);
}

bool PathSubgoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  subgoal_valid_ = compute_path_subgoal(
    global_plan, pose, lookahead_distance_, subgoal_);
  return true;
}

double PathSubgoalDistCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!subgoal_valid_ || trajectory.poses.empty()) {
    return 0.0;
  }
  const auto & terminal_pose = trajectory.poses.back();
  if (allow_forward_overshoot_) {
    return path_subgoal_forward_ray_cost(terminal_pose, subgoal_);
  }
  return std::hypot(
    terminal_pose.x - subgoal_.x, terminal_pose.y - subgoal_.y);
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::PathSubgoalDistCritic,
  dwb_core::TrajectoryCritic)
