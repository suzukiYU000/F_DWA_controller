/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/trajectory_progress_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void TrajectoryProgressCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    rclcpp::ParameterValue(2.88));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    lookahead_distance_);
  lookahead_distance_ = std::max(0.0, lookahead_distance_);
}

bool TrajectoryProgressCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  path_ = global_plan;
  cumulative_distance_.assign(path_.poses.size(), 0.0);
  if (path_.poses.size() < 2u) {
    current_progress_ = 0.0;
    desired_progress_ = 0.0;
    // A pruned plan can contain only the goal pose near completion. Treat the
    // progress term as neutral instead of failing the complete DWB cycle.
    return true;
  }

  for (std::size_t index = 1u; index < path_.poses.size(); ++index) {
    cumulative_distance_[index] = cumulative_distance_[index - 1u] +
      std::hypot(
      path_.poses[index].x - path_.poses[index - 1u].x,
      path_.poses[index].y - path_.poses[index - 1u].y);
  }
  current_progress_ = projectOntoPath(pose);
  desired_progress_ = std::min(
    lookahead_distance_,
    std::max(0.0, cumulative_distance_.back() - current_progress_));
  return true;
}

double TrajectoryProgressCritic::projectOntoPath(
  const geometry_msgs::msg::Pose2D & pose) const
{
  if (path_.poses.size() < 2u ||
    cumulative_distance_.size() != path_.poses.size())
  {
    return 0.0;
  }

  double nearest_squared_distance = std::numeric_limits<double>::infinity();
  double nearest_progress = 0.0;
  for (std::size_t index = 1u; index < path_.poses.size(); ++index) {
    const auto & start = path_.poses[index - 1u];
    const auto & end = path_.poses[index];
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_squared_length =
      segment_x * segment_x + segment_y * segment_y;
    if (segment_squared_length <= 1.0e-12) {
      continue;
    }
    const double projection = std::clamp(
      ((pose.x - start.x) * segment_x +
      (pose.y - start.y) * segment_y) / segment_squared_length,
      0.0, 1.0);
    const double projected_x = start.x + projection * segment_x;
    const double projected_y = start.y + projection * segment_y;
    const double squared_distance =
      (pose.x - projected_x) * (pose.x - projected_x) +
      (pose.y - projected_y) * (pose.y - projected_y);
    if (squared_distance < nearest_squared_distance) {
      nearest_squared_distance = squared_distance;
      nearest_progress = cumulative_distance_[index - 1u] +
        projection * std::sqrt(segment_squared_length);
    }
  }
  return nearest_progress;
}

double TrajectoryProgressCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  double maximum_progress = current_progress_;
  for (const auto & pose : trajectory.poses) {
    maximum_progress = std::max(maximum_progress, projectOntoPath(pose));
  }
  const double candidate_progress = std::clamp(
    maximum_progress - current_progress_, 0.0, desired_progress_);
  return desired_progress_ - candidate_progress;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::TrajectoryProgressCritic,
  dwb_core::TrajectoryCritic)
