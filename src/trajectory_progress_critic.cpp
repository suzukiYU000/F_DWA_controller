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
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".lateral_distance_weight",
    rclcpp::ParameterValue(1.0));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    lookahead_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".lateral_distance_weight",
    lateral_distance_weight_);
  lookahead_distance_ = std::max(0.0, lookahead_distance_);
  if (!std::isfinite(lateral_distance_weight_) ||
    lateral_distance_weight_ < 0.0)
  {
    throw std::invalid_argument(
            dwb_plugin_name_ + "." + name_ +
            ".lateral_distance_weight must be finite and non-negative");
  }
}

bool TrajectoryProgressCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  path_ = global_plan;
  cumulative_distance_.assign(path_.poses.size(), 0.0);
  path_segments_.clear();
  current_segment_hint_ = 0u;
  if (path_.poses.size() < 2u) {
    current_progress_ = 0.0;
    current_cross_track_distance_ = 0.0;
    desired_progress_ = 0.0;
    // A pruned plan can contain only the goal pose near completion. Treat the
    // progress term as neutral instead of failing the complete DWB cycle.
    return true;
  }

  path_segments_.reserve(path_.poses.size() - 1u);
  for (std::size_t index = 1u; index < path_.poses.size(); ++index) {
    const auto & start = path_.poses[index - 1u];
    const auto & end = path_.poses[index];
    const double delta_x = end.x - start.x;
    const double delta_y = end.y - start.y;
    const double squared_length = delta_x * delta_x + delta_y * delta_y;
    const double length = std::sqrt(squared_length);
    cumulative_distance_[index] = cumulative_distance_[index - 1u] + length;
    if (squared_length <= 1.0e-12) {
      continue;
    }
    path_segments_.push_back(PathSegment{
        start.x, start.y, delta_x, delta_y, 1.0 / squared_length, length,
        cumulative_distance_[index - 1u],
        std::min(start.x, end.x), std::max(start.x, end.x),
        std::min(start.y, end.y), std::max(start.y, end.y)});
  }
  if (path_segments_.empty()) {
    current_progress_ = 0.0;
    current_cross_track_distance_ = 0.0;
    desired_progress_ = 0.0;
    return true;
  }
  const auto current_projection = projectOntoPath(
    pose, current_segment_hint_);
  current_progress_ = current_projection.progress;
  current_cross_track_distance_ = current_projection.distance;
  desired_progress_ = std::min(
    lookahead_distance_,
    std::max(0.0, cumulative_distance_.back() - current_progress_));
  return true;
}

TrajectoryProgressCritic::PathProjection
TrajectoryProgressCritic::projectOntoPath(
  const geometry_msgs::msg::Pose2D & pose,
  std::size_t & segment_hint) const
{
  if (path_segments_.empty()) {
    return {};
  }

  double nearest_squared_distance = std::numeric_limits<double>::infinity();
  double nearest_progress = 0.0;
  std::size_t nearest_segment = path_segments_.size();
  const auto consider_segment =
    [this, &pose, &nearest_squared_distance, &nearest_progress,
      &nearest_segment](const std::size_t index)
    {
      const auto & segment = path_segments_[index];
      const double box_dx = std::max(
        {segment.minimum_x - pose.x, 0.0, pose.x - segment.maximum_x});
      const double box_dy = std::max(
        {segment.minimum_y - pose.y, 0.0, pose.y - segment.maximum_y});
      const double box_squared_distance = box_dx * box_dx + box_dy * box_dy;
      if (box_squared_distance > nearest_squared_distance ||
        (box_squared_distance == nearest_squared_distance &&
        index >= nearest_segment))
      {
        return;
      }
      const double projection = std::clamp(
        ((pose.x - segment.start_x) * segment.delta_x +
        (pose.y - segment.start_y) * segment.delta_y) *
        segment.inverse_squared_length,
        0.0, 1.0);
      const double projected_x =
        segment.start_x + projection * segment.delta_x;
      const double projected_y =
        segment.start_y + projection * segment.delta_y;
      const double squared_distance =
        (pose.x - projected_x) * (pose.x - projected_x) +
        (pose.y - projected_y) * (pose.y - projected_y);
      if (squared_distance < nearest_squared_distance ||
        (squared_distance == nearest_squared_distance &&
        index < nearest_segment))
      {
        nearest_squared_distance = squared_distance;
        nearest_segment = index;
        nearest_progress = segment.cumulative_start +
          projection * segment.length;
      }
    };

  segment_hint = std::min(segment_hint, path_segments_.size() - 1u);
  consider_segment(segment_hint);
  for (std::size_t index = 0u; index < path_segments_.size(); ++index) {
    if (index != segment_hint) {
      consider_segment(index);
    }
  }
  segment_hint = nearest_segment;
  return PathProjection{
    nearest_progress, std::sqrt(nearest_squared_distance)};
}

double TrajectoryProgressCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  double maximum_effective_progress = 0.0;
  std::size_t segment_hint = current_segment_hint_;
  for (const auto & pose : trajectory.poses) {
    const auto projection = projectOntoPath(pose, segment_hint);
    const double path_advance = std::max(
      0.0, projection.progress - current_progress_);
    const double added_cross_track_distance = std::max(
      0.0, projection.distance - current_cross_track_distance_);
    maximum_effective_progress = std::max(
      maximum_effective_progress,
      path_advance -
      lateral_distance_weight_ * added_cross_track_distance);
    if (maximum_effective_progress >= desired_progress_) {
      return 0.0;
    }
  }
  const double candidate_progress = std::clamp(
    maximum_effective_progress, 0.0, desired_progress_);
  return desired_progress_ - candidate_progress;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::TrajectoryProgressCritic,
  dwb_core::TrajectoryCritic)
