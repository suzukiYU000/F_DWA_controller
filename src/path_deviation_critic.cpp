/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/path_deviation_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "f_dwa_controller/path_subgoal.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void PathDeviationCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "maximum_path_distance",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "deviation_penalty",
    rclcpp::ParameterValue(1000.0));
  node->get_parameter(
    prefix + "maximum_path_distance", maximum_path_distance_);
  node->get_parameter(prefix + "deviation_penalty", deviation_penalty_);

  validateParameters();
}

void PathDeviationCritic::validateParameters() const
{
  if (!std::isfinite(maximum_path_distance_) ||
    maximum_path_distance_ < 0.0 ||
    !std::isfinite(deviation_penalty_) || deviation_penalty_ < 0.0)
  {
    throw std::invalid_argument(
            dwb_plugin_name_ + "." + name_ +
            " path-deviation parameters are invalid");
  }
}

bool PathDeviationCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reference_path_ = global_plan;
  PathProjection projection;
  reference_path_valid_ = project_pose_onto_path(
    reference_path_, pose, projection);
  path_segments_.clear();
  if (!reference_path_valid_) {
    return false;
  }
  path_segments_.reserve(reference_path_.poses.size());
  for (std::size_t index = 1u; index < reference_path_.poses.size(); ++index) {
    const auto & start = reference_path_.poses[index - 1u];
    const auto & end = reference_path_.poses[index];
    const double delta_x = end.x - start.x;
    const double delta_y = end.y - start.y;
    const double squared_length =
      delta_x * delta_x + delta_y * delta_y;
    if (squared_length <= 1.0e-12) {
      continue;
    }
    path_segments_.push_back(PathSegment{
        start.x, start.y, delta_x, delta_y, 1.0 / squared_length,
        std::min(start.x, end.x), std::max(start.x, end.x),
        std::min(start.y, end.y), std::max(start.y, end.y)});
  }
  if (path_segments_.empty()) {
    const auto & point = reference_path_.poses.back();
    path_segments_.push_back(PathSegment{
        point.x, point.y, 0.0, 0.0, 0.0,
        point.x, point.x, point.y, point.y});
  }
  return reference_path_valid_;
}

bool PathDeviationCritic::poseIsInsideCorridor(
  const geometry_msgs::msg::Pose2D & pose,
  std::size_t & segment_hint) const
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    path_segments_.empty())
  {
    return false;
  }
  const double maximum_squared_distance =
    maximum_path_distance_ * maximum_path_distance_;
  const auto segment_is_within =
    [this, &pose, maximum_squared_distance](const PathSegment & segment) {
      if (pose.x < segment.minimum_x - maximum_path_distance_ ||
        pose.x > segment.maximum_x + maximum_path_distance_ ||
        pose.y < segment.minimum_y - maximum_path_distance_ ||
        pose.y > segment.maximum_y + maximum_path_distance_)
      {
        return false;
      }
      const double along = segment.inverse_squared_length > 0.0 ?
        std::clamp(
        ((pose.x - segment.start_x) * segment.delta_x +
        (pose.y - segment.start_y) * segment.delta_y) *
        segment.inverse_squared_length, 0.0, 1.0) : 0.0;
      const double offset_x =
        pose.x - (segment.start_x + along * segment.delta_x);
      const double offset_y =
        pose.y - (segment.start_y + along * segment.delta_y);
      return offset_x * offset_x + offset_y * offset_y <=
             maximum_squared_distance;
    };

  segment_hint = std::min(segment_hint, path_segments_.size() - 1u);
  for (std::size_t index = segment_hint;
    index < path_segments_.size(); ++index)
  {
    if (segment_is_within(path_segments_[index])) {
      segment_hint = index;
      return true;
    }
  }
  for (std::size_t index = 0u; index < segment_hint; ++index) {
    if (segment_is_within(path_segments_[index])) {
      segment_hint = index;
      return true;
    }
  }
  return false;
}

double PathDeviationCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!reference_path_valid_ || trajectory.poses.empty()) {
    return 0.0;
  }

  std::size_t segment_hint = 0u;
  for (const auto & pose : trajectory.poses) {
    if (!poseIsInsideCorridor(pose, segment_hint)) {
      return deviation_penalty_;
    }
  }
  return 0.0;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::PathDeviationCritic,
  dwb_core::TrajectoryCritic)
