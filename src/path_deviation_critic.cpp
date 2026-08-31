/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/path_deviation_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "excess_distance_scale",
    rclcpp::ParameterValue(1000.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "heading_recovery_activation_distance",
    rclcpp::ParameterValue(1.05));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "heading_recovery_lookahead_distance",
    rclcpp::ParameterValue(0.9));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "heading_recovery_scale",
    rclcpp::ParameterValue(200.0));
  node->get_parameter(
    prefix + "maximum_path_distance", maximum_path_distance_);
  node->get_parameter(prefix + "deviation_penalty", deviation_penalty_);
  node->get_parameter(
    prefix + "excess_distance_scale", excess_distance_scale_);
  node->get_parameter(
    prefix + "heading_recovery_activation_distance",
    heading_recovery_activation_distance_);
  node->get_parameter(
    prefix + "heading_recovery_lookahead_distance",
    heading_recovery_lookahead_distance_);
  node->get_parameter(
    prefix + "heading_recovery_scale", heading_recovery_scale_);

  validateParameters();
}

void PathDeviationCritic::validateParameters() const
{
  if (!std::isfinite(maximum_path_distance_) ||
    maximum_path_distance_ < 0.0 ||
    !std::isfinite(deviation_penalty_) || deviation_penalty_ < 0.0 ||
    !std::isfinite(excess_distance_scale_) || excess_distance_scale_ < 0.0 ||
    !std::isfinite(heading_recovery_activation_distance_) ||
    heading_recovery_activation_distance_ < 0.0 ||
    heading_recovery_activation_distance_ > maximum_path_distance_ ||
    !std::isfinite(heading_recovery_lookahead_distance_) ||
    heading_recovery_lookahead_distance_ < 0.0 ||
    !std::isfinite(heading_recovery_scale_) || heading_recovery_scale_ < 0.0)
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

double PathDeviationCritic::distanceToPath(
  const geometry_msgs::msg::Pose2D & pose,
  std::size_t & segment_hint) const
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    path_segments_.empty())
  {
    return std::numeric_limits<double>::infinity();
  }

  double minimum_squared_distance = std::numeric_limits<double>::infinity();
  std::size_t nearest_segment = 0u;
  const auto consider_segment =
    [this, &pose, &minimum_squared_distance, &nearest_segment](
    const std::size_t index) {
      const auto & segment = path_segments_[index];
      const double box_dx = std::max(
        {segment.minimum_x - pose.x, 0.0, pose.x - segment.maximum_x});
      const double box_dy = std::max(
        {segment.minimum_y - pose.y, 0.0, pose.y - segment.maximum_y});
      if (box_dx * box_dx + box_dy * box_dy >= minimum_squared_distance) {
        return;
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
      const double squared_distance =
        offset_x * offset_x + offset_y * offset_y;
      if (squared_distance < minimum_squared_distance) {
        minimum_squared_distance = squared_distance;
        nearest_segment = index;
      }
    };

  segment_hint = std::min(segment_hint, path_segments_.size() - 1u);
  consider_segment(segment_hint);
  for (std::size_t index = segment_hint + 1u;
    index < path_segments_.size(); ++index)
  {
    consider_segment(index);
  }
  for (std::size_t index = 0u; index < segment_hint; ++index) {
    consider_segment(index);
  }
  segment_hint = nearest_segment;
  return std::sqrt(minimum_squared_distance);
}

double PathDeviationCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!reference_path_valid_ || trajectory.poses.empty()) {
    return 0.0;
  }

  std::size_t segment_hint = 0u;
  bool outside_corridor = false;
  double excess_distance_sum = 0.0;
  double terminal_distance = 0.0;
  for (const auto & pose : trajectory.poses) {
    const double distance = distanceToPath(pose, segment_hint);
    if (!std::isfinite(distance)) {
      return deviation_penalty_;
    }
    const double excess = std::max(0.0, distance - maximum_path_distance_);
    outside_corridor = outside_corridor || excess > 0.0;
    excess_distance_sum += excess;
    terminal_distance = distance;
  }

  double heading_recovery_cost = 0.0;
  const double activation_width =
    maximum_path_distance_ - heading_recovery_activation_distance_;
  if (activation_width > 1.0e-12 &&
    heading_recovery_lookahead_distance_ > 0.0 &&
    heading_recovery_scale_ > 0.0)
  {
    const auto & terminal_pose = trajectory.poses.back();
    geometry_msgs::msg::Pose2D heading_probe = terminal_pose;
    heading_probe.x += heading_recovery_lookahead_distance_ *
      std::cos(terminal_pose.theta);
    heading_probe.y += heading_recovery_lookahead_distance_ *
      std::sin(terminal_pose.theta);
    const double probe_distance = distanceToPath(heading_probe, segment_hint);
    if (!std::isfinite(probe_distance)) {
      return deviation_penalty_;
    }
    // Activate from the projected heading probe, not only after the rollout
    // endpoint has already departed. Acceleration-, jerk-, and FIR-limited
    // methods need this finite gradient while their endpoint is still inside
    // the corridor so they can start a smooth turn before crossing it.
    const double activation = std::clamp(
      (probe_distance - heading_recovery_activation_distance_) /
      activation_width, 0.0, 1.0);
    const double heading_departure = std::max(
      0.0, probe_distance - terminal_distance);
    heading_recovery_cost =
      heading_recovery_scale_ * activation * heading_departure;
  }

  double boundary_cost = 0.0;
  if (outside_corridor) {
    const double mean_excess_distance =
      excess_distance_sum / static_cast<double>(trajectory.poses.size());
    boundary_cost = deviation_penalty_ +
      excess_distance_scale_ * mean_excess_distance;
  }
  return boundary_cost + heading_recovery_cost;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::PathDeviationCritic,
  dwb_core::TrajectoryCritic)
