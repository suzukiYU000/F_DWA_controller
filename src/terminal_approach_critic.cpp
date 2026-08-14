/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/terminal_approach_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kTimeTolerance = 1.0e-8;

}  // namespace

void TerminalApproachCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".evaluation_time",
    rclcpp::ParameterValue(1.4));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".outer_distance",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".full_weight_distance",
    rclcpp::ParameterValue(0.65));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".reference_deceleration",
    rclcpp::ParameterValue(0.15));

  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".evaluation_time",
    evaluation_time_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".outer_distance",
    outer_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".full_weight_distance",
    full_weight_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".reference_deceleration",
    reference_deceleration_);
  node->get_parameter(
    dwb_plugin_name_ + ".include_last_point", include_last_point_);

  validateParameters();
}

void TerminalApproachCritic::validateParameters() const
{
  if (!std::isfinite(evaluation_time_) || evaluation_time_ <= 0.0 ||
    !std::isfinite(outer_distance_) || outer_distance_ <= 0.0 ||
    !std::isfinite(full_weight_distance_) ||
    full_weight_distance_ < 0.0 ||
    full_weight_distance_ >= outer_distance_ ||
    !std::isfinite(reference_deceleration_) ||
    reference_deceleration_ <= 0.0)
  {
    throw std::invalid_argument(
            dwb_plugin_name_ + "." + name_ +
            " terminal-approach parameters are invalid");
  }
}

bool TerminalApproachCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  goal_ = goal;
  const double distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
  if (!std::isfinite(distance) || distance >= outer_distance_) {
    approach_weight_ = 0.0;
    return true;
  }
  if (distance <= full_weight_distance_) {
    approach_weight_ = 1.0;
    return true;
  }

  const double ratio = std::clamp(
    (outer_distance_ - distance) /
    (outer_distance_ - full_weight_distance_),
    0.0, 1.0);
  approach_weight_ = ratio * ratio * (3.0 - 2.0 * ratio);
  return true;
}

double TerminalApproachCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (approach_weight_ <= 0.0 || trajectory.poses.empty()) {
    return 0.0;
  }

  const auto evaluation_state = evaluationState(trajectory);
  if (!evaluation_state) {
    // This is a soft ranker, not a trajectory-validity gate. A synthetic or
    // malformed timing layout is therefore handled explicitly as neutral.
    return 0.0;
  }

  const double stopping_distance =
    evaluation_state->speed * evaluation_state->speed /
    (2.0 * reference_deceleration_);
  const double stopping_x = evaluation_state->pose.x +
    stopping_distance * std::cos(evaluation_state->pose.theta);
  const double stopping_y = evaluation_state->pose.y +
    stopping_distance * std::sin(evaluation_state->pose.theta);
  const double stopping_pose_distance =
    std::hypot(stopping_x - goal_.x, stopping_y - goal_.y);
  if (!std::isfinite(stopping_pose_distance)) {
    return 0.0;
  }
  return approach_weight_ * stopping_pose_distance;
}

std::optional<TerminalApproachCritic::EvaluationState>
TerminalApproachCritic::evaluationState(
  const dwb_msgs::msg::Trajectory2D & trajectory) const
{
  const std::size_t offset_count = trajectory.time_offsets.size();
  if (trajectory.poses.size() < 3u || offset_count < 2u ||
    trajectory.poses.size() != offset_count + 1u)
  {
    return std::nullopt;
  }

  // With Nav2's include_last convention, offsets[i] is the timestamp for
  // poses[i]. The final pose is an un-timestamped duplicate that only exists
  // to retain the historical Trajectory2D shape.
  if (include_last_point_) {
    const auto & timestamped_last_pose = trajectory.poses[offset_count - 1u];
    const auto & duplicate_last_pose = trajectory.poses[offset_count];
    constexpr double duplicate_tolerance = 1.0e-9;
    if (!std::isfinite(timestamped_last_pose.x) ||
      !std::isfinite(timestamped_last_pose.y) ||
      !std::isfinite(timestamped_last_pose.theta) ||
      !std::isfinite(duplicate_last_pose.x) ||
      !std::isfinite(duplicate_last_pose.y) ||
      !std::isfinite(duplicate_last_pose.theta) ||
      std::hypot(
        timestamped_last_pose.x - duplicate_last_pose.x,
        timestamped_last_pose.y - duplicate_last_pose.y) >
      duplicate_tolerance ||
      std::abs(std::remainder(
        timestamped_last_pose.theta - duplicate_last_pose.theta,
        kTwoPi)) > duplicate_tolerance)
    {
      return std::nullopt;
    }
  }

  std::size_t evaluation_index = offset_count;
  double previous_time = -1.0;
  for (std::size_t index = 0u; index < offset_count; ++index) {
    const double current_time =
      rclcpp::Duration(trajectory.time_offsets[index]).seconds();
    if (!std::isfinite(current_time) ||
      (index == 0u && std::abs(current_time) > kTimeTolerance) ||
      (index > 0u && current_time <= previous_time))
    {
      return std::nullopt;
    }
    if (evaluation_index == offset_count &&
      current_time >= evaluation_time_ - kTimeTolerance)
    {
      evaluation_index = index;
    }
    previous_time = current_time;
  }
  if (evaluation_index == 0u || evaluation_index == offset_count) {
    return std::nullopt;
  }

  const std::size_t previous_index = evaluation_index - 1u;
  const double before_time =
    rclcpp::Duration(trajectory.time_offsets[previous_index]).seconds();
  const double after_time =
    rclcpp::Duration(trajectory.time_offsets[evaluation_index]).seconds();
  const double time_step = after_time - before_time;
  if (!std::isfinite(time_step) || time_step <= 0.0 ||
    evaluation_time_ > after_time + kTimeTolerance)
  {
    return std::nullopt;
  }

  const auto & before_pose = trajectory.poses[previous_index];
  const auto & after_pose = trajectory.poses[evaluation_index];
  if (!std::isfinite(before_pose.x) || !std::isfinite(before_pose.y) ||
    !std::isfinite(before_pose.theta) || !std::isfinite(after_pose.x) ||
    !std::isfinite(after_pose.y) || !std::isfinite(after_pose.theta))
  {
    return std::nullopt;
  }

  const double interval_ratio = std::clamp(
    (evaluation_time_ - before_time) / time_step, 0.0, 1.0);
  EvaluationState state;
  state.pose.x = before_pose.x +
    interval_ratio * (after_pose.x - before_pose.x);
  state.pose.y = before_pose.y +
    interval_ratio * (after_pose.y - before_pose.y);
  state.pose.theta = before_pose.theta + interval_ratio * std::remainder(
    after_pose.theta - before_pose.theta, kTwoPi);
  state.speed = std::hypot(
    after_pose.x - before_pose.x,
    after_pose.y - before_pose.y) / time_step;
  if (!std::isfinite(state.speed)) {
    return std::nullopt;
  }
  return state;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::TerminalApproachCritic,
  dwb_core::TrajectoryCritic)
