/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Locus Robotics
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/hysteresis_rotate_to_goal_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{
namespace
{

double squared_norm(const double x, const double y)
{
  return x * x + y * y;
}

geometry_msgs::msg::Pose2D project_pose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  const std::size_t pose_count = std::min(
    trajectory.poses.size(), trajectory.time_offsets.size());
  if (pose_count == 0u) {
    throw dwb_core::IllegalTrajectoryException(
            "RotateToGoal", "Trajectory has no timed pose.");
  }

  const rclcpp::Duration goal_time =
    rclcpp::Duration::from_seconds(time_offset);
  if (goal_time <= rclcpp::Duration(trajectory.time_offsets.front())) {
    return trajectory.poses.front();
  }
  if (goal_time >= rclcpp::Duration(
      trajectory.time_offsets[pose_count - 1u]))
  {
    return trajectory.poses[pose_count - 1u];
  }

  for (std::size_t index = 0u; index + 1u < pose_count; ++index) {
    const rclcpp::Duration start_time(trajectory.time_offsets[index]);
    const rclcpp::Duration end_time(trajectory.time_offsets[index + 1u]);
    if (goal_time < start_time || goal_time >= end_time) {
      continue;
    }
    const double duration = (end_time - start_time).seconds();
    if (duration <= 0.0) {
      return trajectory.poses[index];
    }
    const double ratio = (goal_time - start_time).seconds() / duration;
    geometry_msgs::msg::Pose2D pose;
    pose.x = trajectory.poses[index].x +
      ratio * (trajectory.poses[index + 1u].x -
      trajectory.poses[index].x);
    pose.y = trajectory.poses[index].y +
      ratio * (trajectory.poses[index + 1u].y -
      trajectory.poses[index].y);
    pose.theta = trajectory.poses[index].theta +
      ratio * angles::shortest_angular_distance(
      trajectory.poses[index].theta,
      trajectory.poses[index + 1u].theta);
    return pose;
  }
  return trajectory.poses[pose_count - 1u];
}

}  // namespace

void HysteresisRotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const double xy_goal_tolerance = nav_2d_utils::searchAndGetParam(
    node, dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ =
    xy_goal_tolerance * xy_goal_tolerance;
  const double release_margin = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ +
    ".xy_goal_tolerance_release_margin",
    -1.0);
  release_window_latch_ = release_margin >= 0.0;
  const double release_tolerance =
    xy_goal_tolerance + std::max(0.0, release_margin);
  xy_goal_tolerance_release_sq_ =
    release_tolerance * release_tolerance;

  const double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(
    node, dwb_plugin_name_ + ".trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ =
    stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".slowing_factor",
    5.0);
  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time",
    -1.0);
  ignore_goal_orientation_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".ignore_goal_orientation",
    false);
  reset();
}

void HysteresisRotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
}

bool HysteresisRotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  const double goal_distance_sq =
    squared_norm(pose.x - goal.x, pose.y - goal.y);
  if (release_window_latch_ && in_window_ &&
    goal_distance_sq > xy_goal_tolerance_release_sq_)
  {
    in_window_ = false;
    rotating_ = false;
  }
  in_window_ =
    in_window_ || goal_distance_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = squared_norm(velocity.x, velocity.y);
  rotating_ = rotating_ ||
    (in_window_ &&
    current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  // Position-and-speed experiments still need the rotate critic's controlled
  // deceleration, but their terminal yaw must not affect candidate ranking.
  goal_yaw_ = ignore_goal_orientation_ ? pose.theta : goal.theta;
  return true;
}

double HysteresisRotateToGoalCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!in_window_) {
    return 0.0;
  }
  const double speed_sq = squared_norm(
    trajectory.velocity.x, trajectory.velocity.y);
  if (!rotating_) {
    if (speed_sq >= current_xy_speed_sq_) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor_ + score_rotation(trajectory);
  }
  if (std::fabs(trajectory.velocity.x) > 0.0 ||
    std::fabs(trajectory.velocity.y) > 0.0)
  {
    throw dwb_core::IllegalTrajectoryException(
            name_, "Nonrotation command near goal.");
  }
  return score_rotation(trajectory);
}

double HysteresisRotateToGoalCritic::score_rotation(
  const dwb_msgs::msg::Trajectory2D & trajectory) const
{
  if (trajectory.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "Empty trajectory.");
  }
  const double end_yaw = lookahead_time_ >= 0.0 ?
    project_pose(trajectory, lookahead_time_).theta :
    trajectory.poses.back().theta;
  return std::fabs(
    angles::shortest_angular_distance(end_yaw, goal_yaw_));
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::HysteresisRotateToGoalCritic,
  dwb_core::TrajectoryCritic)
