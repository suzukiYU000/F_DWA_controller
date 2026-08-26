/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/path_deviation_critic.hpp"

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
  return reference_path_valid_;
}

double PathDeviationCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!reference_path_valid_ || trajectory.poses.empty()) {
    return 0.0;
  }

  for (const auto & pose : trajectory.poses) {
    PathProjection projection;
    if (!project_pose_onto_path(reference_path_, pose, projection) ||
      projection.distance > maximum_path_distance_)
    {
      return deviation_penalty_;
    }
  }
  return 0.0;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::PathDeviationCritic,
  dwb_core::TrajectoryCritic)
