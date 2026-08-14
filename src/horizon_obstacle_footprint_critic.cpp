/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

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
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".maximum_swept_distance",
    rclcpp::ParameterValue(0.5 * costmap_->getResolution()));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".maximum_swept_distance",
    maximum_swept_distance_);
  if (!std::isfinite(score_time_horizon_) || score_time_horizon_ < 0.0 ||
    !std::isfinite(maximum_swept_distance_) || maximum_swept_distance_ <= 0.0)
  {
    throw std::runtime_error{
            "HorizonObstacleFootprintCritic parameters must be finite"};
  }
}

bool HorizonObstacleFootprintCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  if (!dwb_critics::ObstacleFootprintCritic::prepare(
      pose, velocity, goal, global_plan))
  {
    return false;
  }
  footprint_radius_ = 0.0;
  for (const auto & point : footprint_spec_) {
    footprint_radius_ = std::max(
      footprint_radius_, std::hypot(point.x, point.y));
  }
  return std::isfinite(footprint_radius_);
}

double HorizonObstacleFootprintCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "Trajectory has no poses.");
  }
  for (std::size_t index = 0u; index < trajectory.poses.size(); ++index) {
    const auto & pose = trajectory.poses[index];
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta))
    {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Trajectory contains a non-finite pose.");
    }

    // Numeric legal-cell cost is intentionally ignored. This critic defines
    // only the physical hard gate; the common soft critics rank legal poses.
    static_cast<void>(scorePose(pose));

    if (index == 0u) {
      continue;
    }
    const auto & previous = trajectory.poses[index - 1u];
    const double delta_x = pose.x - previous.x;
    const double delta_y = pose.y - previous.y;
    const double delta_yaw = std::remainder(
      pose.theta - previous.theta, 2.0 * M_PI);
    const double corner_sweep =
      std::hypot(delta_x, delta_y) +
      footprint_radius_ * std::abs(delta_yaw);
    if (!std::isfinite(corner_sweep)) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Trajectory sweep is non-finite.");
    }
    const std::size_t subdivisions = std::max<std::size_t>(
      1u, static_cast<std::size_t>(
        std::ceil(corner_sweep / maximum_swept_distance_)));
    constexpr std::size_t kMaximumSubdivisions = 10000u;
    if (subdivisions > kMaximumSubdivisions) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Trajectory sweep requires too many samples.");
    }
    for (std::size_t subdivision = 1u;
      subdivision < subdivisions; ++subdivision)
    {
      const double ratio = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      geometry_msgs::msg::Pose2D intermediate;
      intermediate.x = previous.x + ratio * delta_x;
      intermediate.y = previous.y + ratio * delta_y;
      intermediate.theta = previous.theta + ratio * delta_yaw;
      // Ignore the legal-cell numeric cost: this sample exists solely to
      // prevent a physical footprint from crossing a lethal/unknown cell
      // between two 50 ms rollout poses.
      static_cast<void>(scorePose(intermediate));
    }
  }
  return 0.0;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::HorizonObstacleFootprintCritic,
  dwb_core::TrajectoryCritic)
