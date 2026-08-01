/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/forward_obstacle_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void ForwardObstacleCritic::onInit()
{
  auto node = node_.lock();
  if (!node || !costmap_ros_) {
    throw std::runtime_error{"ForwardObstacleCritic initialization failed"};
  }
  costmap_ = costmap_ros_->getCostmap();
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".sample_resolution",
    rclcpp::ParameterValue(costmap_->getResolution()));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".lookahead_distance",
    lookahead_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".sample_resolution",
    sample_resolution_);
  if (!std::isfinite(lookahead_distance_) || lookahead_distance_ < 0.0 ||
    !std::isfinite(sample_resolution_) || sample_resolution_ <= 0.0)
  {
    throw std::runtime_error{
            "ForwardObstacleCritic distances must be finite and positive"};
  }
}

double ForwardObstacleCritic::normalizedCostAt(
  const geometry_msgs::msg::Pose2D & pose) const
{
  unsigned int cell_x = 0u;
  unsigned int cell_y = 0u;
  if (!costmap_ || !costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    return 1.0;
  }
  const unsigned char cost = costmap_->getCost(cell_x, cell_y);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return 1.0;
  }
  return static_cast<double>(cost) /
         static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
}

double ForwardObstacleCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.empty() || lookahead_distance_ <= 0.0) {
    return 0.0;
  }
  const auto & endpoint = trajectory.poses.back();
  const double cos_theta = std::cos(endpoint.theta);
  const double sin_theta = std::sin(endpoint.theta);
  double maximum_cost = 0.0;
  for (double distance = sample_resolution_;
    distance <= lookahead_distance_ + 1.0e-9;
    distance += sample_resolution_)
  {
    geometry_msgs::msg::Pose2D probe = endpoint;
    probe.x += distance * cos_theta;
    probe.y += distance * sin_theta;
    maximum_cost = std::max(maximum_cost, normalizedCostAt(probe));
  }
  return maximum_cost;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::ForwardObstacleCritic,
  dwb_core::TrajectoryCritic)
