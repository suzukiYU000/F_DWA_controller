/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/forward_obstacle_critic.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/fixed_distance_risk_path.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/line_iterator.hpp"
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
    node, dwb_plugin_name_ + "." + name_ + ".risk_distance",
    rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".risk_seed_time",
    rclcpp::ParameterValue(1.4));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".heading_relaxation_distance",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".sample_resolution",
    rclcpp::ParameterValue(costmap_->getResolution()));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".distance_weighting_power",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".peak_weight",
    rclcpp::ParameterValue(0.25));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".risk_distance",
    risk_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".risk_seed_time",
    risk_seed_time_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".heading_relaxation_distance",
    heading_relaxation_distance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".sample_resolution",
    sample_resolution_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".distance_weighting_power",
    distance_weighting_power_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".peak_weight", peak_weight_);
  if (!std::isfinite(risk_distance_) || risk_distance_ <= 0.0 ||
    !std::isfinite(risk_seed_time_) || risk_seed_time_ <= 0.0 ||
    !std::isfinite(heading_relaxation_distance_) ||
    heading_relaxation_distance_ <= 0.0 ||
    !std::isfinite(sample_resolution_) || sample_resolution_ <= 0.0 ||
    !std::isfinite(distance_weighting_power_) ||
    distance_weighting_power_ < 0.0 || !std::isfinite(peak_weight_) ||
    peak_weight_ < 0.0 || peak_weight_ > 1.0)
  {
    throw std::runtime_error{
            "ForwardObstacleCritic distances must be finite and positive"};
  }
}

bool ForwardObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  prepared_plan_geometry_ = PreparedPlanGeometry{};
  global_plan_.poses.clear();
  try {
    prepared_plan_geometry_ =
      prepare_plan_continuation_geometry(global_plan);
  } catch (const std::invalid_argument &) {
    return false;
  }
  global_plan_ = global_plan;
  return true;
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
  const auto & risk_path = [this, &trajectory]()
    -> const std::vector<RiskPathSample> &
    {
      try {
        if (!prepared_plan_geometry_.empty()) {
          return build_plan_continued_risk_path(
            trajectory, prepared_plan_geometry_, risk_distance_,
            sample_resolution_, risk_seed_time_,
            heading_relaxation_distance_, risk_path_workspace_);
        }
        // Protected global_plan_ predates the prepared cache and is used by
        // lightweight derived tests. Production prepare() always takes the
        // cache branch above.
        return build_plan_continued_risk_path(
          trajectory, global_plan_, risk_distance_, sample_resolution_,
          risk_seed_time_, heading_relaxation_distance_,
          risk_path_workspace_);
      } catch (const std::invalid_argument & exception) {
        throw dwb_core::IllegalTrajectoryException(name_, exception.what());
      }
    }();
  double maximum_cost = 0.0;
  const auto weighted_cost_at =
    [this](
    const geometry_msgs::msg::Pose2D & pose, const double arc_length)
    {
      const double normalized_distance = std::clamp(
        (arc_length - sample_resolution_) /
        std::max(risk_distance_, sample_resolution_),
        0.0, 1.0);
      const double proximity_weight = std::pow(
        1.0 - normalized_distance, distance_weighting_power_);
      return normalizedCostAt(pose) * proximity_weight;
    };
  const auto consider_peak =
    [&maximum_cost, &weighted_cost_at](
    const geometry_msgs::msg::Pose2D & pose, const double arc_length)
    {
      maximum_cost = std::max(
        maximum_cost, weighted_cost_at(pose, arc_length));
    };

  // A pure maximum saturates as soon as every candidate ray touches the same
  // inflated obstacle and then provides almost no gradient for taking a wider
  // detour. Integrate the fixed spatial path as the primary score so leaving
  // the obstacle field earlier is always cheaper. Keep a bounded peak term to
  // retain isolated-cell visibility between the fixed samples rasterized
  // below. The total observation distance remains independent of sim_time.
  double exposure_integral = 0.0;
  double previous_cost = weighted_cost_at(
    risk_path.front().pose, risk_path.front().arc_length);
  maximum_cost = previous_cost;
  for (std::size_t index = 1u; index < risk_path.size(); ++index) {
    const double current_cost = weighted_cost_at(
      risk_path[index].pose, risk_path[index].arc_length);
    const double interval =
      risk_path[index].arc_length - risk_path[index - 1u].arc_length;
    if (!std::isfinite(interval) || interval <= 0.0) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "fixed-distance obstacle path is not strictly ordered");
    }
    exposure_integral += 0.5 * interval *
      (previous_cost + current_cost);
    maximum_cost = std::max(maximum_cost, current_cost);
    previous_cost = current_cost;
  }

  if (!costmap_) {
    // Unit-test and subclass fallback. Production scoring below rasterizes
    // every costmap cell crossed between spatial samples.
    const double mean_cost = std::clamp(
      exposure_integral / risk_distance_, 0.0, 1.0);
    return peak_weight_ * maximum_cost +
           (1.0 - peak_weight_) * mean_cost;
  }

  for (std::size_t index = 1u; index < risk_path.size(); ++index) {
    const auto & first = risk_path[index - 1u];
    const auto & second = risk_path[index];
    unsigned int first_x = 0u;
    unsigned int first_y = 0u;
    unsigned int second_x = 0u;
    unsigned int second_y = 0u;
    const bool first_is_on_map = costmap_->worldToMap(
      first.pose.x, first.pose.y, first_x, first_y);
    const bool second_is_on_map = costmap_->worldToMap(
      second.pose.x, second.pose.y, second_x, second_y);
    if (!first_is_on_map || !second_is_on_map) {
      // Off-map is the maximum soft risk. Evaluating both ends also preserves
      // the distance weighting at the boundary where the segment leaves it.
      if (!first_is_on_map && first.arc_length <= 1.0e-9) {
        maximum_cost = 1.0;
      } else {
        consider_peak(first.pose, first.arc_length);
      }
      consider_peak(second.pose, second.arc_length);
      continue;
    }

    const double delta_x = second.pose.x - first.pose.x;
    const double delta_y = second.pose.y - first.pose.y;
    const double squared_length = delta_x * delta_x + delta_y * delta_y;
    for (nav2_util::LineIterator line(
        static_cast<int>(first_x), static_cast<int>(first_y),
        static_cast<int>(second_x), static_cast<int>(second_y));
      line.isValid(); line.advance())
    {
      const unsigned int cell_x = static_cast<unsigned int>(line.getX());
      const unsigned int cell_y = static_cast<unsigned int>(line.getY());
      double world_x = 0.0;
      double world_y = 0.0;
      costmap_->mapToWorld(cell_x, cell_y, world_x, world_y);
      const double ratio = squared_length > 1.0e-18 ?
        std::clamp(
        ((world_x - first.pose.x) * delta_x +
        (world_y - first.pose.y) * delta_y) / squared_length,
        0.0, 1.0) : 1.0;
      geometry_msgs::msg::Pose2D probe = first.pose;
      probe.x = world_x;
      probe.y = world_y;
      probe.theta = first.pose.theta + ratio * std::remainder(
        second.pose.theta - first.pose.theta, 2.0 * M_PI);
      const double arc_length = first.arc_length + ratio *
        (second.arc_length - first.arc_length);
      consider_peak(probe, arc_length);
    }
  }
  const double mean_cost = std::clamp(
    exposure_integral / risk_distance_, 0.0, 1.0);
  return peak_weight_ * maximum_cost +
         (1.0 - peak_weight_) * mean_cost;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::ForwardObstacleCritic,
  dwb_core::TrajectoryCritic)
