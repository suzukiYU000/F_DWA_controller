/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/linear_map_grid_critics.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{
namespace
{

void initialize_grid(
  nav2_costmap_2d::Costmap2D & costmap,
  std::vector<double> & values,
  double & obstacle_score,
  double & unreachable_score)
{
  const std::size_t cell_count =
    static_cast<std::size_t>(costmap.getSizeInCellsX()) *
    static_cast<std::size_t>(costmap.getSizeInCellsY());
  obstacle_score = static_cast<double>(cell_count);
  unreachable_score = obstacle_score + 1.0;
  values.assign(cell_count, unreachable_score);
}

void propagate_manhattan_distances(
  const unsigned int size_x,
  const unsigned int size_y,
  std::vector<double> & values)
{
  for (unsigned int y = 0; y < size_y; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * size_x;
    for (unsigned int x = 0; x < size_x; ++x) {
      const std::size_t index = row + x;
      if (x > 0) {
        values[index] = std::min(values[index], values[index - 1u] + 1.0);
      }
      if (y > 0) {
        values[index] = std::min(values[index], values[index - size_x] + 1.0);
      }
    }
  }

  for (unsigned int y = size_y; y-- > 0u; ) {
    const std::size_t row = static_cast<std::size_t>(y) * size_x;
    for (unsigned int x = size_x; x-- > 0u; ) {
      const std::size_t index = row + x;
      if (x + 1u < size_x) {
        values[index] = std::min(values[index], values[index + 1u] + 1.0);
      }
      if (y + 1u < size_y) {
        values[index] = std::min(values[index], values[index + size_x] + 1.0);
      }
    }
  }
}

bool prepare_path_grid(
  nav2_costmap_2d::Costmap2D & costmap,
  const nav_2d_msgs::msg::Path2D & global_plan,
  std::vector<double> & values,
  double & obstacle_score,
  double & unreachable_score)
{
  initialize_grid(costmap, values, obstacle_score, unreachable_score);
  const nav_2d_msgs::msg::Path2D adjusted_plan =
    nav_2d_utils::adjustPlanResolution(global_plan, costmap.getResolution());

  bool started_path = false;
  for (const auto & plan_pose : adjusted_plan.poses) {
    unsigned int map_x = 0u;
    unsigned int map_y = 0u;
    if (costmap.worldToMap(plan_pose.x, plan_pose.y, map_x, map_y) &&
      costmap.getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      values[costmap.getIndex(map_x, map_y)] = 0.0;
      started_path = true;
    } else if (started_path) {
      break;
    }
  }
  if (!started_path) {
    return false;
  }
  propagate_manhattan_distances(
    costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), values);
  return true;
}

void prepare_goal_grid(
  nav2_costmap_2d::Costmap2D & costmap,
  const unsigned int goal_x,
  const unsigned int goal_y,
  std::vector<double> & values,
  double & obstacle_score,
  double & unreachable_score)
{
  initialize_grid(costmap, values, obstacle_score, unreachable_score);
  values[costmap.getIndex(goal_x, goal_y)] = 0.0;
  propagate_manhattan_distances(
    costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), values);
}

}  // namespace

bool LinearPathDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  return prepare_path_grid(
    *costmap_, global_plan, cell_values_, obstacle_score_, unreachable_score_);
}

bool LinearPathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  const double dx = pose.x - goal.x;
  const double dy = pose.y - goal.y;
  if (dx * dx + dy * dy <=
    forward_point_distance_ * forward_point_distance_)
  {
    zero_scale_ = true;
    return true;
  }
  zero_scale_ = false;
  return prepare_path_grid(
    *costmap_, global_plan, cell_values_, obstacle_score_, unreachable_score_);
}

bool LinearGoalAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  if (global_plan.poses.empty()) {
    return false;
  }
  nav_2d_msgs::msg::Path2D target_poses = global_plan;
  const double angle_to_goal = std::atan2(goal.y - pose.y, goal.x - pose.x);
  target_poses.poses.back().x +=
    forward_point_distance_ * std::cos(angle_to_goal);
  target_poses.poses.back().y +=
    forward_point_distance_ * std::sin(angle_to_goal);

  unsigned int goal_x = 0u;
  unsigned int goal_y = 0u;
  if (!getLastPoseOnCostmap(target_poses, goal_x, goal_y)) {
    return false;
  }
  prepare_goal_grid(
    *costmap_, goal_x, goal_y, cell_values_, obstacle_score_, unreachable_score_);
  return true;
}

bool LinearGoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  unsigned int goal_x = 0u;
  unsigned int goal_y = 0u;
  if (!getLastPoseOnCostmap(global_plan, goal_x, goal_y)) {
    return false;
  }
  prepare_goal_grid(
    *costmap_, goal_x, goal_y, cell_values_, obstacle_score_, unreachable_score_);
  return true;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::LinearPathAlignCritic,
  dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::LinearGoalAlignCritic,
  dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::LinearGoalDistCritic,
  dwb_core::TrajectoryCritic)
