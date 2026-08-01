/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/mean_path_dist_critic.hpp"

#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

double MeanPathDistCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.empty()) {
    return 0.0;
  }

  double score = 0.0;
  for (const auto & pose : trajectory.poses) {
    const double grid_distance = scorePose(pose);
    if (grid_distance == obstacle_score_) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Trajectory Hits Obstacle.");
    }
    if (grid_distance == unreachable_score_) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "Trajectory Hits Unreachable Area.");
    }
    score += grid_distance;
  }
  return score / static_cast<double>(trajectory.poses.size());
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::MeanPathDistCritic,
  dwb_core::TrajectoryCritic)
