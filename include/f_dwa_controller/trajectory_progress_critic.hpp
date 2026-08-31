/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_
#define F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_

#include <cstddef>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Penalize failure to advance along the observable reference Path.
 *
 * The raw score is a non-negative distance in meters. It is zero when a
 * trajectory reaches the configured lookahead arclength and increases as the
 * maximum effective progress of the trajectory decreases. Effective progress
 * is Path arclength advance minus newly introduced cross-track distance, so a
 * lateral shortcut across a bend cannot masquerade as forward Path progress.
 * This preserves DWB's non-negative-score short-circuit assumption.
 */
class TrajectoryProgressCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;

protected:
  struct PathProjection
  {
    double progress{0.0};
    double distance{0.0};
  };

  struct PathSegment
  {
    double start_x{0.0};
    double start_y{0.0};
    double delta_x{0.0};
    double delta_y{0.0};
    double inverse_squared_length{0.0};
    double length{0.0};
    double cumulative_start{0.0};
    double minimum_x{0.0};
    double maximum_x{0.0};
    double minimum_y{0.0};
    double maximum_y{0.0};
  };

  PathProjection projectOntoPath(
    const geometry_msgs::msg::Pose2D & pose,
    std::size_t & segment_hint) const;

  nav_2d_msgs::msg::Path2D path_;
  std::vector<double> cumulative_distance_;
  std::vector<PathSegment> path_segments_;
  std::size_t current_segment_hint_{0u};
  double current_progress_{0.0};
  double current_cross_track_distance_{0.0};
  double desired_progress_{0.0};
  double lookahead_distance_{2.88};
  double lateral_distance_weight_{1.0};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_
