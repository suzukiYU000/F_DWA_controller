/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_
#define F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_

#include <vector>

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Penalize failure to advance along the observable reference Path.
 *
 * The raw score is a non-negative distance in meters. It is zero when a
 * trajectory reaches the configured lookahead arclength and increases as the
 * maximum projected progress of the trajectory decreases. This preserves
 * DWB's non-negative-score short-circuit assumption while being equivalent,
 * up to a candidate-independent constant, to rewarding path progress.
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
  double projectOntoPath(const geometry_msgs::msg::Pose2D & pose) const;

  nav_2d_msgs::msg::Path2D path_;
  std::vector<double> cumulative_distance_;
  double current_progress_{0.0};
  double desired_progress_{0.0};
  double lookahead_distance_{2.88};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TRAJECTORY_PROGRESS_CRITIC_HPP_
