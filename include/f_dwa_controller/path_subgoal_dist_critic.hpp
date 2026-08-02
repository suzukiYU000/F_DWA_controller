/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__PATH_SUBGOAL_DIST_CRITIC_HPP_
#define F_DWA_CONTROLLER__PATH_SUBGOAL_DIST_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Score the nominal trajectory endpoint against a Path lookahead point.
 *
 * Safety certification remains responsible for the appended stopping
 * trajectory. This critic intentionally scores the nominal DWB horizon, as in
 * the earlier evaluator's local subgoal term.
 */
class PathSubgoalDistCritic : public dwb_core::TrajectoryCritic
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
  geometry_msgs::msg::Pose2D subgoal_;
  double lookahead_distance_{1.5};
  bool allow_forward_overshoot_{false};
  bool subgoal_valid_{false};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__PATH_SUBGOAL_DIST_CRITIC_HPP_
