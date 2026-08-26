/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_
#define F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Apply one fixed soft cost when any predicted pose leaves a Path corridor.
 *
 * Poses on the corridor boundary remain neutral. The critic never rejects a
 * trajectory, and the penalty does not grow with the amount of deviation.
 */
class PathDeviationCritic : public dwb_core::TrajectoryCritic
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
  void validateParameters() const;

  nav_2d_msgs::msg::Path2D reference_path_;
  double maximum_path_distance_{1.5};
  double deviation_penalty_{1000.0};
  bool reference_path_valid_{false};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_
