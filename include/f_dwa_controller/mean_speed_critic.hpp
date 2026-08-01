/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__MEAN_SPEED_CRITIC_HPP_
#define F_DWA_CONTROLLER__MEAN_SPEED_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Penalize low mean predicted translational speed.
 *
 * Speed is reconstructed from predicted poses and time offsets rather than
 * the requested command, so A/J/F internal dynamics are evaluated on the same
 * physical quantity. The critic uses no state unavailable on the robot.
 */
class MeanSpeedCritic : public dwb_core::TrajectoryCritic
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
  geometry_msgs::msg::Pose2D current_pose_;
  double target_speed_{1.2};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__MEAN_SPEED_CRITIC_HPP_
