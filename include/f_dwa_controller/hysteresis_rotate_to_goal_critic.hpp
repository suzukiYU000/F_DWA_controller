/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Locus Robotics
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__HYSTERESIS_ROTATE_TO_GOAL_CRITIC_HPP_
#define F_DWA_CONTROLLER__HYSTERESIS_ROTATE_TO_GOAL_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Nav2 RotateToGoal behavior with an optional release hysteresis.
 *
 * A delayed command can carry the robot outside the terminal window after
 * upstream RotateToGoal has latched rotation-only mode. A non-negative
 * xy_goal_tolerance_release_margin releases that latch only beyond the outer
 * window. The default negative value preserves upstream one-way latching.
 */
class HysteresisRotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  double score_rotation(
    const dwb_msgs::msg::Trajectory2D & trajectory) const;

  bool in_window_{false};
  bool rotating_{false};
  bool release_window_latch_{false};
  bool ignore_goal_orientation_{false};
  double goal_yaw_{0.0};
  double xy_goal_tolerance_sq_{0.0};
  double xy_goal_tolerance_release_sq_{0.0};
  double current_xy_speed_sq_{0.0};
  double stopped_xy_velocity_sq_{0.0};
  double slowing_factor_{5.0};
  double lookahead_time_{-1.0};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__HYSTERESIS_ROTATE_TO_GOAL_CRITIC_HPP_
