/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__TERMINAL_APPROACH_CRITIC_HPP_
#define F_DWA_CONTROLLER__TERMINAL_APPROACH_CRITIC_HPP_

#include <optional>

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Softly prefer trajectories that can settle at the position goal.
 *
 * The critic is neutral away from the goal. Near the goal it ranks the
 * already-generated V/A/J/F rollouts by the position at which their fixed-time
 * state would stop. It neither modifies a command nor rejects a trajectory.
 */
class TerminalApproachCritic : public dwb_core::TrajectoryCritic
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
  struct EvaluationState
  {
    geometry_msgs::msg::Pose2D pose;
    double speed{0.0};
  };

  std::optional<EvaluationState> evaluationState(
    const dwb_msgs::msg::Trajectory2D & trajectory) const;
  void validateParameters() const;

  geometry_msgs::msg::Pose2D goal_;
  double approach_weight_{0.0};
  double evaluation_time_{1.4};
  double outer_distance_{1.5};
  double full_weight_distance_{0.65};
  double reference_deceleration_{0.15};
  bool include_last_point_{true};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TERMINAL_APPROACH_CRITIC_HPP_
