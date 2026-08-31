/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_
#define F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_

#include <vector>

#include "dwb_core/trajectory_critic.hpp"

namespace f_dwa_controller
{

/**
 * @brief Apply a finite, recovery-directed soft cost outside a Path corridor.
 *
 * If any predicted pose is farther than the configured distance, the critic
 * returns one finite boundary penalty plus the mean excess-distance cost.
 * Near that boundary, a smooth heading term also penalizes a terminal heading
 * whose forward probe moves farther from the Path. The probe activates the
 * term before the rollout endpoint itself leaves the corridor, giving
 * acceleration/jerk/FIR rollouts enough lead distance to turn smoothly. It
 * never rejects a trajectory, so obstacle avoidance remains selectable and
 * the physical-footprint critic stays the independent hard gate.
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
  struct PathSegment
  {
    double start_x{0.0};
    double start_y{0.0};
    double delta_x{0.0};
    double delta_y{0.0};
    double inverse_squared_length{0.0};
    double minimum_x{0.0};
    double maximum_x{0.0};
    double minimum_y{0.0};
    double maximum_y{0.0};
  };

  void validateParameters() const;
  double distanceToPath(
    const geometry_msgs::msg::Pose2D & pose,
    std::size_t & segment_hint) const;

  nav_2d_msgs::msg::Path2D reference_path_;
  std::vector<PathSegment> path_segments_;
  double maximum_path_distance_{1.5};
  double deviation_penalty_{1000.0};
  double excess_distance_scale_{1000.0};
  double heading_recovery_activation_distance_{1.05};
  double heading_recovery_lookahead_distance_{0.9};
  double heading_recovery_scale_{200.0};
  bool reference_path_valid_{false};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__PATH_DEVIATION_CRITIC_HPP_
