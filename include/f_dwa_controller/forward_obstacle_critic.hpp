/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_
#define F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"
#include "f_dwa_controller/fixed_distance_risk_path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace f_dwa_controller
{

/**
 * @brief Softly score one fixed spatial horizon for every candidate.
 *
 * DWB replans before the terminal extension is executed. It therefore never
 * rejects a trajectory and is not a safety certificate; it only provides a
 * common terminal value over a distance that does not change with sim_time.
 * The nominal seed is cut at risk_seed_time and continued along the prepared
 * transformed plan. Its spatial heading relaxes back to the plan over
 * heading_relaxation_distance, so a longer rollout suffix cannot change this
 * prediction while an early avoidance turn retains a useful soft gradient.
 */
class ForwardObstacleCritic : public dwb_core::TrajectoryCritic
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
  virtual double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav_2d_msgs::msg::Path2D global_plan_;
  PreparedPlanGeometry prepared_plan_geometry_;
  double risk_distance_{2.5};
  double risk_seed_time_{1.4};
  double heading_relaxation_distance_{1.0};
  double sample_resolution_{0.05};
  double distance_weighting_power_{1.0};
  double peak_weight_{0.25};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_
