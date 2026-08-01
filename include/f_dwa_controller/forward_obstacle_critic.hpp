/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_
#define F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace f_dwa_controller
{

/**
 * @brief Softly score the observable space beyond a candidate endpoint.
 *
 * DWB replans before this extension is executed. It therefore never rejects a
 * trajectory and is not a safety certificate; it only exposes whether the
 * terminal heading leaves room for the next receding-horizon cycle.
 */
class ForwardObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;

protected:
  virtual double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  double lookahead_distance_{1.2};
  double sample_resolution_{0.05};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FORWARD_OBSTACLE_CRITIC_HPP_
