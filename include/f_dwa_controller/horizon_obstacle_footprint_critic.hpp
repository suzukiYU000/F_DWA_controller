/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_
#define F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_

#include "dwb_critics/obstacle_footprint.hpp"

namespace f_dwa_controller
{

/**
 * @brief Score the executable obstacle-avoidance horizon.
 *
 * Poses through score_time_horizon use Nav2's ObstacleFootprintCritic and are
 * averaged so the scale does not depend on the sampling interval. The common
 * safety certificate independently validates the delayed first command and
 * its complete stopping trajectory; later poses in the DWB constant-control
 * scoring rollout are not executed before replanning.
 */
class HorizonObstacleFootprintCritic
  : public dwb_critics::ObstacleFootprintCritic
{
public:
  void onInit() override;
  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;

protected:
  double score_time_horizon_{1.25};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_
