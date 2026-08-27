/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__MEAN_PATH_DIST_CRITIC_HPP_
#define F_DWA_CONTROLLER__MEAN_PATH_DIST_CRITIC_HPP_

#include "f_dwa_controller/linear_map_grid_critics.hpp"

namespace f_dwa_controller
{

/**
 * @brief Average Nav2's path-distance grid score over the complete trajectory.
 *
 * PathDistCritic supplies the standard transformed-Path distance field. Only
 * its trajectory aggregation changes from the endpoint default to a mean, so
 * the score is independent of trajectory sampling interval.
 */
class MeanPathDistCritic : public LinearPathDistCritic
{
public:
  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__MEAN_PATH_DIST_CRITIC_HPP_
