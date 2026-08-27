/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__LINEAR_MAP_GRID_CRITICS_HPP_
#define F_DWA_CONTROLLER__LINEAR_MAP_GRID_CRITICS_HPP_

#include "dwb_critics/goal_align.hpp"
#include "dwb_critics/goal_dist.hpp"
#include "dwb_critics/path_align.hpp"
#include "dwb_critics/path_dist.hpp"

namespace f_dwa_controller
{

/**
 * @brief PathDist with the same Manhattan field as Nav2, built in linear time.
 */
class LinearPathDistCritic : public dwb_critics::PathDistCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
};

/**
 * @brief PathAlign with the same Manhattan field as Nav2, built in linear time.
 */
class LinearPathAlignCritic : public dwb_critics::PathAlignCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
};

/**
 * @brief GoalAlign with the same Manhattan field as Nav2, built in linear time.
 */
class LinearGoalAlignCritic : public dwb_critics::GoalAlignCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
};

/**
 * @brief GoalDist with the same Manhattan field as Nav2, built in linear time.
 */
class LinearGoalDistCritic : public dwb_critics::GoalDistCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__LINEAR_MAP_GRID_CRITICS_HPP_
