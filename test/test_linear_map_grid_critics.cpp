/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <memory>

#include "dwb_critics/goal_align.hpp"
#include "dwb_critics/goal_dist.hpp"
#include "dwb_critics/path_align.hpp"
#include "dwb_critics/path_dist.hpp"
#include "f_dwa_controller/linear_map_grid_critics.hpp"
#include "f_dwa_controller/mean_path_dist_critic.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

template<typename CriticT>
class GridCriticAdapter : public CriticT
{
public:
  void initialize(nav2_costmap_2d::Costmap2D & costmap)
  {
    this->costmap_ = &costmap;
    this->queue_ = std::make_shared<typename CriticT::MapGridQueue>(
      costmap, *this);
  }
};

template<typename CriticT>
class AlignCriticAdapter : public GridCriticAdapter<CriticT>
{
public:
  void set_forward_point_distance(const double distance)
  {
    this->forward_point_distance_ = distance;
  }
};

nav_2d_msgs::msg::Path2D make_plan()
{
  nav_2d_msgs::msg::Path2D plan;
  for (int index = 0; index <= 8; ++index) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = -1.2 + 0.3 * index;
    pose.y = -0.55 + 0.13 * index;
    plan.poses.push_back(pose);
  }
  return plan;
}

template<typename ReferenceCriticT, typename LinearCriticT>
void expect_identical_fields(
  ReferenceCriticT & reference,
  LinearCriticT & linear,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav_2d_msgs::msg::Path2D & plan,
  const geometry_msgs::msg::Pose2D & pose,
  const geometry_msgs::msg::Pose2D & goal)
{
  const nav_2d_msgs::msg::Twist2D velocity;
  ASSERT_EQ(
    reference.prepare(pose, velocity, goal, plan),
    linear.prepare(pose, velocity, goal, plan));
  for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x) {
      EXPECT_DOUBLE_EQ(reference.getScore(x, y), linear.getScore(x, y))
        << "cell=(" << x << ',' << y << ')';
    }
  }
}

TEST(LinearMapGridCritics, PathDistanceMatchesNav2AtEveryCell)
{
  nav2_costmap_2d::Costmap2D costmap(31u, 23u, 0.1, -1.5, -1.0);
  costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  const auto plan = make_plan();
  geometry_msgs::msg::Pose2D pose;
  pose.x = -1.0;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1.1;
  goal.y = 0.5;

  GridCriticAdapter<dwb_critics::PathDistCritic> reference;
  GridCriticAdapter<f_dwa_controller::LinearPathDistCritic> linear;
  reference.initialize(costmap);
  linear.initialize(costmap);
  expect_identical_fields(reference, linear, costmap, plan, pose, goal);

  GridCriticAdapter<f_dwa_controller::MeanPathDistCritic> mean;
  mean.initialize(costmap);
  expect_identical_fields(reference, mean, costmap, plan, pose, goal);
}

TEST(LinearMapGridCritics, PathAlignMatchesNav2AtEveryCell)
{
  nav2_costmap_2d::Costmap2D costmap(31u, 23u, 0.1, -1.5, -1.0);
  costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  const auto plan = make_plan();
  geometry_msgs::msg::Pose2D pose;
  pose.x = -1.0;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1.1;
  goal.y = 0.5;

  AlignCriticAdapter<dwb_critics::PathAlignCritic> reference;
  AlignCriticAdapter<f_dwa_controller::LinearPathAlignCritic> linear;
  reference.initialize(costmap);
  linear.initialize(costmap);
  reference.set_forward_point_distance(0.6);
  linear.set_forward_point_distance(0.6);
  expect_identical_fields(reference, linear, costmap, plan, pose, goal);
}

TEST(LinearMapGridCritics, GoalDistanceMatchesNav2AtEveryCell)
{
  nav2_costmap_2d::Costmap2D costmap(31u, 23u, 0.1, -1.5, -1.0);
  costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  const auto plan = make_plan();
  geometry_msgs::msg::Pose2D pose;
  pose.x = -1.0;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1.1;
  goal.y = 0.5;

  GridCriticAdapter<dwb_critics::GoalDistCritic> reference;
  GridCriticAdapter<f_dwa_controller::LinearGoalDistCritic> linear;
  reference.initialize(costmap);
  linear.initialize(costmap);
  expect_identical_fields(reference, linear, costmap, plan, pose, goal);
}

TEST(LinearMapGridCritics, GoalAlignMatchesNav2AtEveryCell)
{
  nav2_costmap_2d::Costmap2D costmap(31u, 23u, 0.1, -1.5, -1.0);
  costmap.setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  const auto plan = make_plan();
  geometry_msgs::msg::Pose2D pose;
  pose.x = -1.0;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1.1;
  goal.y = 0.5;

  AlignCriticAdapter<dwb_critics::GoalAlignCritic> reference;
  AlignCriticAdapter<f_dwa_controller::LinearGoalAlignCritic> linear;
  reference.initialize(costmap);
  linear.initialize(costmap);
  reference.set_forward_point_distance(0.6);
  linear.set_forward_point_distance(0.6);
  expect_identical_fields(reference, linear, costmap, plan, pose, goal);
}

}  // namespace
