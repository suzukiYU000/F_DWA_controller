/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <memory>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/hysteresis_rotate_to_goal_critic.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

TEST(HysteresisRotateToGoalCritic, ReleasesOutsideOuterWindow)
{
  auto critic =
    std::make_shared<
    f_dwa_controller::HysteresisRotateToGoalCritic>();
  auto costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "hysteresis_rotate_costmap");
  auto node =
    nav2_util::LifecycleNode::make_shared(
    "hysteresis_rotate_node");
  node->configure();
  node->activate();

  const std::string critic_name = "RotateToGoal";
  const std::string controller_name = "FollowPath";
  nav2_util::declare_parameter_if_not_declared(
    node, controller_name + ".xy_goal_tolerance",
    rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, controller_name + ".trans_stopped_velocity",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node,
    controller_name + "." + critic_name +
    ".xy_goal_tolerance_release_margin",
    rclcpp::ParameterValue(0.03));
  critic->initialize(
    node, critic_name, controller_name, costmap_ros);

  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Twist2D stopped_velocity;
  nav_2d_msgs::msg::Path2D path;
  dwb_msgs::msg::Trajectory2D translating_trajectory;
  translating_trajectory.poses.resize(1);
  translating_trajectory.velocity.x = 0.1;

  pose.x = 0.19;
  ASSERT_TRUE(critic->prepare(
      pose, stopped_velocity, goal, path));
  EXPECT_THROW(
    critic->scoreTrajectory(translating_trajectory),
    dwb_core::IllegalTrajectoryException);

  pose.x = 0.229;
  ASSERT_TRUE(critic->prepare(
      pose, stopped_velocity, goal, path));
  EXPECT_THROW(
    critic->scoreTrajectory(translating_trajectory),
    dwb_core::IllegalTrajectoryException);

  pose.x = 0.24;
  ASSERT_TRUE(critic->prepare(
      pose, stopped_velocity, goal, path));
  EXPECT_DOUBLE_EQ(
    critic->scoreTrajectory(translating_trajectory), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
