/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <cstdint>

#include "builtin_interfaces/msg/duration.hpp"
#include "f_dwa_controller/forward_obstacle_critic.hpp"
#include "f_dwa_controller/mean_path_dist_critic.hpp"
#include "f_dwa_controller/mean_speed_critic.hpp"
#include "f_dwa_controller/trajectory_progress_critic.hpp"
#include "gtest/gtest.h"

namespace
{

class StubMeanPathDistCritic : public f_dwa_controller::MeanPathDistCritic
{
public:
  StubMeanPathDistCritic()
  {
    obstacle_score_ = -1.0;
    unreachable_score_ = -2.0;
  }

  double scorePose(const geometry_msgs::msg::Pose2D & pose) override
  {
    return pose.y;
  }
};

class StubForwardObstacleCritic : public f_dwa_controller::ForwardObstacleCritic
{
public:
  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return pose.y < -0.1 ? 1.0 : 0.0;
  }
};

builtin_interfaces::msg::Duration duration(double seconds)
{
  builtin_interfaces::msg::Duration value;
  value.sec = static_cast<std::int32_t>(seconds);
  value.nanosec = static_cast<std::uint32_t>(
    (seconds - static_cast<double>(value.sec)) * 1.0e9);
  return value;
}

nav_2d_msgs::msg::Path2D straight_path(double length)
{
  nav_2d_msgs::msg::Path2D path;
  geometry_msgs::msg::Pose2D start;
  geometry_msgs::msg::Pose2D end;
  end.x = length;
  path.poses = {start, end};
  return path;
}

dwb_msgs::msg::Trajectory2D trajectory_to(double end_x, double seconds)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D first;
  first.x = 0.5 * end_x;
  geometry_msgs::msg::Pose2D last;
  last.x = end_x;
  trajectory.poses = {first, last};
  trajectory.time_offsets = {duration(0.5 * seconds), duration(seconds)};
  return trajectory;
}

}  // namespace

TEST(MeanPathDistCritic, AveragesEveryPredictedPose)
{
  StubMeanPathDistCritic critic;
  auto trajectory = trajectory_to(1.0, 1.0);
  trajectory.poses[0].y = 1.0;
  trajectory.poses[1].y = 3.0;
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory), 2.0);
}

TEST(ForwardObstacleCritic, PenalizesBlockedEndpointHeading)
{
  StubForwardObstacleCritic critic;
  auto downward = trajectory_to(1.0, 1.0);
  downward.poses.back().theta = -0.5;
  auto upward = downward;
  upward.poses.back().theta = 0.5;

  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(downward), 1.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(upward), 0.0);
}

TEST(TrajectoryProgressCritic, RewardsFurthestProgressWithNonnegativeScores)
{
  f_dwa_controller::TrajectoryProgressCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, straight_path(4.0)));

  EXPECT_NEAR(critic.scoreTrajectory(trajectory_to(1.0, 1.0)), 1.88, 1.0e-9);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory_to(2.0, 1.0)), 0.88, 1.0e-9);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory_to(3.0, 1.0)), 0.0);
}

TEST(TrajectoryProgressCritic, DoesNotRewardLateralDistance)
{
  f_dwa_controller::TrajectoryProgressCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, straight_path(4.0)));
  auto trajectory = trajectory_to(1.0, 1.0);
  trajectory.poses.back().y = 3.0;
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 1.88, 1.0e-9);
}

TEST(TrajectoryProgressCritic, IsNeutralForSinglePosePlanNearGoal)
{
  f_dwa_controller::TrajectoryProgressCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  path.poses.push_back(goal);
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory_to(1.0, 1.0)), 0.0);
}

TEST(MeanSpeedCritic, UsesPredictedMotionInsteadOfRequestedVelocity)
{
  f_dwa_controller::MeanSpeedCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  auto trajectory = trajectory_to(0.6, 1.0);
  trajectory.velocity.x = 1.2;
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.6, 1.0e-9);
}

TEST(MeanSpeedCritic, NeverReturnsNegativeCost)
{
  f_dwa_controller::MeanSpeedCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory_to(2.0, 1.0)), 0.0);
}

TEST(MeanSpeedCritic, AcceptsUpstreamDwbStartPoseConvention)
{
  f_dwa_controller::MeanSpeedCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  auto trajectory = trajectory_to(0.6, 1.0);
  trajectory.time_offsets.erase(trajectory.time_offsets.begin());
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.6, 1.0e-9);
}
