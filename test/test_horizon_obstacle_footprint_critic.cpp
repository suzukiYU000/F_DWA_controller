/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <cstdint>
#include <initializer_list>

#include "builtin_interfaces/msg/duration.hpp"
#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"
#include "gtest/gtest.h"

namespace
{

class StubHorizonObstacleFootprintCritic
  : public f_dwa_controller::HorizonObstacleFootprintCritic
{
public:
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override
  {
    if (pose.x < 0.0) {
      throw dwb_core::IllegalTrajectoryException(
              "HorizonObstacleFootprint", "lethal_obstacle");
    }
    return pose.x;
  }
};

dwb_msgs::msg::Trajectory2D make_trajectory(
  const std::initializer_list<double> costs)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  double time_seconds = 0.5;
  for (const double cost : costs) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = cost;
    trajectory.poses.push_back(pose);
    builtin_interfaces::msg::Duration time_offset;
    time_offset.sec = static_cast<std::int32_t>(time_seconds);
    time_offset.nanosec = static_cast<std::uint32_t>(
      (time_seconds - static_cast<double>(time_offset.sec)) * 1.0e9);
    trajectory.time_offsets.push_back(time_offset);
    time_seconds += 0.5;
  }
  return trajectory;
}

}  // namespace

TEST(HorizonObstacleFootprintCritic, ScoresOnlyDefaultHorizon)
{
  StubHorizonObstacleFootprintCritic critic;
  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(make_trajectory({2.0, 7.0, 100.0})), 4.5);
}

TEST(HorizonObstacleFootprintCritic, NormalizesForScoredPoseCount)
{
  StubHorizonObstacleFootprintCritic critic;
  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(make_trajectory({3.0, 3.0, 100.0})), 3.0);
}

TEST(HorizonObstacleFootprintCritic, IgnoresCollisionBeyondScoreHorizon)
{
  StubHorizonObstacleFootprintCritic critic;
  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(make_trajectory({2.0, 7.0, -1.0})), 4.5);
}

TEST(HorizonObstacleFootprintCritic, RejectsCollisionInsideScoreHorizon)
{
  StubHorizonObstacleFootprintCritic critic;
  EXPECT_THROW(
    critic.scoreTrajectory(make_trajectory({2.0, -1.0, 100.0})),
    dwb_core::IllegalTrajectoryException);
}
