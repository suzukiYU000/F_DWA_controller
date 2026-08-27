/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>

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
  void setMaximumSweptDistance(const double value)
  {
    maximum_swept_distance_ = value;
  }

  void setCollisionInterval(const double minimum, const double maximum)
  {
    collision_minimum_ = minimum;
    collision_maximum_ = maximum;
  }

  double scorePose(const geometry_msgs::msg::Pose2D & pose) override
  {
    if (pose.x < 0.0 ||
      (pose.x >= collision_minimum_ && pose.x <= collision_maximum_))
    {
      throw dwb_core::IllegalTrajectoryException(
              "HorizonObstacleFootprint", "lethal_obstacle");
    }
    return pose.x;
  }

private:
  double collision_minimum_{1000.0};
  double collision_maximum_{-1000.0};
};

dwb_msgs::msg::Trajectory2D make_trajectory(
  const std::initializer_list<double> costs)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  std::size_t pose_index = 0u;
  for (const double cost : costs) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = cost;
    trajectory.poses.push_back(pose);
    if (pose_index++ == 0u) {
      continue;
    }
    builtin_interfaces::msg::Duration time_offset;
    const double time_seconds = 0.05 * static_cast<double>(pose_index - 1u);
    time_offset.sec = static_cast<std::int32_t>(time_seconds);
    time_offset.nanosec = static_cast<std::uint32_t>(
      (time_seconds - static_cast<double>(time_offset.sec)) * 1.0e9);
    trajectory.time_offsets.push_back(time_offset);
  }
  return trajectory;
}

dwb_msgs::msg::Trajectory2D make_linear_trajectory(
  const std::size_t segment_count, const double segment_distance = 0.01)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.reserve(segment_count + 1u);
  trajectory.time_offsets.reserve(segment_count);
  for (std::size_t pose_index = 0u; pose_index <= segment_count; ++pose_index) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = segment_distance * static_cast<double>(pose_index);
    trajectory.poses.push_back(pose);
    if (pose_index == 0u) {
      continue;
    }
    builtin_interfaces::msg::Duration time_offset;
    const double time_seconds = 0.05 * static_cast<double>(pose_index);
    time_offset.sec = static_cast<std::int32_t>(time_seconds);
    time_offset.nanosec = static_cast<std::uint32_t>(
      (time_seconds - static_cast<double>(time_offset.sec)) * 1.0e9);
    trajectory.time_offsets.push_back(time_offset);
  }
  return trajectory;
}

}  // namespace

TEST(HorizonObstacleFootprintCritic, LegalScoreIsNeutralAcrossPredictionHorizons)
{
  StubHorizonObstacleFootprintCritic critic;
  const auto horizon_1p4 = make_linear_trajectory(28u);
  const auto horizon_1p6 = make_linear_trajectory(32u);
  const auto horizon_1p8 = make_linear_trajectory(36u);

  ASSERT_EQ(horizon_1p4.poses.size(), horizon_1p4.time_offsets.size() + 1u);
  ASSERT_EQ(horizon_1p6.poses.size(), horizon_1p6.time_offsets.size() + 1u);
  ASSERT_EQ(horizon_1p8.poses.size(), horizon_1p8.time_offsets.size() + 1u);
  for (std::size_t index = 0u; index < horizon_1p4.poses.size(); ++index) {
    EXPECT_DOUBLE_EQ(horizon_1p4.poses[index].x, horizon_1p6.poses[index].x);
    EXPECT_DOUBLE_EQ(horizon_1p4.poses[index].x, horizon_1p8.poses[index].x);
  }
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(horizon_1p4), 0.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(horizon_1p6), 0.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(horizon_1p8), 0.0);
}

TEST(HorizonObstacleFootprintCritic, LongerNominalSuffixCanChangeOnlyHardLegality)
{
  StubHorizonObstacleFootprintCritic critic;
  critic.setCollisionInterval(0.295, 0.305);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(make_linear_trajectory(28u)), 0.0);
  EXPECT_THROW(
    critic.scoreTrajectory(make_linear_trajectory(32u)),
    dwb_core::IllegalTrajectoryException);
  EXPECT_THROW(
    critic.scoreTrajectory(make_linear_trajectory(36u)),
    dwb_core::IllegalTrajectoryException);
}

TEST(HorizonObstacleFootprintCritic, RejectsCollisionBetweenGeneratedPoses)
{
  StubHorizonObstacleFootprintCritic critic;
  critic.setMaximumSweptDistance(0.10);
  critic.setCollisionInterval(0.49, 0.51);
  auto trajectory = make_trajectory({0.0, 1.0});
  EXPECT_THROW(
    critic.scoreTrajectory(trajectory),
    dwb_core::IllegalTrajectoryException);
}

TEST(HorizonObstacleFootprintCritic, RejectsNonFinitePose)
{
  StubHorizonObstacleFootprintCritic critic;
  auto trajectory = make_trajectory({2.0, 3.0});
  trajectory.poses.back().x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    critic.scoreTrajectory(trajectory),
    dwb_core::IllegalTrajectoryException);
}

TEST(HorizonObstacleFootprintCritic, RejectsCollisionAtGeneratedPose)
{
  StubHorizonObstacleFootprintCritic critic;
  try {
    static_cast<void>(
      critic.scoreTrajectory(make_trajectory({2.0, -1.0, 100.0})));
    FAIL() << "Expected a collision rejection";
  } catch (const dwb_core::IllegalTrajectoryException & exception) {
    const std::string detail = exception.what();
    EXPECT_NE(detail.find("pose_index=1"), std::string::npos);
    EXPECT_NE(detail.find("pose_x=-1"), std::string::npos);
  }
}

TEST(HorizonObstacleFootprintCritic, ConciseFailureRetainsRecoveryIndices)
{
  StubHorizonObstacleFootprintCritic critic;
  critic.setDetailedFailureDiagnostics(false);
  try {
    static_cast<void>(
      critic.scoreTrajectory(make_trajectory({2.0, -1.0, 100.0})));
    FAIL() << "Expected a collision rejection";
  } catch (const dwb_core::IllegalTrajectoryException & exception) {
    const std::string detail = exception.what();
    EXPECT_NE(detail.find("pose_index=1"), std::string::npos);
    EXPECT_NE(detail.find("subdivision=0"), std::string::npos);
    EXPECT_EQ(detail.find("pose_x="), std::string::npos);
  }
}
