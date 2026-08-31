// Copyright (c) 2026 suzukiYU000
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "f_dwa_controller/path_subgoal.hpp"

namespace
{

nav_2d_msgs::msg::Path2D env2_path()
{
  nav_2d_msgs::msg::Path2D path;
  geometry_msgs::msg::Pose2D pose;
  pose.x = 0.0;
  pose.y = 0.0;
  path.poses.push_back(pose);
  pose.y = 5.0;
  path.poses.push_back(pose);
  pose.x = 10.0;
  path.poses.push_back(pose);
  pose.y = 0.0;
  path.poses.push_back(pose);
  return path;
}

TEST(PathSubgoal, FollowsArclengthBeforeEnv2Corner)
{
  const auto path = env2_path();
  geometry_msgs::msg::Pose2D current_pose;
  current_pose.x = 0.2;
  current_pose.y = 2.0;
  geometry_msgs::msg::Pose2D subgoal;

  ASSERT_TRUE(f_dwa_controller::compute_path_subgoal(
      path, current_pose, 1.5, subgoal));
  EXPECT_NEAR(subgoal.x, 0.0, 1.0e-12);
  EXPECT_NEAR(subgoal.y, 3.5, 1.0e-12);
}

TEST(PathSubgoal, ContinuesAroundEnv2Corner)
{
  const auto path = env2_path();
  geometry_msgs::msg::Pose2D current_pose;
  current_pose.x = 0.0;
  current_pose.y = 4.5;
  geometry_msgs::msg::Pose2D subgoal;

  ASSERT_TRUE(f_dwa_controller::compute_path_subgoal(
      path, current_pose, 1.5, subgoal));
  EXPECT_NEAR(subgoal.x, 1.0, 1.0e-12);
  EXPECT_NEAR(subgoal.y, 5.0, 1.0e-12);
}

TEST(PathSubgoal, UsesPathEndWhenRemainingDistanceIsShort)
{
  const auto path = env2_path();
  geometry_msgs::msg::Pose2D current_pose;
  current_pose.x = 10.0;
  current_pose.y = 0.8;
  geometry_msgs::msg::Pose2D subgoal;

  ASSERT_TRUE(f_dwa_controller::compute_path_subgoal(
      path, current_pose, 1.5, subgoal));
  EXPECT_NEAR(subgoal.x, 10.0, 1.0e-12);
  EXPECT_NEAR(subgoal.y, 0.0, 1.0e-12);
}

TEST(PathSubgoal, ProjectsCandidateProgressAroundCornerByArclength)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(3u);
  path.poses[1u].x = 1.0;
  path.poses[2u].x = 1.0;
  path.poses[2u].y = 1.0;
  geometry_msgs::msg::Pose2D pose;
  pose.x = 1.0;
  pose.y = 0.4;
  f_dwa_controller::PathProjection projection;

  ASSERT_TRUE(f_dwa_controller::project_pose_onto_path(
      path, pose, projection));
  EXPECT_NEAR(projection.arclength, 1.4, 1.0e-12);
  EXPECT_NEAR(projection.lateral_error, 0.0, 1.0e-12);
  EXPECT_NEAR(projection.tangent_heading, M_PI_2, 1.0e-12);
}

TEST(PathSubgoal, MeaningfulProgressUsesPathArclengthAndLookaheadHeading)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(3u);
  path.poses[1u].x = 1.0;
  path.poses[2u].x = 1.0;
  path.poses[2u].y = 1.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.theta = M_PI_2;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(3u);
  trajectory.poses[0u].x = 0.9;
  trajectory.poses[1u].x = 1.0;
  trajectory.poses[1u].y = 0.1;
  trajectory.poses[2u].x = 1.0;
  trajectory.poses[2u].y = 0.3;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.20, 0.20));

  trajectory.poses[1u].x = 1.1;
  trajectory.poses[1u].y = 0.0;
  trajectory.poses[2u].x = 1.3;
  trajectory.poses[2u].y = 0.0;
  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.20, 0.20));

  trajectory.poses[2u].theta = 0.25;
  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.20, 0.20));
  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.0, 0.20));
}

TEST(PathSubgoal, TranslationAndHeadingEvidenceCanBeClassifiedSeparately)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 1.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 1.0;
  heading_target.theta = 0.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses[0u].theta = 0.3;
  trajectory.poses[1u].theta = 0.1;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.1, 0.1));
  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.1, 0.0));
  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.0, 0.1));
}

TEST(PathSubgoal, ArclengthProgressRejectsGrowingPathDistance)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 2.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(3u);
  trajectory.poses[0u].x = 0.5;
  trajectory.poses[0u].y = 0.7;
  trajectory.poses[1u].x = 0.55;
  trajectory.poses[1u].y = 0.775;
  trajectory.poses[2u].x = 1.0;
  trajectory.poses[2u].y = 2.2;

  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, EarlyProgressCannotHideTerminalDeparture)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 2.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(3u);
  trajectory.poses[0u].x = 0.5;
  trajectory.poses[1u].x = 0.55;
  trajectory.poses[2u].x = 1.0;
  trajectory.poses[2u].y = 1.0;

  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, ArclengthProgressAllowsBoundedPathDistanceGrowth)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 2.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses[0u].x = 0.5;
  trajectory.poses[0u].y = 0.7;
  trajectory.poses[1u].x = 0.525;
  trajectory.poses[1u].y = 0.72;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, ArclengthProgressAllowsForwardFacingAvoidanceHeading)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 2.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses[0u].x = 0.5;
  trajectory.poses[1u].x = 0.55;
  trajectory.poses[1u].theta = 0.1;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));

  trajectory.poses[1u].theta = M_PI;
  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, ArclengthProgressAllowsUnknownObstacleDetour)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.x = 2.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses[0u].x = 0.50;
  trajectory.poses[0u].y = -0.15;
  trajectory.poses[0u].theta = 0.05;
  trajectory.poses[1u].x = 1.10;
  trajectory.poses[1u].y = -0.25;
  trajectory.poses[1u].theta = -0.35;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, InPlaceTurnCountsWhileTranslationProgressIsEnabled)
{
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 2.0;
  geometry_msgs::msg::Pose2D heading_target;
  heading_target.theta = 0.0;
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses[0u].theta = 0.40;
  trajectory.poses[1u].theta = 0.35;

  EXPECT_TRUE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));

  trajectory.poses[1u].theta = 0.45;
  EXPECT_FALSE(f_dwa_controller::trajectory_has_meaningful_path_progress(
      trajectory, path, heading_target, 0.025, 0.025));
}

TEST(PathSubgoal, RejectsInvalidInput)
{
  nav_2d_msgs::msg::Path2D empty_path;
  geometry_msgs::msg::Pose2D current_pose;
  geometry_msgs::msg::Pose2D subgoal;
  EXPECT_FALSE(f_dwa_controller::compute_path_subgoal(
      empty_path, current_pose, 1.5, subgoal));

  const auto path = env2_path();
  EXPECT_FALSE(f_dwa_controller::compute_path_subgoal(
      path, current_pose, -1.0, subgoal));
  EXPECT_FALSE(f_dwa_controller::compute_path_subgoal(
      path, current_pose, std::numeric_limits<double>::quiet_NaN(), subgoal));
}

TEST(PathSubgoal, ProgressCostDoesNotPenalizePassingSubgoal)
{
  geometry_msgs::msg::Pose2D subgoal;
  subgoal.x = 1.5;
  subgoal.theta = 0.0;
  geometry_msgs::msg::Pose2D terminal_pose;

  terminal_pose.x = 0.5;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_progress_cost(terminal_pose, subgoal),
    1.0, 1.0e-12);
  terminal_pose.x = 1.5;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_progress_cost(terminal_pose, subgoal),
    0.0, 1.0e-12);
  terminal_pose.x = 2.0;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_progress_cost(terminal_pose, subgoal),
    0.0, 1.0e-12);
}

TEST(PathSubgoal, ProgressCostUsesSubgoalPathTangent)
{
  geometry_msgs::msg::Pose2D subgoal;
  subgoal.x = 2.0;
  subgoal.y = 5.0;
  subgoal.theta = 1.5707963267948966;
  geometry_msgs::msg::Pose2D terminal_pose;
  terminal_pose.x = -3.0;
  terminal_pose.y = 4.25;

  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_progress_cost(terminal_pose, subgoal),
    0.75, 1.0e-12);
}

TEST(PathSubgoal, ForwardRayCostRetainsLateralPathErrorAfterPassingSubgoal)
{
  geometry_msgs::msg::Pose2D subgoal;
  subgoal.x = 1.5;
  subgoal.theta = 0.0;
  geometry_msgs::msg::Pose2D terminal_pose;

  terminal_pose.x = 0.5;
  terminal_pose.y = 0.4;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_forward_ray_cost(terminal_pose, subgoal),
    std::hypot(1.0, 0.4), 1.0e-12);

  terminal_pose.x = 2.0;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_forward_ray_cost(terminal_pose, subgoal),
    0.4, 1.0e-12);

  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_forward_ray_cost(
      terminal_pose, subgoal, 0.5),
    0.2, 1.0e-12);

  terminal_pose.y = 0.0;
  EXPECT_NEAR(
    f_dwa_controller::path_subgoal_forward_ray_cost(terminal_pose, subgoal),
    0.0, 1.0e-12);
}

}  // namespace
