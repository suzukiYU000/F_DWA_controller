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
