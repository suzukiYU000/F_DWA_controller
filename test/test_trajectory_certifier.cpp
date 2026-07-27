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

#include <vector>

#include "f_dwa_controller/trajectory_certifier.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace f_dwa_controller
{

namespace
{

std::vector<geometry_msgs::msg::Point> rectangle_footprint()
{
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -0.5;
  footprint[0].y = -0.3;
  footprint[1].x = 0.5;
  footprint[1].y = -0.3;
  footprint[2].x = 0.5;
  footprint[2].y = 0.3;
  footprint[3].x = -0.5;
  footprint[3].y = 0.3;
  return footprint;
}

}  // namespace

TEST(TrajectoryCertifier, AcceptsFreeSpaceAndDensifiesSweep)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  std::vector<geometry_msgs::msg::Pose2D> poses(2);
  poses[1].x = 0.20;
  poses[1].theta = 0.20;

  const CertificationResult result =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);

  EXPECT_TRUE(result.safe);
  EXPECT_GT(result.checked_pose_count, poses.size());
}

TEST(TrajectoryCertifier, RejectsObstacleInsideFootprintInterior)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  unsigned int obstacle_x = 0;
  unsigned int obstacle_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  std::vector<geometry_msgs::msg::Pose2D> poses(1);

  const CertificationResult result =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);

  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.failure, CertificationFailure::kLethalObstacle);
}

TEST(TrajectoryCertifier, RejectsUnknownAndOffCostmap)
{
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.05, -1.0, -1.0);
  unsigned int unknown_x = 0;
  unsigned int unknown_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, unknown_x, unknown_y));
  costmap.setCost(
    unknown_x, unknown_y, nav2_costmap_2d::NO_INFORMATION);
  std::vector<geometry_msgs::msg::Pose2D> poses(1);

  CertificationResult result =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.failure, CertificationFailure::kUnknownSpace);

  poses.front().x = 0.9;
  result = certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.failure, CertificationFailure::kOffCostmap);
}

}  // namespace f_dwa_controller
