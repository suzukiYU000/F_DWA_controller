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
#include <random>
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

std::vector<geometry_msgs::msg::Point> whill_footprint()
{
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -0.2;
  footprint[0].y = -0.3;
  footprint[1].x = 0.8;
  footprint[1].y = -0.3;
  footprint[2].x = 0.8;
  footprint[2].y = 0.3;
  footprint[3].x = -0.2;
  footprint[3].y = 0.3;
  return footprint;
}

std::vector<geometry_msgs::msg::Point> inset_footprint(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const double inset)
{
  std::vector<geometry_msgs::msg::Point> result = footprint;
  for (geometry_msgs::msg::Point & point : result) {
    point.x = std::copysign(std::abs(point.x) - inset, point.x);
    point.y = std::copysign(std::abs(point.y) - inset, point.y);
  }
  return result;
}

void expect_same_certification(
  const CertificationResult & reference,
  const CertificationResult & broadphase)
{
  EXPECT_EQ(broadphase.safe, reference.safe);
  EXPECT_EQ(broadphase.failure, reference.failure);
  EXPECT_EQ(broadphase.checked_pose_count, reference.checked_pose_count);
  EXPECT_EQ(broadphase.has_failure_pose, reference.has_failure_pose);
  EXPECT_EQ(
    broadphase.failure_source_pose_index,
    reference.failure_source_pose_index);
  EXPECT_EQ(
    broadphase.failure_interpolation_index,
    reference.failure_interpolation_index);
  EXPECT_EQ(broadphase.has_failure_cell, reference.has_failure_cell);
  if (reference.has_failure_cell) {
    EXPECT_EQ(broadphase.failure_cell_x, reference.failure_cell_x);
    EXPECT_EQ(broadphase.failure_cell_y, reference.failure_cell_y);
    EXPECT_EQ(broadphase.failure_cell_cost, reference.failure_cell_cost);
    EXPECT_DOUBLE_EQ(
      broadphase.failure_cell_world_x, reference.failure_cell_world_x);
    EXPECT_DOUBLE_EQ(
      broadphase.failure_cell_world_y, reference.failure_cell_world_y);
  }
  if (reference.has_failure_pose) {
    EXPECT_DOUBLE_EQ(
      broadphase.failure_pose.x, reference.failure_pose.x);
    EXPECT_DOUBLE_EQ(
      broadphase.failure_pose.y, reference.failure_pose.y);
    EXPECT_DOUBLE_EQ(
      broadphase.failure_pose.theta, reference.failure_pose.theta);
  }
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

  const CertificationResult reference =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult broadphase =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025, &workspace);

  expect_same_certification(reference, broadphase);
  EXPECT_FALSE(broadphase.safe);
  EXPECT_EQ(
    broadphase.failure, CertificationFailure::kLethalObstacle);
  EXPECT_TRUE(broadphase.has_failure_pose);
  EXPECT_TRUE(broadphase.has_failure_cell);
  EXPECT_EQ(broadphase.failure_cell_x, obstacle_x);
  EXPECT_EQ(broadphase.failure_cell_y, obstacle_y);
  EXPECT_EQ(
    broadphase.failure_cell_cost, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(broadphase.failure_source_pose_index, 0u);
}

TEST(TrajectoryCertifier, RejectsObstacleCrossingSweptBoundary)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  unsigned int obstacle_x = 0;
  unsigned int obstacle_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.75, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  std::vector<geometry_msgs::msg::Pose2D> poses(2);
  poses[1].x = 1.0;

  const CertificationResult reference =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult broadphase =
    certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025, &workspace);

  expect_same_certification(reference, broadphase);
  EXPECT_FALSE(broadphase.safe);
  EXPECT_EQ(
    broadphase.failure, CertificationFailure::kLethalObstacle);
  EXPECT_TRUE(broadphase.has_failure_pose);
  EXPECT_TRUE(broadphase.has_failure_cell);
  EXPECT_EQ(broadphase.failure_cell_x, obstacle_x);
  EXPECT_EQ(broadphase.failure_cell_y, obstacle_y);
  EXPECT_EQ(broadphase.failure_source_pose_index, 1u);
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
  EXPECT_TRUE(result.has_failure_cell);
  EXPECT_EQ(result.failure_cell_x, unknown_x);
  EXPECT_EQ(result.failure_cell_y, unknown_y);
  EXPECT_EQ(result.failure_cell_cost, nav2_costmap_2d::NO_INFORMATION);

  poses.front().x = 0.9;
  result = certify_pose_sequence(
    costmap, rectangle_footprint(), poses, 0.025);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.failure, CertificationFailure::kOffCostmap);
}

TEST(TrajectoryCertifier, ReserveRecoveryClearsAdditionalMargin)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.36, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::vector<geometry_msgs::msg::Point> certified_footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Point> planning_footprint =
    certified_footprint;
  for (geometry_msgs::msg::Point & point : planning_footprint) {
    point.y = std::copysign(0.30, point.y);
  }
  for (geometry_msgs::msg::Point & point : certified_footprint) {
    point.y = std::copysign(0.40, point.y);
  }

  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].y = -0.05;
  poses[2].y = -0.10;
  EXPECT_TRUE(
    certify_reserve_recovery_sequence(
      costmap, certified_footprint, planning_footprint, poses, 0.025,
      true));

  poses[1].y = 0.05;
  EXPECT_TRUE(
    certify_reserve_recovery_sequence(
      costmap, certified_footprint, planning_footprint, poses, 0.025,
      true));

  // A hard-safe sequence that never clears the additional reserve remains
  // inadmissible, even though recovery motion need not be monotonic.
  poses[1].y = -0.01;
  poses[2].y = -0.02;
  EXPECT_FALSE(
    certify_reserve_recovery_sequence(
      costmap, certified_footprint, planning_footprint, poses, 0.025,
      true));
}

TEST(TrajectoryCertifier, ReserveRecoveryRejectsPlanningFootprintCollision)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.26, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].y = -0.05;
  poses[2].y = -0.10;
  EXPECT_FALSE(
    certify_reserve_recovery_sequence(
      costmap, rectangle_footprint(), rectangle_footprint(), poses,
      0.025, true));
}

TEST(TrajectoryCertifier, ReserveRecoveryAllowsHardSafeNonmonotonicDetour)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.36, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::vector<geometry_msgs::msg::Point> certified_footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Point> planning_footprint =
    certified_footprint;
  for (geometry_msgs::msg::Point & point : planning_footprint) {
    point.y = std::copysign(0.30, point.y);
  }
  for (geometry_msgs::msg::Point & point : certified_footprint) {
    point.y = std::copysign(0.40, point.y);
  }

  // The intermediate pose moves away from the eventual clear pose. This is
  // allowed because the complete swept planning footprint remains hard-safe
  // and the terminal suffix clears the additional reserve.
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = -0.20;
  poses[2].x = 0.70;
  EXPECT_TRUE(
    certify_reserve_recovery_sequence(
      costmap, certified_footprint, planning_footprint, poses, 0.025,
      true));
}

TEST(TrajectoryCertifier, InitialOverlapRecoveryClearsPhysicalFootprint)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.28, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Point> inset_core = physical_footprint;
  for (geometry_msgs::msg::Point & point : inset_core) {
    point.x = std::copysign(std::abs(point.x) - 0.05, point.x);
    point.y = std::copysign(std::abs(point.y) - 0.05, point.y);
  }
  std::vector<geometry_msgs::msg::Pose2D> poses(4);
  poses[1].y = -0.02;
  poses[2].y = -0.08;
  poses[3].y = -0.12;

  EXPECT_TRUE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 2u));
}

TEST(TrajectoryCertifier, InitialOverlapRecoveryIsBoundedAndRejectsReentry)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.28, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Point> inset_core = physical_footprint;
  for (geometry_msgs::msg::Point & point : inset_core) {
    point.x = std::copysign(std::abs(point.x) - 0.05, point.x);
    point.y = std::copysign(std::abs(point.y) - 0.05, point.y);
  }
  std::vector<geometry_msgs::msg::Pose2D> poses(4);
  poses[1].y = -0.005;
  poses[2].y = -0.08;
  poses[3].y = -0.12;

  EXPECT_FALSE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 1u));
  EXPECT_TRUE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 2u));

  poses[3].y = 0.0;
  EXPECT_FALSE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 2u));
}

TEST(TrajectoryCertifier, InitialOverlapRecoveryRejectsCoreCollision)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.24, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Point> inset_core = physical_footprint;
  for (geometry_msgs::msg::Point & point : inset_core) {
    point.x = std::copysign(std::abs(point.x) - 0.05, point.x);
    point.y = std::copysign(std::abs(point.y) - 0.05, point.y);
  }
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].y = -0.08;
  poses[2].y = -0.12;

  EXPECT_FALSE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 1u));
}

TEST(TrajectoryCertifier, InitialOverlapRecoveryAllowsRearClearanceEscape)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = 0.08;
  poses[2].x = 0.16;

  EXPECT_TRUE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 1u));
}

TEST(TrajectoryCertifier, InitialOverlapRecoveryRejectsRearRotationIntoWall)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].theta = 0.30;
  poses[2].theta = 0.60;

  EXPECT_FALSE(
    certify_initial_overlap_recovery_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005, 1u));
}

TEST(TrajectoryCertifier, InitialOverlapMarginAllowsRearParallelStop)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  const std::vector<geometry_msgs::msg::Pose2D> poses(3);
  double overlap_fraction = 0.0;

  EXPECT_TRUE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005,
      &overlap_fraction));
  EXPECT_DOUBLE_EQ(overlap_fraction, 1.0);
}

TEST(TrajectoryCertifier, InitialOverlapMarginPrefersRearClearanceEscape)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  const std::vector<geometry_msgs::msg::Pose2D> stationary_poses(3);
  std::vector<geometry_msgs::msg::Pose2D> escape_poses(3);
  escape_poses[1].x = 0.08;
  escape_poses[2].x = 0.16;
  double stationary_overlap = 0.0;
  double escape_overlap = 0.0;

  ASSERT_TRUE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, stationary_poses, 0.005,
      &stationary_overlap));
  ASSERT_TRUE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, escape_poses, 0.005,
      &escape_overlap));
  EXPECT_LT(escape_overlap, stationary_overlap);
}

TEST(TrajectoryCertifier, InitialOverlapMarginRejectsNewBoundaryEntry)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.82, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[2].x = 0.04;

  EXPECT_FALSE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005));
}

TEST(TrajectoryCertifier, CommittedPrefixMayEnterSafeBoundaryStrip)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.82, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[2].x = 0.04;

  EXPECT_TRUE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005,
      nullptr, nullptr, true));
}

TEST(TrajectoryCertifier, InitialOverlapMarginAllowsFirstResponseEntry)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.82, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = 0.04;
  poses[2].x = -0.04;

  EXPECT_TRUE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005));
}

TEST(TrajectoryCertifier, InitialOverlapMarginRejectsBoundaryReentry)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = 0.16;
  poses[2].x = 0.0;

  EXPECT_FALSE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005));
}

TEST(TrajectoryCertifier, InitialOverlapMarginRejectsRearRotationIntoWall)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].theta = 0.30;
  poses[2].theta = 0.60;

  EXPECT_FALSE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005));
}

TEST(TrajectoryCertifier, InitialOverlapMarginRejectsUnknownOuterStrip)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.01, -2.0, -2.0);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(-0.18, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::NO_INFORMATION);

  const std::vector<geometry_msgs::msg::Point> physical_footprint =
    whill_footprint();
  const std::vector<geometry_msgs::msg::Point> inset_core =
    inset_footprint(physical_footprint, 0.05);
  const std::vector<geometry_msgs::msg::Pose2D> poses(3);

  EXPECT_FALSE(
    certify_initial_overlap_margin_sequence(
      costmap, physical_footprint, inset_core, poses, 0.005));
}

TEST(TrajectoryCertifier, ReusedWorkspacePreservesCertificationResult)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = 0.15;
  poses[1].theta = 0.12;
  poses[2].x = 0.35;
  poses[2].theta = 0.25;
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  const CertificationResult expected =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  CertificationWorkspace workspace;

  const CertificationResult first =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);
  const CertificationResult second =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  EXPECT_EQ(first.safe, expected.safe);
  EXPECT_EQ(first.failure, expected.failure);
  EXPECT_EQ(first.checked_pose_count, expected.checked_pose_count);
  EXPECT_EQ(second.safe, expected.safe);
  EXPECT_EQ(second.failure, expected.failure);
  EXPECT_EQ(second.checked_pose_count, expected.checked_pose_count);
}

TEST(TrajectoryCertifier, WholeSequenceBroadphasePreservesDensifiedResult)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Pose2D> poses(3);
  poses[1].x = 0.15;
  poses[1].y = 0.05;
  poses[1].theta = 0.20;
  poses[2].x = 0.35;
  poses[2].y = 0.12;
  poses[2].theta = 0.45;

  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  expect_same_certification(reference, broadphase);
  EXPECT_TRUE(broadphase.safe);
  EXPECT_GT(broadphase.checked_pose_count, poses.size());
}

TEST(TrajectoryCertifier, CircleBoundOffMapFallsBackToExactRectangle)
{
  nav2_costmap_2d::Costmap2D costmap(80, 16, 0.05, -2.0, -0.4);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  const std::vector<geometry_msgs::msg::Pose2D> poses(1);

  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  expect_same_certification(reference, broadphase);
  EXPECT_TRUE(broadphase.safe);
}

TEST(TrajectoryCertifier, SequenceBoundHazardOutsideSweepUsesExactFallback)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Pose2D> poses(2);
  poses[1].x = 0.2;
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.1, 0.55, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  expect_same_certification(reference, broadphase);
  EXPECT_TRUE(broadphase.safe);
}

TEST(TrajectoryCertifier, ExactBroadphasePreservesFallbackAndInflation)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  std::vector<geometry_msgs::msg::Pose2D> poses(1);
  poses.front().theta = 0.25 * std::acos(-1.0);
  unsigned int map_x = 0;
  unsigned int map_y = 0;

  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, map_x, map_y));
  costmap.setCost(
    map_x, map_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult inflation_reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  const CertificationResult inflation_broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);
  expect_same_certification(
    inflation_reference, inflation_broadphase);
  EXPECT_TRUE(inflation_broadphase.safe);

  // This lethal cell is inside the rotated footprint's map AABB but outside
  // the polygon. The broadphase must fall back to the exact full fill rather
  // than rejecting from the AABB alone.
  ASSERT_TRUE(costmap.worldToMap(0.5, 0.5, map_x, map_y));
  costmap.setCost(
    map_x, map_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const CertificationResult fallback_reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  const CertificationResult fallback_broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);
  expect_same_certification(
    fallback_reference, fallback_broadphase);
  EXPECT_TRUE(fallback_broadphase.safe);
  ASSERT_FALSE(workspace.pose_check_cache.empty());
  const std::size_t cached_polygon_count =
    workspace.pose_check_cache.size();
  const CertificationResult repeated_broadphase =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);
  expect_same_certification(fallback_reference, repeated_broadphase);
  EXPECT_EQ(workspace.pose_check_cache.size(), cached_polygon_count);
}

TEST(TrajectoryCertifier, ExactBroadphaseMatchesRandomCostmaps)
{
  constexpr double kResolution = 0.05;
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  std::mt19937 generator(20260728u);
  std::uniform_int_distribution<unsigned int> cell_distribution(0u, 79u);
  std::uniform_int_distribution<int> cost_distribution(0, 3);
  std::uniform_real_distribution<double> position_distribution(-1.2, 1.2);
  std::uniform_real_distribution<double> angle_distribution(
    -std::acos(-1.0), std::acos(-1.0));

  for (int trial_index = 0; trial_index < 200; ++trial_index) {
    nav2_costmap_2d::Costmap2D costmap(
      80, 80, kResolution, -2.0, -2.0);
    for (int cell_index = 0; cell_index < 16; ++cell_index) {
      const int selection = cost_distribution(generator);
      unsigned char cost = nav2_costmap_2d::FREE_SPACE;
      if (selection == 1) {
        cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      } else if (selection == 2) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (selection == 3) {
        cost = nav2_costmap_2d::NO_INFORMATION;
      }
      costmap.setCost(
        cell_distribution(generator),
        cell_distribution(generator), cost);
    }
    std::vector<geometry_msgs::msg::Pose2D> poses(3);
    for (geometry_msgs::msg::Pose2D & pose : poses) {
      pose.x = position_distribution(generator);
      pose.y = position_distribution(generator);
      pose.theta = angle_distribution(generator);
    }

    const CertificationResult reference =
      certify_pose_sequence(costmap, footprint, poses, 0.025);
    CertificationWorkspace workspace;
    ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
    const CertificationResult broadphase =
      certify_pose_sequence(
      costmap, footprint, poses, 0.025, &workspace);

    SCOPED_TRACE(trial_index);
    expect_same_certification(reference, broadphase);
  }
}

TEST(TrajectoryCertifier, StaleBroadphaseFallsBackAfterOriginChange)
{
  nav2_costmap_2d::Costmap2D costmap(80, 80, 0.05, -2.0, -2.0);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  costmap.updateOrigin(-1.5, -1.5);
  std::vector<geometry_msgs::msg::Pose2D> poses(2);
  poses[1].x = 0.1;
  poses[1].theta = 0.1;
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();

  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  const CertificationResult stale_workspace =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  expect_same_certification(reference, stale_workspace);
}

TEST(TrajectoryCertifier, ReusedBroadphaseAllocationMatchesCostmapUpdates)
{
  nav2_costmap_2d::Costmap2D costmap(80, 80, 0.05, -2.0, -2.0);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  const std::vector<geometry_msgs::msg::Pose2D> poses(1);
  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  const std::size_t * const allocation =
    workspace.hazard_prefix_sum.data();

  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  EXPECT_EQ(workspace.hazard_prefix_sum.data(), allocation);
  expect_same_certification(
    certify_pose_sequence(costmap, footprint, poses, 0.025),
    certify_pose_sequence(
      costmap, footprint, poses, 0.025, &workspace));

  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::FREE_SPACE);
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  EXPECT_EQ(workspace.hazard_prefix_sum.data(), allocation);
  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);
  const CertificationResult reused =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);
  expect_same_certification(reference, reused);
  EXPECT_TRUE(reused.safe);
}

TEST(TrajectoryCertifier, MalformedBroadphaseFallsBackToExactCheck)
{
  nav2_costmap_2d::Costmap2D costmap(80, 80, 0.05, -2.0, -2.0);
  unsigned int obstacle_x = 0;
  unsigned int obstacle_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  const std::vector<geometry_msgs::msg::Point> footprint =
    rectangle_footprint();
  const std::vector<geometry_msgs::msg::Pose2D> poses(1);
  const CertificationResult reference =
    certify_pose_sequence(costmap, footprint, poses, 0.025);

  CertificationWorkspace workspace;
  ASSERT_TRUE(prepare_certification_broadphase(costmap, workspace));
  ASSERT_FALSE(workspace.hazard_prefix_sum.empty());
  workspace.hazard_prefix_sum.pop_back();
  const CertificationResult malformed_workspace =
    certify_pose_sequence(
    costmap, footprint, poses, 0.025, &workspace);

  expect_same_certification(reference, malformed_workspace);
  EXPECT_FALSE(malformed_workspace.safe);
  EXPECT_EQ(
    malformed_workspace.failure,
    CertificationFailure::kLethalObstacle);
}

}  // namespace f_dwa_controller
