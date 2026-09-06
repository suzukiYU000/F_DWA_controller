/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

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

namespace
{
class RasterCacheCritic : public f_dwa_controller::HorizonObstacleFootprintCritic
{
public:
  void setSnapshot(
    nav2_costmap_2d::Costmap2D & map,
    const std::vector<geometry_msgs::msg::Point> & footprint, bool cached = true)
  {
    costmap_ = &map;
    footprint_spec_ = footprint;
    inset_core_footprint_ = footprint;
    nav2_costmap_2d::padFootprint(inset_core_footprint_, -0.05);
    name_ = "ObstacleFootprint";
    footprint_radius_ = 0.0;
    for (const auto & point : footprint) {
      footprint_radius_ = std::max(footprint_radius_, std::hypot(point.x, point.y));
    }
    raster_footprint_cache_.clear();
    cell_diagnostic_cache_.clear();
    raster_footprint_cache_ready_ = cached;
  }

  void attach(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map)
  {
    costmap_ros_ = map;
    costmap_ = map->getCostmap();
  }

  void useMarginRecovery()
  {
    enable_initial_overlap_recovery_ = true;
    enable_transient_boundary_margin_recovery_ = true;
  }

  std::size_t cacheSize() const {return raster_footprint_cache_.size();}
  std::size_t failureCacheSize() const {return cell_diagnostic_cache_.size();}
  void disableCaches() {raster_footprint_cache_ready_ = false;}
};

std::vector<geometry_msgs::msg::Point> test_footprint()
{
  std::vector<geometry_msgs::msg::Point> result(4);
  result[0].x = -0.2; result[0].y = -0.3;
  result[1].x = 0.8; result[1].y = -0.3;
  result[2].x = 0.8; result[2].y = 0.3;
  result[3].x = -0.2; result[3].y = 0.3;
  return result;
}

struct ScoredResult
{
  double score{0.0};
  std::string failure;
};

template<class Callable>
ScoredResult score_or_failure(Callable call)
{
  try {
    return {call(), {}};
  } catch (const dwb_core::IllegalTrajectoryException & exception) {
    return {0.0, exception.what()};
  }
}

void expect_equal(const ScoredResult & actual, const ScoredResult & expected)
{
  EXPECT_DOUBLE_EQ(actual.score, expected.score);
  EXPECT_EQ(actual.failure, expected.failure);
}
}  // namespace

TEST(HorizonObstacleFootprintCritic, RasterCacheMatchesUpstreamAtSubcellBoundaries)
{
  std::mt19937 random(6821u);
  const auto footprint = test_footprint();
  for (int map_index = 0; map_index < 6; ++map_index) {
    nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
    for (unsigned int y = 0; y < 100; ++y) {
      for (unsigned int x = 0; x < 100; ++x) {
        const unsigned int sample = random() % 300;
        map.setCost(x, y, map_index < 2 ? sample % 254 :
          (sample < 254 ? sample : (sample % 2 ? 254 : 255)));
      }
    }
    RasterCacheCritic critic;
    critic.setSnapshot(map, footprint);
    for (int index = 0; index < 2000; ++index) {
      geometry_msgs::msg::Pose2D pose;
      // Include off-map poses, opposite rotation, exact cell borders and
      // their immediately adjacent representable coordinates.
      pose.x = 0.05 * (static_cast<int>(random() % 120) - 60);
      pose.y = 0.05 * (static_cast<int>(random() % 120) - 60);
      pose.theta = (static_cast<int>(random() % 64) - 32) * M_PI / 16.0;
      const double boundary_x = pose.x;
      for (int repetition = 0; repetition < 3; ++repetition) {
        pose.x = repetition == 1 ? boundary_x :
          std::nextafter(boundary_x, repetition == 0 ? -INFINITY : INFINITY);
        const auto expected = score_or_failure([&] {
              return critic.dwb_critics::ObstacleFootprintCritic::scorePose(pose);
          });
        expect_equal(score_or_failure([&] {return critic.scorePose(pose);}), expected);
      }
    }
    EXPECT_GT(critic.cacheSize(), 0u);
    EXPECT_LE(critic.cacheSize(), 4096u);
  }
}

TEST(HorizonObstacleFootprintCritic, RasterCacheReusesOnlyIdenticalRasterEdges)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  RasterCacheCritic critic;
  critic.setSnapshot(map, test_footprint());
  geometry_msgs::msg::Pose2D pose;
  pose.x = 1.011;
  pose.y = 1.011;
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
  const auto first_size = critic.cacheSize();
  pose.x += 0.001;
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
  EXPECT_EQ(critic.cacheSize(), first_size);
  pose.x += 0.05;
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
  EXPECT_GT(critic.cacheSize(), first_size);
}

TEST(HorizonObstacleFootprintCritic, RasterCachePreservesContinuousRecoveryAndDiagnostics)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
  for (unsigned int x = 0; x < 100; ++x) {
    map.setCost(x, 56u, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  map.setCost(49u, 44u, nav2_costmap_2d::NO_INFORMATION);
  RasterCacheCritic optimized, reference;
  optimized.setSnapshot(map, test_footprint());
  reference.setSnapshot(map, test_footprint(), false);
  optimized.useMarginRecovery();
  reference.useMarginRecovery();
  for (const bool details : {true, false}) {
    optimized.setDetailedFailureDiagnostics(details);
    reference.setDetailedFailureDiagnostics(details);
    for (int turn = -20; turn <= 20; ++turn) {
      dwb_msgs::msg::Trajectory2D trajectory;
      for (int index = 0; index <= 30; ++index) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = 0.01 * index;
        pose.y = 0.001 * turn * index;
        pose.theta = 0.002 * turn * index;
        trajectory.poses.push_back(pose);
      }
      const auto expected = score_or_failure([&] {return reference.scoreTrajectory(trajectory);});
      expect_equal(score_or_failure([&] {return optimized.scoreTrajectory(trajectory);}), expected);
      expect_equal(score_or_failure([&] {return optimized.scoreTrajectory(trajectory);}), expected);
    }
  }
}

TEST(HorizonObstacleFootprintCritic, PrepareInvalidatesCachedLegalAndIllegalCells)
{
  const bool initialize = !rclcpp::ok();
  if (initialize) {rclcpp::init(0, nullptr);}
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}}});
    auto map_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(map_ros->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    map_ros->setRobotFootprint(test_footprint());
    auto & map = *map_ros->getCostmap();
    std::fill(map.getCharMap(), map.getCharMap() + map.getSizeInCellsX() * map.getSizeInCellsY(),
      0);
    RasterCacheCritic critic;
    critic.attach(map_ros);
    geometry_msgs::msg::Pose2D pose;
    pose.x = 1.0; pose.y = 1.0;
    const nav_2d_msgs::msg::Twist2D velocity;
    const nav_2d_msgs::msg::Path2D plan;
    ASSERT_TRUE(critic.prepare(pose, velocity, pose, plan));
    EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
    EXPECT_GT(critic.cacheSize(), 0u);
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(1.81, 1.0, x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    ASSERT_TRUE(critic.prepare(pose, velocity, pose, plan));
    EXPECT_EQ(critic.cacheSize(), 0u);
    EXPECT_THROW(critic.scorePose(pose), dwb_core::IllegalTrajectoryException);
    dwb_msgs::msg::Trajectory2D trajectory;
    trajectory.poses.push_back(pose);
    EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
    EXPECT_GT(critic.failureCacheSize(), 0u);
    RasterCacheCritic reference;
    reference.attach(map_ros);
    ASSERT_TRUE(reference.prepare(pose, velocity, pose, plan));
    reference.disableCaches();
    for (const bool details : {false, true}) {
      critic.setDetailedFailureDiagnostics(details);
      reference.setDetailedFailureDiagnostics(details);
      for (int index = 0; index < 50; ++index) {
        trajectory.poses[0].theta = 0.001 * index;
        const auto expected = score_or_failure([&] {return reference.scoreTrajectory(trajectory);});
        expect_equal(score_or_failure([&] {return critic.scoreTrajectory(trajectory);}), expected);
      }
    }
    map.setCost(x, y, 0);
    ASSERT_TRUE(critic.prepare(pose, velocity, pose, plan));
    EXPECT_EQ(critic.failureCacheSize(), 0u);
    EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
    map_ros->on_cleanup(rclcpp_lifecycle::State());
  }
  if (initialize) {rclcpp::shutdown();}
}

TEST(HorizonObstacleFootprintCritic, DiagnosticsPreserveExactPoseAndDiagnosticMode)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(0.811, 0.011, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  RasterCacheCritic optimized, reference;
  optimized.setSnapshot(map, test_footprint());
  reference.setSnapshot(map, test_footprint(), false);
  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D pose;
  pose.x = 0.011; pose.y = 0.011;
  trajectory.poses.push_back(pose);
  for (const bool details : {true, false}) {
    optimized.setDetailedFailureDiagnostics(details);
    reference.setDetailedFailureDiagnostics(details);
    for (const double theta : {0.0, -0.0, 1.0e-10}) {
      trajectory.poses[0].theta = theta;
      const auto expected = score_or_failure([&] {return reference.scoreTrajectory(trajectory);});
      ASSERT_FALSE(expected.failure.empty());
      expect_equal(score_or_failure([&] {return optimized.scoreTrajectory(trajectory);}), expected);
      expect_equal(score_or_failure([&] {return optimized.scoreTrajectory(trajectory);}), expected);
    }
  }
}

TEST(HorizonObstacleFootprintCritic, CacheCapacityAndLargePolygonFallBackWithoutChangingScores)
{
  nav2_costmap_2d::Costmap2D map(400, 200, 0.05, 0.0, 0.0, 37);
  RasterCacheCritic critic;
  critic.setSnapshot(map, test_footprint());
  geometry_msgs::msg::Pose2D pose;
  for (int index = 0; index < 5000; ++index) {
    pose.x = 1.01 + 0.06 * (index % 200);
    pose.y = 1.01 + 0.06 * (index / 200);
    EXPECT_DOUBLE_EQ(critic.scorePose(pose), 37.0);
  }
  EXPECT_EQ(critic.cacheSize(), 4096u);
  std::vector<geometry_msgs::msg::Point> polygon(20);
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const double angle = 2.0 * M_PI * index / polygon.size();
    polygon[index].x = 0.5 * std::cos(angle);
    polygon[index].y = 0.5 * std::sin(angle);
  }
  critic.setSnapshot(map, polygon);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 37.0);
  EXPECT_EQ(critic.cacheSize(), 0u);
}

TEST(HorizonObstacleFootprintCritic, SweptProofCannotOmitRearUnknownOrOffMapHazards)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
  f_dwa_controller::CertificationWorkspace workspace;
  geometry_msgs::msg::Pose2D first, second;
  second.x = 0.5;
  second.theta = M_PI_2;
  const double radius = std::hypot(0.8, 0.3);
  ASSERT_TRUE(f_dwa_controller::prepare_certification_broadphase(map, workspace));
  EXPECT_TRUE(f_dwa_controller::certification_swept_segment_bounds_are_hazard_free(
      map, first, second, radius, workspace));
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(-0.1, -0.1, x, y));
  for (const unsigned char cost : {nav2_costmap_2d::LETHAL_OBSTACLE,
      nav2_costmap_2d::NO_INFORMATION})
  {
    map.setCost(x, y, cost);
    ASSERT_TRUE(f_dwa_controller::prepare_certification_broadphase(map, workspace));
    EXPECT_FALSE(f_dwa_controller::certification_swept_segment_bounds_are_hazard_free(
        map, first, second, radius, workspace));
  }
  map.setCost(x, y, 0);
  ASSERT_TRUE(f_dwa_controller::prepare_certification_broadphase(map, workspace));
  second.x = 2.4;
  EXPECT_FALSE(f_dwa_controller::certification_swept_segment_bounds_are_hazard_free(
      map, first, second, radius, workspace));
  second.x = 0.5;
  map.updateOrigin(-2.0, -2.0);
  EXPECT_FALSE(f_dwa_controller::certification_swept_segment_bounds_are_hazard_free(
      map, first, second, radius, workspace));
}

TEST(HorizonObstacleFootprintCritic, SweptProofMatchesOriginalScoringOnRandomTrajectories)
{
  std::mt19937 random(2913u);
  for (int scene = 0; scene < 6; ++scene) {
    nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
    for (int index = 0; index < scene * 30; ++index) {
      map.setCost(random() % 100, random() % 100,
        index % 3 ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::NO_INFORMATION);
    }
    f_dwa_controller::CertificationWorkspace workspace;
    ASSERT_TRUE(f_dwa_controller::prepare_certification_broadphase(map, workspace));
    RasterCacheCritic optimized, reference;
    optimized.setSnapshot(map, test_footprint());
    reference.setSnapshot(map, test_footprint(), false);
    optimized.setSharedCertificationWorkspace(&workspace);
    for (int candidate = 0; candidate < 200; ++candidate) {
      dwb_msgs::msg::Trajectory2D trajectory;
      geometry_msgs::msg::Pose2D pose;
      pose.x = 0.05 * (static_cast<int>(random() % 60) - 30);
      pose.y = 0.05 * (static_cast<int>(random() % 60) - 30);
      pose.theta = (static_cast<int>(random() % 64) - 32) * M_PI / 16.0;
      const double turn = 0.01 * (static_cast<int>(random() % 31) - 15);
      for (int index = 0; index < 15; ++index) {
        trajectory.poses.push_back(pose);
        pose.x += 0.04 * std::cos(pose.theta);
        pose.y += 0.04 * std::sin(pose.theta);
        pose.theta += turn;
      }
      const auto expected = score_or_failure([&] {return reference.scoreTrajectory(trajectory);});
      expect_equal(score_or_failure([&] {return optimized.scoreTrajectory(trajectory);}), expected);
    }
  }
}
