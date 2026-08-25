/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "builtin_interfaces/msg/duration.hpp"
#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/fixed_distance_risk_path.hpp"
#include "f_dwa_controller/footprint_clearance_critic.hpp"
#include "f_dwa_controller/forward_obstacle_critic.hpp"
#include "f_dwa_controller/mean_path_dist_critic.hpp"
#include "f_dwa_controller/mean_speed_critic.hpp"
#include "f_dwa_controller/path_subgoal_dist_critic.hpp"
#include "f_dwa_controller/terminal_approach_critic.hpp"
#include "f_dwa_controller/trajectory_progress_critic.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

nav_2d_msgs::msg::Path2D default_reference_plan()
{
  nav_2d_msgs::msg::Path2D plan;
  plan.poses.resize(2u);
  plan.poses.back().x = 10.0;
  return plan;
}

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
  StubForwardObstacleCritic()
  {
    risk_seed_time_ = 1.0;
    global_plan_ = default_reference_plan();
  }

  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return pose.y < -0.1 ? 1.0 : 0.0;
  }
};

class AnalyticForwardObstacleCritic
  : public f_dwa_controller::ForwardObstacleCritic
{
public:
  AnalyticForwardObstacleCritic()
  {
    risk_distance_ = 2.5;
    risk_seed_time_ = 1.4;
    sample_resolution_ = 0.05;
    distance_weighting_power_ = 1.0;
    peak_weight_ = 0.25;
    global_plan_ = default_reference_plan();
  }

  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return std::clamp(
      0.5 + 0.2 * std::sin(pose.x) + 0.2 * std::cos(pose.y),
      0.0, 1.0);
  }
};

class ReferenceBlockerForwardObstacleCritic
  : public f_dwa_controller::ForwardObstacleCritic
{
public:
  ReferenceBlockerForwardObstacleCritic()
  {
    risk_distance_ = 2.5;
    risk_seed_time_ = 1.4;
    sample_resolution_ = 0.05;
    distance_weighting_power_ = 0.0;
    peak_weight_ = 0.25;
    global_plan_ = default_reference_plan();
  }

  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return std::hypot(pose.x - 1.5, pose.y) <= 0.12 ? 1.0 : 0.0;
  }
};

class ManeuverPhaseBlockerForwardObstacleCritic
  : public f_dwa_controller::ForwardObstacleCritic
{
public:
  ManeuverPhaseBlockerForwardObstacleCritic()
  {
    risk_distance_ = 2.5;
    risk_seed_time_ = 1.4;
    sample_resolution_ = 0.05;
    distance_weighting_power_ = 0.0;
    peak_weight_ = 0.25;
    global_plan_ = default_reference_plan();
  }

  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return std::hypot(pose.x - 1.8, pose.y) <= 0.24 ? 1.0 : 0.0;
  }
};

class StubRangeForwardObstacleCritic
  : public f_dwa_controller::ForwardObstacleCritic
{
public:
  explicit StubRangeForwardObstacleCritic(const double blocked_x)
  : blocked_x_(blocked_x)
  {
    risk_distance_ = 1.2;
    risk_seed_time_ = 1.0;
    sample_resolution_ = 0.05;
    distance_weighting_power_ = 1.0;
    global_plan_ = default_reference_plan();
  }

  double normalizedCostAt(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    return pose.x >= blocked_x_ ? 1.0 : 0.0;
  }

private:
  double blocked_x_;
};

class CostmapForwardObstacleCritic
  : public f_dwa_controller::ForwardObstacleCritic
{
public:
  void configure(
    nav2_costmap_2d::Costmap2D * costmap,
    const double risk_distance,
    const double sample_resolution)
  {
    costmap_ = costmap;
    risk_distance_ = risk_distance;
    risk_seed_time_ = 1.0;
    sample_resolution_ = sample_resolution;
    distance_weighting_power_ = 1.0;
    global_plan_ = default_reference_plan();
  }
};

class StubPathSubgoalDistCritic
  : public f_dwa_controller::PathSubgoalDistCritic
{
public:
  void allowForwardOvershoot()
  {
    allow_forward_overshoot_ = true;
  }
};

class StubTerminalApproachCritic
  : public f_dwa_controller::TerminalApproachCritic
{
public:
  bool sampleEvaluationState(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    geometry_msgs::msg::Pose2D & pose,
    double & speed) const
  {
    const auto state = evaluationState(trajectory);
    if (!state) {
      return false;
    }
    pose = state->pose;
    speed = state->speed;
    return true;
  }

  void setParameters(
    const double evaluation_time,
    const double outer_distance,
    const double full_weight_distance,
    const double reference_deceleration)
  {
    evaluation_time_ = evaluation_time;
    outer_distance_ = outer_distance;
    full_weight_distance_ = full_weight_distance;
    reference_deceleration_ = reference_deceleration;
    validateParameters();
  }
};

class StubFootprintClearanceCritic
  : public f_dwa_controller::FootprintClearanceCritic
{
public:
  StubFootprintClearanceCritic()
  {
    clearance_bands_ = 5;
    // Keep the requested test sampling resolution below the production
    // half-margin safety bound.  These stubs verify distance integration,
    // independently of the separate sampling-gap clamp.
    clearance_margin_ = 1.0;
    risk_distance_ = 1.0;
    risk_seed_time_ = 1.0;
    sample_resolution_ = 0.5;
    peak_weight_ = 0.5;
    penalty_power_ = 1.0;
    expanded_footprints_.resize(5);
    global_plan_ = default_reference_plan();
    prepared_ = true;
  }

  void setFirstCollidingBand(const std::size_t band)
  {
    first_colliding_band_ = band;
  }

  void setMinimumCollisionX(const double value)
  {
    minimum_collision_x_ = value;
  }

  void setMaximumCollisionAbsY(const double value)
  {
    maximum_collision_abs_y_ = value;
  }

  void setRiskPath(const double distance, const int samples)
  {
    risk_distance_ = distance;
    sample_resolution_ = distance / static_cast<double>(samples);
  }

  void setRiskPathResolution(
    const double distance, const double resolution)
  {
    risk_distance_ = distance;
    sample_resolution_ = resolution;
    clearance_margin_ = std::max(clearance_margin_, 2.0 * resolution);
  }

  void setRiskSeedTime(const double value)
  {
    risk_seed_time_ = value;
  }

  void setFixedClearance(const double clearance)
  {
    fixed_clearance_ = clearance;
  }

  void setClearanceShape(const double margin, const double power)
  {
    clearance_margin_ = margin;
    penalty_power_ = power;
  }

  double scorePose(const geometry_msgs::msg::Pose2D & pose) const
  {
    return scorePoseClearance(pose);
  }

protected:
  double minimumFootprintClearance(
    const geometry_msgs::msg::Pose2D & pose) const override
  {
    if (std::isfinite(fixed_clearance_)) {
      return fixed_clearance_;
    }
    if (first_colliding_band_ >=
      static_cast<std::size_t>(clearance_bands_) ||
      pose.x < minimum_collision_x_ ||
      std::abs(pose.y) > maximum_collision_abs_y_)
    {
      return std::numeric_limits<double>::infinity();
    }
    const double band_width =
      clearance_margin_ / static_cast<double>(clearance_bands_);
    return (static_cast<double>(first_colliding_band_) + 0.5) * band_width;
  }

private:
  std::size_t first_colliding_band_{5};
  double minimum_collision_x_{
    -std::numeric_limits<double>::infinity()};
  double maximum_collision_abs_y_{
    std::numeric_limits<double>::infinity()};
  double fixed_clearance_{std::numeric_limits<double>::quiet_NaN()};
};

class CostmapFootprintClearanceCritic
  : public f_dwa_controller::FootprintClearanceCritic
{
public:
  bool configure(
    nav2_costmap_2d::Costmap2D * source,
    nav2_costmap_2d::Costmap2D * exclusion = nullptr,
    const double exclusion_tolerance = 0.0,
    const bool apply_aligned_tolerance = false)
  {
    costmap_ = source;
    exclusion_costmap_ = exclusion;
    exclude_layer_tolerance_ = exclusion_tolerance;
    apply_exclude_tolerance_on_aligned_grids_ = apply_aligned_tolerance;
    return refreshPenalizedCellMask();
  }

  bool configureProjected(
    nav2_costmap_2d::Costmap2D * source,
    nav2_costmap_2d::CostmapLayer * exclusion,
    const double exclusion_tolerance)
  {
    costmap_ = source;
    exclusion_costmap_layer_ = exclusion;
    project_exclusion_costmap_ = true;
    exclude_layer_tolerance_ = exclusion_tolerance;
    apply_exclude_tolerance_on_aligned_grids_ = true;
    refreshProjectedExclusionCostmap();
    return refreshPenalizedCellMask();
  }

  bool refreshDistanceField()
  {
    return refreshPenalizedCellMask();
  }

  void setExpandedFootprints(
    const std::vector<nav2_costmap_2d::Footprint> & footprints)
  {
    expanded_footprints_ = footprints;
  }

  bool setPhysicalFootprint(
    const nav2_costmap_2d::Footprint & footprint)
  {
    physical_footprint_ = footprint;
    return refreshFootprintBoundarySamples();
  }

  double minimumClearance(
    const geometry_msgs::msg::Pose2D & pose) const
  {
    return minimumFootprintClearance(pose);
  }

  double poseScore(const geometry_msgs::msg::Pose2D & pose) const
  {
    return scorePoseClearance(pose);
  }

  double score(const dwb_msgs::msg::Trajectory2D & trajectory)
  {
    return scoreTrajectory(trajectory);
  }

  void enableTrajectoryScoring()
  {
    global_plan_ = default_reference_plan();
    if (expanded_footprints_.empty()) {
      expanded_footprints_.push_back(physical_footprint_);
    }
    prepared_ = true;
  }

  bool hitsExpandedFootprint(
    const geometry_msgs::msg::Pose2D & pose,
    const std::size_t band) const
  {
    return expandedFootprintHitsLethal(pose, band);
  }

  bool isExcluded(const double world_x, const double world_y) const
  {
    return excludedByStaticLayer(world_x, world_y);
  }

  bool isPenalized(const unsigned int x, const unsigned int y) const
  {
    if (!costmap_) {
      return false;
    }
    const std::size_t index =
      static_cast<std::size_t>(y) * costmap_->getSizeInCellsX() + x;
    return index < penalized_cell_mask_.size() &&
           penalized_cell_mask_[index] != 0u;
  }

  double maximumProbeGap() const
  {
    return maximum_footprint_probe_gap_;
  }

  const std::vector<float> & distanceField() const
  {
    return obstacle_distance_field_;
  }

  std::size_t distanceTransformInputCapacity() const
  {
    return distance_transform_input_scratch_.capacity();
  }

  std::size_t distanceTransformOutputCapacity() const
  {
    return distance_transform_output_scratch_.capacity();
  }

  std::size_t distanceTransformSitesCapacity() const
  {
    return distance_transform_sites_scratch_.capacity();
  }

  std::size_t distanceTransformBoundariesCapacity() const
  {
    return distance_transform_boundaries_scratch_.capacity();
  }

  std::size_t rowDistanceCapacity() const
  {
    return row_distance_scratch_.capacity();
  }
};

class ProjectedStaticTestLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void reset() override {}

  bool isClearable() override {return false;}

  void updateBounds(
    double, double, double, double *, double *, double *, double *) override
  {}

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int, int, int, int) override
  {
    master_grid.setCost(4u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
};

nav2_costmap_2d::Footprint squareFootprint(const double half_extent)
{
  nav2_costmap_2d::Footprint footprint(4);
  footprint[0].x = -half_extent;
  footprint[0].y = -half_extent;
  footprint[1].x = half_extent;
  footprint[1].y = -half_extent;
  footprint[2].x = half_extent;
  footprint[2].y = half_extent;
  footprint[3].x = -half_extent;
  footprint[3].y = half_extent;
  return footprint;
}

builtin_interfaces::msg::Duration duration(double seconds)
{
  constexpr std::int64_t nanoseconds_per_second = 1000000000LL;
  const std::int64_t total_nanoseconds = static_cast<std::int64_t>(
    std::llround(seconds * static_cast<double>(nanoseconds_per_second)));
  builtin_interfaces::msg::Duration value;
  value.sec = static_cast<std::int32_t>(
    total_nanoseconds / nanoseconds_per_second);
  value.nanosec = static_cast<std::uint32_t>(
    total_nanoseconds % nanoseconds_per_second);
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
  trajectory.time_offsets = {duration(0.0), duration(seconds)};
  return trajectory;
}

dwb_msgs::msg::Trajectory2D fixed_step_terminal_trajectory(
  const double sim_time,
  const double prefix_speed,
  const double evaluation_x,
  const double heading = 0.0,
  const double suffix_speed = 0.0)
{
  constexpr double time_step = 0.05;
  constexpr double evaluation_time = 1.4;
  const std::size_t step_count = static_cast<std::size_t>(
    std::llround(sim_time / time_step));
  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D pose;
  pose.x = evaluation_x -
    prefix_speed * evaluation_time * std::cos(heading);
  pose.y = -prefix_speed * evaluation_time * std::sin(heading);
  pose.theta = heading;
  trajectory.poses.push_back(pose);
  double running_time = 0.0;
  for (std::size_t step = 0u; step < step_count; ++step) {
    const double speed = running_time < evaluation_time - 1.0e-12 ?
      prefix_speed : suffix_speed;
    pose.x += speed * time_step * std::cos(heading);
    pose.y += speed * time_step * std::sin(heading);
    trajectory.poses.push_back(pose);
    trajectory.time_offsets.push_back(duration(running_time));
    running_time += time_step;
  }
  trajectory.poses.push_back(pose);
  trajectory.time_offsets.push_back(duration(running_time));
  return trajectory;
}

dwb_msgs::msg::Trajectory2D constant_curvature_trajectory(
  const double path_distance,
  const double pose_spacing,
  const double curvature)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  const std::size_t complete_steps = static_cast<std::size_t>(
    std::floor(path_distance / pose_spacing + 1.0e-9));
  const auto append_pose = [&trajectory, curvature](const double distance) {
      geometry_msgs::msg::Pose2D pose;
      pose.theta = curvature * distance;
      pose.x = std::sin(pose.theta) / curvature;
      pose.y = (1.0 - std::cos(pose.theta)) / curvature;
      trajectory.poses.push_back(pose);
    };
  for (std::size_t index = 0u; index <= complete_steps; ++index) {
    append_pose(static_cast<double>(index) * pose_spacing);
  }
  if (path_distance -
    static_cast<double>(complete_steps) * pose_spacing > 1.0e-9)
  {
    append_pose(path_distance);
  }
  return trajectory;
}

dwb_msgs::msg::Trajectory2D timed_curvature_suffix_trajectory(
  const double sim_time,
  const double suffix_angular_velocity)
{
  constexpr double time_step = 0.05;
  constexpr double risk_seed_time = 1.4;
  constexpr double linear_velocity = 0.4;
  const std::size_t step_count = static_cast<std::size_t>(
    std::llround(sim_time / time_step));

  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D pose;
  trajectory.poses.push_back(pose);
  trajectory.time_offsets.push_back(duration(0.0));
  for (std::size_t step = 1u; step <= step_count; ++step) {
    const double time = static_cast<double>(step) * time_step;
    const double angular_velocity = time <= risk_seed_time + 1.0e-12 ?
      0.12 + 0.10 * time : suffix_angular_velocity;
    pose.x += linear_velocity * std::cos(pose.theta) * time_step;
    pose.y += linear_velocity * std::sin(pose.theta) * time_step;
    pose.theta += angular_velocity * time_step;
    trajectory.poses.push_back(pose);
    trajectory.time_offsets.push_back(duration(time));
  }
  // StandardTrajectoryGenerator's include_last_point convention adds one
  // unpaired duplicate after the final timed pose.
  trajectory.poses.push_back(pose);
  return trajectory;
}

dwb_msgs::msg::Trajectory2D timed_spatial_heading_trajectory(
  const double translation, const double heading,
  const double terminal_yaw_offset = 0.0)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D start;
  geometry_msgs::msg::Pose2D endpoint;
  endpoint.x = translation * std::cos(heading);
  endpoint.y = translation * std::sin(heading);
  endpoint.theta = heading + terminal_yaw_offset;
  trajectory.poses = {start, endpoint, endpoint};
  trajectory.time_offsets = {duration(0.0), duration(1.4)};
  return trajectory;
}

dwb_msgs::msg::Trajectory2D timed_spatial_heading_with_suffix(
  const double sim_time, const double translation, const double heading,
  const double terminal_yaw_offset = 0.0)
{
  auto trajectory = timed_spatial_heading_trajectory(
    translation, heading, terminal_yaw_offset);
  trajectory.poses.pop_back();
  auto suffix_pose = trajectory.poses.back();
  for (double time = 1.6; time <= sim_time + 1.0e-9; time += 0.2) {
    // Deliberately horizon-dependent and unrelated to the shared prefix. The
    // common predictor must exclude every one of these suffix poses.
    suffix_pose.x += 0.03 * std::cos(2.0 * time);
    suffix_pose.y += 0.03 * std::sin(3.0 * time);
    suffix_pose.theta -= 0.4;
    trajectory.poses.push_back(suffix_pose);
    trajectory.time_offsets.push_back(duration(time));
  }
  trajectory.poses.push_back(suffix_pose);
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

TEST(ForwardObstacleCritic, PenalizesBlockedNominalPath)
{
  StubForwardObstacleCritic critic;
  auto downward = trajectory_to(1.0, 1.0);
  downward.poses.back().y = -0.3;
  auto upward = downward;
  upward.poses.back().y = 0.3;

  EXPECT_GT(critic.scoreTrajectory(downward), 0.0);
  EXPECT_LT(critic.scoreTrajectory(downward), 1.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(upward), 0.0);
}

TEST(ForwardObstacleCritic, RewardsMovingTheFirstBlockedProbeFurtherAway)
{
  const auto trajectory = trajectory_to(1.0, 1.0);
  StubRangeForwardObstacleCritic near_obstacle(1.0);
  StubRangeForwardObstacleCritic far_obstacle(1.5);

  EXPECT_GT(
    near_obstacle.scoreTrajectory(trajectory),
    far_obstacle.scoreTrajectory(trajectory));
  EXPECT_GT(far_obstacle.scoreTrajectory(trajectory), 0.0);
}

TEST(ForwardObstacleCritic, RejectsEmptyAndNonFiniteTrajectories)
{
  StubForwardObstacleCritic critic;
  dwb_msgs::msg::Trajectory2D empty;
  EXPECT_THROW(
    critic.scoreTrajectory(empty), dwb_core::IllegalTrajectoryException);

  auto non_finite = trajectory_to(1.0, 1.0);
  non_finite.poses.back().x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    critic.scoreTrajectory(non_finite),
    dwb_core::IllegalTrajectoryException);
}

TEST(ForwardObstacleCritic, RasterizesCellsBetweenRiskPathSamples)
{
  nav2_costmap_2d::Costmap2D costmap(6, 3, 1.0, 0.0, 0.0, 0u);
  costmap.setCost(1u, 1u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapForwardObstacleCritic critic;
  critic.configure(&costmap, 4.0, 2.0);

  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D start;
  start.x = 0.5;
  start.y = 1.5;
  geometry_msgs::msg::Pose2D end = start;
  end.x = 4.5;
  trajectory.poses = {start, end};
  trajectory.time_offsets = {duration(0.0), duration(1.0)};

  // Samples fall in cells 0, 2 and 4. The lethal cell 1 is visible only when
  // the segment between samples is rasterized.
  EXPECT_GT(critic.scoreTrajectory(trajectory), 0.0);
}

TEST(FootprintClearanceCritic, PenalizesCloserClearanceWithoutRejecting)
{
  StubFootprintClearanceCritic critic;
  geometry_msgs::msg::Pose2D pose;

  critic.setFirstCollidingBand(0);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.9);
  critic.setFirstCollidingBand(3);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.3);
  critic.setFirstCollidingBand(5);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
  pose.theta = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 1.0);
}

TEST(FootprintClearanceCritic, AppliesContinuousPoweredClearancePenalty)
{
  StubFootprintClearanceCritic critic;
  geometry_msgs::msg::Pose2D pose;
  critic.setClearanceShape(0.75, 3.0);

  critic.setFixedClearance(0.60);
  const double at_sixty_centimetres = critic.scorePose(pose);
  EXPECT_NEAR(at_sixty_centimetres, 0.008, 1.0e-12);
  critic.setFixedClearance(0.60 - 1.0e-6);
  const double immediately_closer = critic.scorePose(pose);
  critic.setFixedClearance(0.60 + 1.0e-6);
  const double immediately_further = critic.scorePose(pose);

  EXPECT_GT(immediately_closer, at_sixty_centimetres);
  EXPECT_LT(immediately_further, at_sixty_centimetres);
  EXPECT_LT(immediately_closer - immediately_further, 1.0e-5);
  critic.setFixedClearance(0.75);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
  critic.setFixedClearance(0.76);
  EXPECT_DOUBLE_EQ(critic.scorePose(pose), 0.0);
}

TEST(FootprintClearanceCritic, ReturnsBoundedDistanceExposure)
{
  StubFootprintClearanceCritic critic;
  critic.setFirstCollidingBand(1);
  const auto trajectory = trajectory_to(1.0, 1.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory), 0.7);

  dwb_msgs::msg::Trajectory2D empty;
  EXPECT_THROW(
    critic.scoreTrajectory(empty), dwb_core::IllegalTrajectoryException);
}

TEST(FootprintClearanceCritic, RewardsLeavingAnAlreadyEnteredSoftMargin)
{
  StubFootprintClearanceCritic critic;
  critic.setFirstCollidingBand(0);
  critic.setMaximumCollisionAbsY(0.2);

  auto remains_inside = trajectory_to(1.0, 1.0);
  auto leaves_margin = remains_inside;
  leaves_margin.poses.back().y = 0.3;

  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(remains_inside), 0.9);
  EXPECT_NEAR(critic.scoreTrajectory(leaves_margin), 0.5625, 1.0e-12);
}

TEST(FootprintClearanceCritic, SoftlyObservesBeyondExecutableEndpoint)
{
  StubFootprintClearanceCritic critic;
  critic.setFirstCollidingBand(0);
  critic.setMinimumCollisionX(1.0);
  critic.setRiskPath(1.0, 2);

  const auto trajectory = trajectory_to(0.5, 1.0);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.5625, 1.0e-12);
}

TEST(FootprintClearanceCritic, WeightsShortFinalDistanceInterval)
{
  StubFootprintClearanceCritic critic;
  critic.setFirstCollidingBand(0);
  critic.setMinimumCollisionX(1.0);
  critic.setRiskPathResolution(1.0, 0.6);

  const auto trajectory = trajectory_to(0.5, 1.0);
  // Penalties at s={0, 0.6, 1.0} are {0, 0, 0.9}. The final exposure occupies
  // only 0.4 m, so its trapezoidal mean is 0.18 rather than a point mean 0.3.
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.54, 1.0e-12);
}

TEST(FootprintClearanceCritic, IgnoresInitialCommandVelocityInTerminalValue)
{
  StubFootprintClearanceCritic critic;
  critic.setFirstCollidingBand(0);
  critic.setMinimumCollisionX(1.0);
  critic.setMaximumCollisionAbsY(0.2);
  critic.setRiskPath(1.0, 2);

  auto first = trajectory_to(0.5, 1.0);
  first.velocity.x = 1.0;
  first.velocity.theta = 1.0;
  auto second = first;
  second.velocity.x = 0.0;
  second.velocity.theta = -1.0;

  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(first), critic.scoreTrajectory(second));
}

TEST(FootprintClearanceCritic, FindsIsolatedCellInsideExpandedBand)
{
  nav2_costmap_2d::Costmap2D costmap(60, 60, 0.1, -3.0, -3.0, 0u);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(0.45, 0.05, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  CostmapFootprintClearanceCritic critic;
  critic.configure(&costmap);
  ASSERT_TRUE(critic.setPhysicalFootprint(squareFootprint(0.25)));
  critic.setExpandedFootprints({squareFootprint(0.50)});
  geometry_msgs::msg::Pose2D pose;

  // The cell is inside the expanded margin but outside the physical square.
  // The distance-field query must retain this soft-clearance obstacle.
  EXPECT_TRUE(critic.hitsExpandedFootprint(pose, 0u));
}

TEST(FootprintClearanceCritic, BoundsSoftBoundaryProbesByClearanceBand)
{
  nav2_costmap_2d::Costmap2D costmap(80, 80, 0.025, -1.0, -1.0, 0u);
  CostmapFootprintClearanceCritic critic;
  critic.configure(&costmap);
  ASSERT_TRUE(critic.setPhysicalFootprint(squareFootprint(0.30)));

  // Defaults are a 0.25 m margin split into five bands.  Half the maximum
  // probe gap is subtracted from every clearance query, so a two-band gap
  // remains a conservative continuous-boundary lower bound.
  EXPECT_GT(critic.maximumProbeGap(), costmap.getResolution());
  EXPECT_LE(critic.maximumProbeGap(), 0.10 + 1.0e-12);
}

TEST(FootprintClearanceCritic, DistanceFieldSeparatesNearAndFarPoses)
{
  nav2_costmap_2d::Costmap2D costmap(100, 40, 0.05, -1.0, -1.0, 0u);
  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(1.0, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  CostmapFootprintClearanceCritic critic;
  critic.configure(&costmap);
  ASSERT_TRUE(critic.setPhysicalFootprint(squareFootprint(0.20)));
  critic.setExpandedFootprints({squareFootprint(0.45)});
  geometry_msgs::msg::Pose2D near_pose;
  near_pose.x = 0.55;
  geometry_msgs::msg::Pose2D far_pose;
  far_pose.x = -0.50;

  EXPECT_LT(critic.minimumClearance(near_pose), 0.25);
  EXPECT_GT(critic.minimumClearance(far_pose), 0.25);
}

TEST(FootprintClearanceCritic, ReusesDistanceFieldForUnchangedLethalMask)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0, 0u);
  costmap.setCost(10u, 10u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapFootprintClearanceCritic critic;

  EXPECT_TRUE(critic.configure(&costmap));
  EXPECT_FALSE(critic.refreshDistanceField());

  // Non-lethal costs do not contribute to this critic's source mask and must
  // not invalidate the exact lethal-cell distance field.
  costmap.setCost(0u, 0u, 42u);
  EXPECT_FALSE(critic.refreshDistanceField());

  costmap.setCost(15u, 10u, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(critic.refreshDistanceField());
  EXPECT_FALSE(critic.refreshDistanceField());
  costmap.setCost(10u, 10u, nav2_costmap_2d::FREE_SPACE);
  EXPECT_TRUE(critic.refreshDistanceField());
}

TEST(FootprintClearanceCritic, InvalidatesDistanceFieldForGridGeometryChanges)
{
  // Both grids place the sole lethal cell at linear mask index three.  Shape
  // and resolution therefore need explicit cache keys beyond mask equality.
  nav2_costmap_2d::Costmap2D wide(4, 2, 0.1, 0.0, 0.0, 0u);
  wide.setCost(3u, 0u, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav2_costmap_2d::Costmap2D tall(2, 4, 0.1, 0.0, 0.0, 0u);
  tall.setCost(1u, 1u, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav2_costmap_2d::Costmap2D coarse(2, 4, 0.2, 0.0, 0.0, 0u);
  coarse.setCost(1u, 1u, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav2_costmap_2d::Costmap2D shifted(2, 4, 0.2, 1.0, -2.0, 0u);
  shifted.setCost(1u, 1u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapFootprintClearanceCritic critic;

  EXPECT_TRUE(critic.configure(&wide));
  EXPECT_TRUE(critic.configure(&tall));
  EXPECT_TRUE(critic.configure(&coarse));
  // The EDT is expressed only in grid indices and metres per cell.  A changed
  // origin is applied by worldToMap at query time and needs no rebuild.
  EXPECT_FALSE(critic.configure(&shifted));
}

TEST(FootprintClearanceCritic, ExclusionEmptyFastPathMatchesGeneralPathExactly)
{
  nav2_costmap_2d::Costmap2D source(40, 24, 0.125, -1.0, -1.5, 0u);
  nav2_costmap_2d::Costmap2D empty_exclusion(
    40, 24, 0.125, -1.0, -1.5, 0u);
  source.setCost(1u, 1u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(20u, 12u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(28u, 8u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(4u, 20u, 42u);

  CostmapFootprintClearanceCritic direct;
  CostmapFootprintClearanceCritic general;
  ASSERT_TRUE(direct.configure(&source));
  ASSERT_TRUE(general.configure(&source, &empty_exclusion));
  ASSERT_EQ(direct.distanceField(), general.distanceField());
  for (unsigned int y = 0u; y < source.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0u; x < source.getSizeInCellsX(); ++x) {
      EXPECT_EQ(direct.isPenalized(x, y), general.isPenalized(x, y));
    }
  }

  const auto footprint = squareFootprint(0.1875);
  ASSERT_TRUE(direct.setPhysicalFootprint(footprint));
  ASSERT_TRUE(general.setPhysicalFootprint(footprint));
  direct.setExpandedFootprints({squareFootprint(0.4375)});
  general.setExpandedFootprints({squareFootprint(0.4375)});
  std::vector<geometry_msgs::msg::Pose2D> poses(3u);
  poses[1].x = 0.25;
  poses[1].y = 0.25;
  poses[2].x = -0.5;
  poses[2].y = 0.5;
  poses[2].theta = 0.4;
  for (const auto & pose : poses) {
    EXPECT_DOUBLE_EQ(
      direct.minimumClearance(pose), general.minimumClearance(pose));
    EXPECT_DOUBLE_EQ(direct.poseScore(pose), general.poseScore(pose));
  }
  direct.enableTrajectoryScoring();
  general.enableTrajectoryScoring();
  for (const auto & trajectory :
    std::vector<dwb_msgs::msg::Trajectory2D>{
    timed_spatial_heading_trajectory(0.20, -0.35),
    timed_spatial_heading_trajectory(0.35, 0.0),
    timed_spatial_heading_trajectory(0.20, 0.35)})
  {
    EXPECT_DOUBLE_EQ(direct.score(trajectory), general.score(trajectory));
  }

  source.setCost(20u, 12u, nav2_costmap_2d::FREE_SPACE);
  source.setCost(21u, 12u, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(direct.refreshDistanceField());
  ASSERT_TRUE(general.refreshDistanceField());
  EXPECT_EQ(direct.distanceField(), general.distanceField());
}

TEST(FootprintClearanceCritic, ReusesEdtScratchAcrossBigSmallBigGeometry)
{
  nav2_costmap_2d::Costmap2D source(24, 16, 0.125, -1.0, -1.0, 0u);
  source.setCost(20u, 12u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapFootprintClearanceCritic critic;
  ASSERT_TRUE(critic.configure(&source));
  const std::size_t large_input_capacity =
    critic.distanceTransformInputCapacity();
  const std::size_t large_output_capacity =
    critic.distanceTransformOutputCapacity();
  const std::size_t large_sites_capacity =
    critic.distanceTransformSitesCapacity();
  const std::size_t large_boundaries_capacity =
    critic.distanceTransformBoundariesCapacity();
  const std::size_t large_row_capacity = critic.rowDistanceCapacity();
  EXPECT_GE(large_input_capacity, 24u);
  EXPECT_GE(large_output_capacity, 24u);
  EXPECT_GE(large_sites_capacity, 24u);
  EXPECT_GE(large_boundaries_capacity, 25u);
  EXPECT_GE(large_row_capacity, 24u * 16u);

  source.resizeMap(7, 5, 0.25, 2.0, -3.0);
  source.setCost(2u, 3u, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(critic.refreshDistanceField());
  CostmapFootprintClearanceCritic fresh_small;
  ASSERT_TRUE(fresh_small.configure(&source));
  EXPECT_EQ(critic.distanceField(), fresh_small.distanceField());
  EXPECT_GE(critic.distanceTransformInputCapacity(), large_input_capacity);
  EXPECT_GE(critic.distanceTransformOutputCapacity(), large_output_capacity);
  EXPECT_GE(critic.distanceTransformSitesCapacity(), large_sites_capacity);
  EXPECT_GE(
    critic.distanceTransformBoundariesCapacity(), large_boundaries_capacity);
  EXPECT_GE(critic.rowDistanceCapacity(), large_row_capacity);

  source.resizeMap(24, 16, 0.125, -1.0, -1.0);
  source.setCost(3u, 2u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(21u, 13u, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(critic.refreshDistanceField());
  CostmapFootprintClearanceCritic fresh_large;
  ASSERT_TRUE(fresh_large.configure(&source));
  EXPECT_EQ(critic.distanceField(), fresh_large.distanceField());
  EXPECT_GE(critic.distanceTransformInputCapacity(), large_input_capacity);
  EXPECT_GE(critic.distanceTransformOutputCapacity(), large_output_capacity);
  EXPECT_GE(critic.distanceTransformSitesCapacity(), large_sites_capacity);
  EXPECT_GE(
    critic.distanceTransformBoundariesCapacity(), large_boundaries_capacity);
  EXPECT_GE(critic.rowDistanceCapacity(), large_row_capacity);
}

TEST(FootprintClearanceCritic, RollingOriginAndCellChangesCannotStaleDistanceField)
{
  nav2_costmap_2d::Costmap2D source(16, 12, 0.125, 0.0, 0.0, 0u);
  source.setCost(10u, 6u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapFootprintClearanceCritic critic;
  ASSERT_TRUE(critic.configure(&source));
  const auto footprint = squareFootprint(0.125);
  ASSERT_TRUE(critic.setPhysicalFootprint(footprint));
  critic.setExpandedFootprints({squareFootprint(0.375)});
  geometry_msgs::msg::Pose2D original_pose;
  original_pose.x = 0.75;
  original_pose.y = 0.8125;
  const double original_clearance = critic.minimumClearance(original_pose);
  const auto original_field = critic.distanceField();

  // The index mask and resolution are unchanged, so the EDT is reusable. The
  // live origin must nevertheless be used for clearance queries.
  source.resizeMap(16, 12, 0.125, 1.0, -2.0);
  source.setCost(10u, 6u, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(critic.refreshDistanceField());
  EXPECT_EQ(critic.distanceField(), original_field);
  ASSERT_TRUE(critic.setPhysicalFootprint(footprint));
  geometry_msgs::msg::Pose2D shifted_pose = original_pose;
  shifted_pose.x += 1.0;
  shifted_pose.y -= 2.0;
  EXPECT_DOUBLE_EQ(critic.minimumClearance(shifted_pose), original_clearance);

  source.setCost(10u, 6u, nav2_costmap_2d::FREE_SPACE);
  source.setCost(11u, 6u, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(critic.refreshDistanceField());
  CostmapFootprintClearanceCritic fresh;
  ASSERT_TRUE(fresh.configure(&source));
  ASSERT_TRUE(fresh.setPhysicalFootprint(footprint));
  EXPECT_EQ(critic.distanceField(), fresh.distanceField());
  EXPECT_DOUBLE_EQ(
    critic.minimumClearance(shifted_pose),
    fresh.minimumClearance(shifted_pose));
}

TEST(FootprintClearanceCritic, InvalidatesDistanceFieldForExclusionChanges)
{
  nav2_costmap_2d::Costmap2D source(20, 20, 0.1, 0.0, 0.0, 0u);
  nav2_costmap_2d::Costmap2D exclusion(20, 20, 0.1, 0.0, 0.0, 0u);
  source.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  CostmapFootprintClearanceCritic critic;

  EXPECT_TRUE(critic.configure(&source, &exclusion));
  EXPECT_TRUE(critic.isPenalized(5u, 5u));
  exclusion.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(critic.refreshDistanceField());
  EXPECT_FALSE(critic.isPenalized(5u, 5u));
  EXPECT_FALSE(critic.refreshDistanceField());
  exclusion.setCost(5u, 5u, nav2_costmap_2d::FREE_SPACE);
  EXPECT_TRUE(critic.refreshDistanceField());
  EXPECT_TRUE(critic.isPenalized(5u, 5u));
}

TEST(FootprintClearanceCritic, ProjectsRollingStaticExclusionBeforeTolerance)
{
  nav2_costmap_2d::Costmap2D source(12, 12, 0.1, 3.0, -2.0, 0u);
  source.setCost(4u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(7u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  ProjectedStaticTestLayer static_layer;
  CostmapFootprintClearanceCritic critic;

  ASSERT_TRUE(critic.configureProjected(&source, &static_layer, 0.1));
  EXPECT_FALSE(critic.isPenalized(4u, 5u));
  EXPECT_FALSE(critic.isPenalized(5u, 5u));
  EXPECT_TRUE(critic.isPenalized(7u, 5u));
}

TEST(FootprintClearanceCritic, EmptyDynamicLayerHasNoSoftObstacle)
{
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.05, -1.0, -1.0, 0u);
  CostmapFootprintClearanceCritic critic;
  critic.configure(&costmap);
  ASSERT_TRUE(critic.setPhysicalFootprint(squareFootprint(0.20)));
  critic.setExpandedFootprints({squareFootprint(0.45)});
  geometry_msgs::msg::Pose2D pose;

  EXPECT_GT(critic.minimumClearance(pose), 0.25);
  EXPECT_FALSE(critic.hitsExpandedFootprint(pose, 0u));
}

TEST(FootprintClearanceCritic, EmptyFootprintAndNonFinitePoseFailClosed)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0, 0u);
  CostmapFootprintClearanceCritic critic;
  critic.configure(&costmap);
  EXPECT_FALSE(critic.setPhysicalFootprint({}));
  critic.setExpandedFootprints({squareFootprint(0.45)});
  geometry_msgs::msg::Pose2D pose;
  EXPECT_DOUBLE_EQ(critic.minimumClearance(pose), 0.0);
  pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(critic.minimumClearance(pose), 0.0);
  EXPECT_THROW(
    critic.score(trajectory_to(1.0, 1.0)),
    dwb_core::IllegalTrajectoryException);
}

TEST(FootprintClearanceCritic, RejectsDegenerateRiskDistance)
{
  StubFootprintClearanceCritic critic;
  critic.setRiskPathResolution(1.0e-10, 1.0e-10);
  EXPECT_THROW(
    critic.scoreTrajectory(trajectory_to(1.0, 1.0)),
    dwb_core::IllegalTrajectoryException);
}

TEST(FootprintClearanceCritic, PreservesAlignedStaticAdjacentProtrusion)
{
  nav2_costmap_2d::Costmap2D source(20, 20, 0.1, 0.0, 0.0, 0u);
  nav2_costmap_2d::Costmap2D static_layer(20, 20, 0.1, 0.0, 0.0, 0u);
  static_layer.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(6u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);

  CostmapFootprintClearanceCritic critic;
  critic.configure(&source, &static_layer, 0.2);
  double static_x = 0.0;
  double static_y = 0.0;
  double protrusion_x = 0.0;
  double protrusion_y = 0.0;
  source.mapToWorld(5u, 5u, static_x, static_y);
  source.mapToWorld(6u, 5u, protrusion_x, protrusion_y);

  EXPECT_TRUE(critic.isExcluded(static_x, static_y));
  EXPECT_FALSE(critic.isExcluded(protrusion_x, protrusion_y));
  EXPECT_FALSE(critic.isPenalized(5u, 5u));
  EXPECT_TRUE(critic.isPenalized(6u, 5u));
}

TEST(FootprintClearanceCritic, OptionallyExcludesAlignedStaticRegistrationError)
{
  nav2_costmap_2d::Costmap2D source(20, 20, 0.05, 0.0, 0.0, 0u);
  nav2_costmap_2d::Costmap2D static_layer(20, 20, 0.05, 0.0, 0.0, 0u);
  static_layer.setCost(5u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);
  source.setCost(7u, 5u, nav2_costmap_2d::LETHAL_OBSTACLE);

  CostmapFootprintClearanceCritic critic;
  critic.configure(&source, &static_layer, 0.10, true);
  double shifted_x = 0.0;
  double shifted_y = 0.0;
  source.mapToWorld(7u, 5u, shifted_x, shifted_y);

  EXPECT_TRUE(critic.isExcluded(shifted_x, shifted_y));
  EXPECT_FALSE(critic.isPenalized(7u, 5u));
}

TEST(FixedDistanceRiskPath, IsIndependentOfTemporalPoseSampling)
{
  dwb_msgs::msg::Trajectory2D coarse;
  coarse.poses.resize(2);
  coarse.poses[1].x = 1.0;
  dwb_msgs::msg::Trajectory2D dense;
  dense.poses.resize(5);
  for (std::size_t index = 0u; index < dense.poses.size(); ++index) {
    dense.poses[index].x = 0.25 * static_cast<double>(index);
  }
  coarse.velocity.x = 0.6;
  dense.velocity.x = 0.1;
  dense.velocity.theta = 0.5;

  const auto coarse_path =
    f_dwa_controller::build_fixed_distance_risk_path(coarse, 2.5, 0.1);
  const auto dense_path =
    f_dwa_controller::build_fixed_distance_risk_path(dense, 2.5, 0.1);
  ASSERT_EQ(coarse_path.size(), 26u);
  ASSERT_EQ(coarse_path.size(), dense_path.size());
  for (std::size_t index = 0u; index < coarse_path.size(); ++index) {
    EXPECT_NEAR(
      coarse_path[index].arc_length, dense_path[index].arc_length, 1.0e-12);
    EXPECT_NEAR(coarse_path[index].pose.x, dense_path[index].pose.x, 1.0e-12);
    EXPECT_NEAR(coarse_path[index].pose.y, dense_path[index].pose.y, 1.0e-12);
  }
  EXPECT_NEAR(coarse_path.back().arc_length, 2.5, 1.0e-12);
  EXPECT_NEAR(coarse_path.back().pose.x, 2.5, 1.0e-12);
}

TEST(FixedDistanceRiskPath, ConstantCurvatureIsIndependentOfTemporalHorizon)
{
  const auto short_trajectory =
    constant_curvature_trajectory(0.84, 0.01, 0.4);
  const auto canonical_trajectory =
    constant_curvature_trajectory(0.96, 0.01, 0.4);
  const auto long_trajectory =
    constant_curvature_trajectory(1.08, 0.01, 0.4);

  const auto short_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    short_trajectory, 2.5, 0.05);
  const auto canonical_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    canonical_trajectory, 2.5, 0.05);
  const auto long_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    long_trajectory, 2.5, 0.05);
  ASSERT_EQ(short_path.size(), canonical_path.size());
  ASSERT_EQ(short_path.size(), long_path.size());
  for (std::size_t index = 0u; index < short_path.size(); ++index) {
    EXPECT_NEAR(
      short_path[index].pose.x, canonical_path[index].pose.x, 2.0e-5);
    EXPECT_NEAR(
      short_path[index].pose.y, canonical_path[index].pose.y, 2.0e-5);
    EXPECT_NEAR(
      std::remainder(
        short_path[index].pose.theta - canonical_path[index].pose.theta,
        2.0 * M_PI),
      0.0, 2.0e-5);
    EXPECT_NEAR(
      long_path[index].pose.x, canonical_path[index].pose.x, 2.0e-5);
    EXPECT_NEAR(
      long_path[index].pose.y, canonical_path[index].pose.y, 2.0e-5);
    EXPECT_NEAR(
      std::remainder(
        long_path[index].pose.theta - canonical_path[index].pose.theta,
        2.0 * M_PI),
      0.0, 2.0e-5);
  }
}

TEST(FixedDistanceRiskPath, FixedTimedPrefixIgnoresVariableCurvatureSuffix)
{
  const auto short_trajectory =
    timed_curvature_suffix_trajectory(1.4, 0.0);
  const auto canonical_trajectory =
    timed_curvature_suffix_trajectory(1.6, -1.2);
  const auto long_trajectory =
    timed_curvature_suffix_trajectory(1.8, 1.2);

  const auto short_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    short_trajectory, 2.5, 0.05, 1.4);
  const auto canonical_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    canonical_trajectory, 2.5, 0.05, 1.4);
  const auto long_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    long_trajectory, 2.5, 0.05, 1.4);

  ASSERT_EQ(short_path.size(), 51u);
  ASSERT_EQ(canonical_path.size(), short_path.size());
  ASSERT_EQ(long_path.size(), short_path.size());
  EXPECT_NEAR(short_path.back().arc_length, 2.5, 1.0e-12);
  for (std::size_t index = 0u; index < short_path.size(); ++index) {
    EXPECT_NEAR(
      short_path[index].pose.x, canonical_path[index].pose.x, 1.0e-12);
    EXPECT_NEAR(
      short_path[index].pose.y, canonical_path[index].pose.y, 1.0e-12);
    EXPECT_NEAR(
      std::remainder(
        short_path[index].pose.theta - canonical_path[index].pose.theta,
        2.0 * M_PI),
      0.0, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.x, long_path[index].pose.x, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.y, long_path[index].pose.y, 1.0e-12);
    EXPECT_NEAR(
      std::remainder(
        short_path[index].pose.theta - long_path[index].pose.theta,
        2.0 * M_PI),
      0.0, 1.0e-12);
  }

  const auto decimetre_samples =
    f_dwa_controller::build_fixed_distance_risk_path(
    long_trajectory, 2.5, 0.10, 1.4);
  ASSERT_EQ(decimetre_samples.size(), 26u);
  EXPECT_NEAR(decimetre_samples.back().arc_length, 2.5, 1.0e-12);
}

TEST(FixedDistanceRiskPath, PlanContinuationAvoidsHeldCurvatureFalsePenalty)
{
  nav_2d_msgs::msg::Path2D bending_plan;
  bending_plan.poses.resize(6u);
  bending_plan.poses[1].x = 0.5;
  bending_plan.poses[2].x = 1.0;
  bending_plan.poses[3].x = 1.5;
  bending_plan.poses[3].y = 0.10;
  bending_plan.poses[4].x = 2.0;
  bending_plan.poses[4].y = 0.40;
  bending_plan.poses[5].x = 2.4;
  bending_plan.poses[5].y = 0.90;

  const auto candidate = timed_curvature_suffix_trajectory(1.8, 1.2);
  const auto held_curvature =
    f_dwa_controller::build_fixed_distance_risk_path(
    candidate, 2.5, 0.02, 1.4);
  const auto plan_continued =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, bending_plan, 2.5, 0.02, 1.4);
  ASSERT_EQ(held_curvature.size(), plan_continued.size());
  ASSERT_GT(candidate.poses.size(), 28u);
  ASSERT_GT(plan_continued.size(), 28u);
  EXPECT_NEAR(
    plan_continued[28].pose.x, candidate.poses[28].x, 1.0e-9);
  EXPECT_NEAR(
    plan_continued[28].pose.y, candidate.poses[28].y, 1.0e-9);
  EXPECT_NEAR(
    std::remainder(
      plan_continued[28].pose.theta - candidate.poses[28].theta,
      2.0 * M_PI),
    0.0, 1.0e-9);

  // The old terminal-curvature probe bends into this synthetic blocker. The
  // candidate's legal native prefix is unchanged, but the plan continuation
  // follows the upcoming gentler bend and does not invent that encounter.
  const auto blocker = held_curvature.back().pose;
  double continued_clearance = std::numeric_limits<double>::infinity();
  for (const auto & sample : plan_continued) {
    continued_clearance = std::min(
      continued_clearance,
      std::hypot(sample.pose.x - blocker.x, sample.pose.y - blocker.y));
  }
  EXPECT_GT(continued_clearance, 0.30);

  // Spatial heading relaxes wrap-aware along the reference instead of using
  // a horizon-dependent endpoint yaw or snapping to a plan pose orientation.
  for (std::size_t index = 1u; index < plan_continued.size(); ++index) {
    EXPECT_LT(
      std::abs(std::remainder(
        plan_continued[index].pose.theta -
        plan_continued[index - 1u].pose.theta, 2.0 * M_PI)),
      0.20);
  }
}

TEST(FixedDistanceRiskPath, StraightSpatialHeadingStaysOnStraightPlan)
{
  const auto risk_path = f_dwa_controller::build_plan_continued_risk_path(
    timed_spatial_heading_trajectory(0.5, 0.0), straight_path(3.0),
    2.5, 0.05, 1.4, 1.0);

  ASSERT_EQ(risk_path.size(), 51u);
  for (const auto & sample : risk_path) {
    EXPECT_NEAR(sample.pose.y, 0.0, 1.0e-12);
    EXPECT_NEAR(
      std::remainder(sample.pose.theta, 2.0 * M_PI), 0.0, 1.0e-12);
  }
  EXPECT_NEAR(risk_path.back().pose.x, 2.5, 1.0e-12);
}

TEST(FixedDistanceRiskPath, SpatialHeadingCreatesSymmetricDetourSignal)
{
  constexpr double heading = 0.4;
  const auto left_candidate =
    timed_spatial_heading_trajectory(0.5, heading);
  const auto right_candidate =
    timed_spatial_heading_trajectory(0.5, -heading);
  const auto plan = straight_path(4.0);
  const auto left = f_dwa_controller::build_plan_continued_risk_path(
    left_candidate, plan, 2.5, 0.025, 1.4, 1.0);
  const auto right = f_dwa_controller::build_plan_continued_risk_path(
    right_candidate, plan, 2.5, 0.025, 1.4, 1.0);

  ASSERT_EQ(left.size(), right.size());
  const double native_endpoint_y = left_candidate.poses[1].y;
  EXPECT_GT(left.back().pose.y - native_endpoint_y, 0.15);
  EXPECT_LT(right.back().pose.y + native_endpoint_y, -0.15);
  for (std::size_t index = 0u; index < left.size(); ++index) {
    EXPECT_NEAR(left[index].pose.x, right[index].pose.x, 1.0e-9);
    EXPECT_NEAR(left[index].pose.y, -right[index].pose.y, 1.0e-9);
    EXPECT_NEAR(
      std::remainder(
        left[index].pose.theta + right[index].pose.theta,
        2.0 * M_PI),
      0.0, 1.0e-9);
  }
}

TEST(FixedDistanceRiskPath, ContinuationIgnoresTerminalYawWithoutTranslation)
{
  const auto plan = straight_path(4.0);
  const auto nominal = f_dwa_controller::build_plan_continued_risk_path(
    timed_spatial_heading_trajectory(0.5, 0.35), plan,
    2.5, 0.05, 1.4, 1.0);
  const auto terminal_yaw_changed =
    f_dwa_controller::build_plan_continued_risk_path(
    timed_spatial_heading_trajectory(0.5, 0.35, -1.2), plan,
    2.5, 0.05, 1.4, 1.0);

  ASSERT_EQ(nominal.size(), terminal_yaw_changed.size());
  for (std::size_t index = 0u; index < nominal.size(); ++index) {
    EXPECT_NEAR(
      nominal[index].pose.x, terminal_yaw_changed[index].pose.x, 1.0e-12);
    EXPECT_NEAR(
      nominal[index].pose.y, terminal_yaw_changed[index].pose.y, 1.0e-12);
    // The native endpoint yaw is deliberately preserved through the first
    // interpolation segment. Beyond that seed transition, only spatial
    // heading controls the continuation.
    if (nominal[index].arc_length > 0.60 + 1.0e-9) {
      EXPECT_NEAR(
        std::remainder(
          nominal[index].pose.theta -
          terminal_yaw_changed[index].pose.theta,
          2.0 * M_PI),
        0.0, 1.0e-12);
    }
  }
}

TEST(FixedDistanceRiskPath, NearZeroHeadingErrorIsFiniteAndContinuous)
{
  const auto plan = straight_path(3.0);
  const auto straight = f_dwa_controller::build_plan_continued_risk_path(
    timed_spatial_heading_trajectory(0.5, 0.0), plan,
    2.5, 0.05, 1.4, 1.0);
  const auto nearly_straight =
    f_dwa_controller::build_plan_continued_risk_path(
    timed_spatial_heading_trajectory(0.5, 1.0e-10), plan,
    2.5, 0.05, 1.4, 1.0);

  ASSERT_EQ(straight.size(), nearly_straight.size());
  for (std::size_t index = 0u; index < straight.size(); ++index) {
    EXPECT_TRUE(std::isfinite(nearly_straight[index].pose.x));
    EXPECT_TRUE(std::isfinite(nearly_straight[index].pose.y));
    EXPECT_TRUE(std::isfinite(nearly_straight[index].pose.theta));
    EXPECT_NEAR(
      nearly_straight[index].pose.x, straight[index].pose.x, 1.0e-9);
    EXPECT_NEAR(
      nearly_straight[index].pose.y, straight[index].pose.y, 5.0e-10);
  }
}

TEST(FixedDistanceRiskPath, RotationWithoutCredibleTranslationCannotInventDetour)
{
  const auto plan = straight_path(3.0);
  for (const double sim_time : {1.4, 1.6, 1.8}) {
    const auto pure_rotation =
      f_dwa_controller::build_plan_continued_risk_path(
      timed_spatial_heading_with_suffix(
        sim_time, 0.0, 0.0, M_PI_2),
      plan, 2.5, 0.05, 1.4, 1.0);
    for (const auto & sample : pure_rotation) {
      EXPECT_NEAR(sample.pose.y, 0.0, 1.0e-12);
    }
  }

  // The 0.09 m chord is deliberately below the documented 0.10 m heading
  // credibility threshold. Its achieved offset remains, but it gains no
  // additional metre-long lateral displacement from its terminal yaw.
  const auto low_translation_candidate =
    timed_spatial_heading_trajectory(0.09, M_PI_2);
  const auto low_translation =
    f_dwa_controller::build_plan_continued_risk_path(
    low_translation_candidate, plan, 2.5, 0.05, 1.4, 1.0);
  const double achieved_offset = low_translation_candidate.poses[1].y;
  for (const auto & sample : low_translation) {
    EXPECT_LE(std::abs(sample.pose.y), achieved_offset + 1.0e-12);
  }
  EXPECT_NEAR(low_translation.back().pose.y, achieved_offset, 1.0e-12);
}

TEST(FixedDistanceRiskPath, HeadingRelaxationRestoresCommonBlockerGradient)
{
  constexpr double turning_heading = 0.20;
  const auto straight = timed_spatial_heading_trajectory(0.5, 0.0);
  const auto turning =
    timed_spatial_heading_trajectory(0.5, turning_heading);
  const auto stopped = timed_spatial_heading_trajectory(0.0, 0.0);

  // Both moving candidates have the same seed translation. The turn's native
  // endpoint offset remains inside the synthetic 0.12 m blocker cross-section,
  // so a constant-offset continuation would still be blocked. Only the new
  // integrated heading relaxation creates the clearance measured below.
  ASSERT_LT(std::abs(turning.poses[1].y), 0.12);
  ReferenceBlockerForwardObstacleCritic obstacle_critic;
  const double straight_obstacle_cost =
    obstacle_critic.scoreTrajectory(straight);
  const double turning_obstacle_cost =
    obstacle_critic.scoreTrajectory(turning);
  const double stopped_obstacle_cost =
    obstacle_critic.scoreTrajectory(stopped);
  EXPECT_LT(turning_obstacle_cost, straight_obstacle_cost);
  EXPECT_LT(turning_obstacle_cost, stopped_obstacle_cost);

  StubFootprintClearanceCritic clearance_critic;
  clearance_critic.setRiskPathResolution(2.5, 0.05);
  clearance_critic.setRiskSeedTime(1.4);
  clearance_critic.setFirstCollidingBand(0u);
  clearance_critic.setMinimumCollisionX(1.3);
  clearance_critic.setMaximumCollisionAbsY(0.12);
  const double straight_clearance_cost =
    clearance_critic.scoreTrajectory(straight);
  const double turning_clearance_cost =
    clearance_critic.scoreTrajectory(turning);
  const double stopped_clearance_cost =
    clearance_critic.scoreTrajectory(stopped);
  EXPECT_LT(turning_clearance_cost, straight_clearance_cost);
  EXPECT_LT(turning_clearance_cost, stopped_clearance_cost);
}

TEST(FixedDistanceRiskPath, TwoPhaseManeuverClearsBlockerAcrossSimTimes)
{
  constexpr double turning_heading = 0.20;
  const std::vector<double> sim_times{1.4, 1.6, 1.8};
  std::vector<double> turning_obstacle_costs;
  std::vector<double> turning_clearance_costs;

  for (const double sim_time : sim_times) {
    const auto straight = timed_spatial_heading_with_suffix(
      sim_time, 0.5, 0.0);
    const auto turning = timed_spatial_heading_with_suffix(
      sim_time, 0.5, turning_heading);
    const auto stopped = timed_spatial_heading_with_suffix(
      sim_time, 0.0, 0.0);

    // At y=0.099 m the native endpoint, and the former one-phase relaxation
    // maximum of about 0.199 m, both remain inside this 0.24 m blocker. The
    // fixed 0.50 m maneuver phase is therefore what exposes the valid detour.
    ASSERT_LT(std::abs(turning.poses[1].y), 0.12);
    ManeuverPhaseBlockerForwardObstacleCritic obstacle_critic;
    const double straight_obstacle_cost =
      obstacle_critic.scoreTrajectory(straight);
    const double turning_obstacle_cost =
      obstacle_critic.scoreTrajectory(turning);
    const double stopped_obstacle_cost =
      obstacle_critic.scoreTrajectory(stopped);
    EXPECT_LT(turning_obstacle_cost, straight_obstacle_cost);
    EXPECT_LT(turning_obstacle_cost, stopped_obstacle_cost);
    turning_obstacle_costs.push_back(turning_obstacle_cost);

    StubFootprintClearanceCritic clearance_critic;
    clearance_critic.setRiskPathResolution(2.5, 0.05);
    clearance_critic.setRiskSeedTime(1.4);
    clearance_critic.setFirstCollidingBand(0u);
    clearance_critic.setMinimumCollisionX(1.6);
    clearance_critic.setMaximumCollisionAbsY(0.24);
    const double straight_clearance_cost =
      clearance_critic.scoreTrajectory(straight);
    const double turning_clearance_cost =
      clearance_critic.scoreTrajectory(turning);
    const double stopped_clearance_cost =
      clearance_critic.scoreTrajectory(stopped);
    EXPECT_LT(turning_clearance_cost, straight_clearance_cost);
    EXPECT_LT(turning_clearance_cost, stopped_clearance_cost);
    turning_clearance_costs.push_back(turning_clearance_cost);

    const auto risk_path =
      f_dwa_controller::build_plan_continued_risk_path(
      turning, straight_path(4.0), 2.5, 0.05, 1.4, 1.0);
    double maximum_lateral_separation = 0.0;
    for (const auto & sample : risk_path) {
      maximum_lateral_separation = std::max(
        maximum_lateral_separation, std::abs(sample.pose.y));
    }
    EXPECT_GT(maximum_lateral_separation, 0.27);
  }

  ASSERT_EQ(turning_obstacle_costs.size(), sim_times.size());
  ASSERT_EQ(turning_clearance_costs.size(), sim_times.size());
  for (std::size_t index = 1u; index < sim_times.size(); ++index) {
    EXPECT_NEAR(
      turning_obstacle_costs[index], turning_obstacle_costs.front(),
      1.0e-12);
    EXPECT_NEAR(
      turning_clearance_costs[index], turning_clearance_costs.front(),
      1.0e-12);
  }
}

TEST(FixedDistanceRiskPath, PlanContinuationPreservesDetourSide)
{
  const auto make_candidate = [](const double lateral_offset, const bool move) {
      dwb_msgs::msg::Trajectory2D trajectory;
      geometry_msgs::msg::Pose2D start;
      start.y = lateral_offset;
      geometry_msgs::msg::Pose2D endpoint = start;
      if (move) {
        endpoint.x = 0.7;
      }
      trajectory.poses = {start, endpoint, endpoint};
      trajectory.time_offsets = {duration(0.0), duration(1.4)};
      return trajectory;
    };
  const auto plan = straight_path(3.0);
  const auto centre_candidate = make_candidate(0.0, false);
  const auto left_candidate = make_candidate(0.3, true);
  const auto right_candidate = make_candidate(-0.3, true);
  const auto centre = f_dwa_controller::build_plan_continued_risk_path(
    centre_candidate, plan, 2.5, 0.05, 1.4);
  const auto left = f_dwa_controller::build_plan_continued_risk_path(
    left_candidate, plan, 2.5, 0.05, 1.4);
  const auto right = f_dwa_controller::build_plan_continued_risk_path(
    right_candidate, plan, 2.5, 0.05, 1.4);

  const auto minimum_blocker_distance = [](
    const std::vector<f_dwa_controller::RiskPathSample> & path,
    const double blocker_y)
    {
      double distance = std::numeric_limits<double>::infinity();
      for (const auto & sample : path) {
        distance = std::min(
          distance,
          std::hypot(sample.pose.x - 1.5, sample.pose.y - blocker_y));
      }
      return distance;
    };
  EXPECT_LT(minimum_blocker_distance(centre, 0.0), 1.0e-9);
  EXPECT_GT(minimum_blocker_distance(left, 0.0), 0.29);
  EXPECT_GT(minimum_blocker_distance(right, 0.0), 0.29);
  EXPECT_LT(
    minimum_blocker_distance(left, 0.2),
    minimum_blocker_distance(right, 0.2));
  EXPECT_NEAR(left.back().pose.y, 0.3, 1.0e-9);
  EXPECT_NEAR(right.back().pose.y, -0.3, 1.0e-9);

  ReferenceBlockerForwardObstacleCritic critic;
  const double centre_cost = critic.scoreTrajectory(centre_candidate);
  const double left_cost = critic.scoreTrajectory(left_candidate);
  const double right_cost = critic.scoreTrajectory(right_candidate);
  EXPECT_GT(centre_cost, 0.0);
  EXPECT_DOUBLE_EQ(left_cost, 0.0);
  EXPECT_DOUBLE_EQ(right_cost, 0.0);
}

TEST(FixedDistanceRiskPath, PlanContinuationIsSuffixInvariant)
{
  nav_2d_msgs::msg::Path2D plan;
  plan.poses.resize(4u);
  plan.poses[1].x = 0.8;
  plan.poses[2].x = 1.4;
  plan.poses[2].y = 0.2;
  plan.poses[3].x = 1.8;
  plan.poses[3].y = 0.6;
  const auto short_path = f_dwa_controller::build_plan_continued_risk_path(
    timed_curvature_suffix_trajectory(1.4, 0.0), plan, 2.5, 0.05, 1.4);
  const auto medium_path = f_dwa_controller::build_plan_continued_risk_path(
    timed_curvature_suffix_trajectory(1.6, -1.2), plan, 2.5, 0.05, 1.4);
  const auto long_path = f_dwa_controller::build_plan_continued_risk_path(
    timed_curvature_suffix_trajectory(1.8, 1.2), plan, 2.5, 0.05, 1.4);
  ASSERT_EQ(short_path.size(), medium_path.size());
  ASSERT_EQ(short_path.size(), long_path.size());
  for (std::size_t index = 0u; index < short_path.size(); ++index) {
    EXPECT_NEAR(short_path[index].pose.x, medium_path[index].pose.x, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.y, medium_path[index].pose.y, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.theta,
      medium_path[index].pose.theta, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.x, long_path[index].pose.x, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.y, long_path[index].pose.y, 1.0e-12);
    EXPECT_NEAR(short_path[index].pose.theta,
      long_path[index].pose.theta, 1.0e-12);
  }
}

TEST(FixedDistanceRiskPath, PreparedPlanGeometryMatchesCompatibilityPath)
{
  nav_2d_msgs::msg::Path2D plan;
  plan.poses.resize(6u);
  plan.poses[1].x = 0.8;
  plan.poses[2] = plan.poses[1];
  plan.poses[3].x = 1.4;
  plan.poses[3].y = 0.2;
  plan.poses[4] = plan.poses[3];
  plan.poses[5].x = 1.8;
  plan.poses[5].y = 0.7;
  const auto candidate = timed_curvature_suffix_trajectory(1.8, 1.2);
  const auto prepared =
    f_dwa_controller::prepare_plan_continuation_geometry(plan);

  const auto compatibility_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, plan, 2.5, 0.05, 1.4);
  const auto prepared_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, prepared, 2.5, 0.05, 1.4);

  ASSERT_EQ(prepared_path.size(), compatibility_path.size());
  for (std::size_t index = 0u; index < prepared_path.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      prepared_path[index].arc_length,
      compatibility_path[index].arc_length);
    EXPECT_DOUBLE_EQ(
      prepared_path[index].pose.x, compatibility_path[index].pose.x);
    EXPECT_DOUBLE_EQ(
      prepared_path[index].pose.y, compatibility_path[index].pose.y);
    EXPECT_DOUBLE_EQ(
      prepared_path[index].pose.theta,
      compatibility_path[index].pose.theta);
  }
}

TEST(FixedDistanceRiskPath, PreparedPlanGeometryRebuildsAfterPlanChange)
{
  auto plan = straight_path(3.0);
  const auto candidate = timed_curvature_suffix_trajectory(1.4, 0.0);
  const auto original_geometry =
    f_dwa_controller::prepare_plan_continuation_geometry(plan);
  const auto original_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, original_geometry, 2.5, 0.05, 1.4);

  plan.poses.back().y = 1.2;
  const auto original_snapshot_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, original_geometry, 2.5, 0.05, 1.4);
  ASSERT_EQ(original_snapshot_path.size(), original_path.size());
  for (std::size_t index = 0u; index < original_path.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      original_snapshot_path[index].pose.x, original_path[index].pose.x);
    EXPECT_DOUBLE_EQ(
      original_snapshot_path[index].pose.y, original_path[index].pose.y);
    EXPECT_DOUBLE_EQ(
      original_snapshot_path[index].pose.theta,
      original_path[index].pose.theta);
  }

  const auto updated_geometry =
    f_dwa_controller::prepare_plan_continuation_geometry(plan);
  const auto updated_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, updated_geometry, 2.5, 0.05, 1.4);
  const auto compatibility_updated_path =
    f_dwa_controller::build_plan_continued_risk_path(
    candidate, plan, 2.5, 0.05, 1.4);
  ASSERT_EQ(updated_path.size(), compatibility_updated_path.size());
  for (std::size_t index = 0u; index < updated_path.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      updated_path[index].pose.x, compatibility_updated_path[index].pose.x);
    EXPECT_DOUBLE_EQ(
      updated_path[index].pose.y, compatibility_updated_path[index].pose.y);
    EXPECT_DOUBLE_EQ(
      updated_path[index].pose.theta,
      compatibility_updated_path[index].pose.theta);
  }
  EXPECT_GT(
    std::abs(updated_path.back().pose.y - original_path.back().pose.y),
    0.25);
}

TEST(FixedDistanceRiskPath, PlanContinuationRejectsMalformedPlan)
{
  const auto candidate = timed_curvature_suffix_trajectory(1.4, 0.0);
  nav_2d_msgs::msg::Path2D empty;
  EXPECT_THROW(
    f_dwa_controller::build_plan_continued_risk_path(
      candidate, empty, 2.5, 0.05, 1.4),
    std::invalid_argument);

  auto malformed = straight_path(3.0);
  malformed.poses.back().theta =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    f_dwa_controller::build_plan_continued_risk_path(
      candidate, malformed, 2.5, 0.05, 1.4),
    std::invalid_argument);
  EXPECT_THROW(
    f_dwa_controller::prepare_plan_continuation_geometry(empty),
    std::invalid_argument);
  EXPECT_THROW(
    f_dwa_controller::prepare_plan_continuation_geometry(malformed),
    std::invalid_argument);

  const f_dwa_controller::PreparedPlanGeometry unprepared;
  EXPECT_THROW(
    f_dwa_controller::build_plan_continued_risk_path(
      candidate, unprepared, 2.5, 0.05, 1.4),
    std::invalid_argument);
  const auto prepared =
    f_dwa_controller::prepare_plan_continuation_geometry(straight_path(3.0));
  EXPECT_THROW(
    f_dwa_controller::build_plan_continued_risk_path(
      candidate, prepared, 2.5, 0.05, 1.4, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    f_dwa_controller::build_plan_continued_risk_path(
      candidate, straight_path(3.0), 2.5, 0.05, 1.4,
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

TEST(ForwardObstacleCritic, FailedPrepareInvalidatesPreparedPlanGeometry)
{
  ReferenceBlockerForwardObstacleCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  const auto candidate = timed_curvature_suffix_trajectory(1.4, 0.0);
  ASSERT_TRUE(critic.prepare(
      pose, velocity, goal, default_reference_plan()));
  EXPECT_NO_THROW(critic.scoreTrajectory(candidate));

  auto malformed = default_reference_plan();
  malformed.poses.back().x = std::numeric_limits<double>::quiet_NaN();
  ASSERT_FALSE(critic.prepare(pose, velocity, goal, malformed));
  EXPECT_THROW(
    critic.scoreTrajectory(candidate),
    dwb_core::IllegalTrajectoryException);
}

TEST(FixedDistanceRiskPath, PlanContinuationHandlesPlanEndAndPureRotation)
{
  dwb_msgs::msg::Trajectory2D pure_rotation;
  geometry_msgs::msg::Pose2D start;
  start.x = 1.0;
  geometry_msgs::msg::Pose2D endpoint = start;
  endpoint.theta = M_PI_2;
  pure_rotation.poses = {start, endpoint, endpoint};
  pure_rotation.time_offsets = {duration(0.0), duration(1.4)};

  nav_2d_msgs::msg::Path2D goal_only_plan;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1.0;
  goal.theta = M_PI_2;
  goal_only_plan.poses = {goal};
  const auto risk_path = f_dwa_controller::build_plan_continued_risk_path(
    pure_rotation, goal_only_plan, 1.0, 0.1, 1.4);
  ASSERT_EQ(risk_path.size(), 11u);
  EXPECT_NEAR(risk_path.back().pose.x, 1.0, 1.0e-12);
  EXPECT_NEAR(risk_path.back().pose.y, 1.0, 1.0e-12);
  for (const auto & sample : risk_path) {
    EXPECT_TRUE(std::isfinite(sample.pose.x));
    EXPECT_TRUE(std::isfinite(sample.pose.y));
    EXPECT_TRUE(std::isfinite(sample.pose.theta));
  }

  dwb_msgs::msg::Trajectory2D beyond_plan_end;
  geometry_msgs::msg::Pose2D beyond_endpoint;
  beyond_endpoint.x = 1.0;
  beyond_plan_end.poses = {
    geometry_msgs::msg::Pose2D{}, beyond_endpoint, beyond_endpoint};
  beyond_plan_end.time_offsets = {duration(0.0), duration(1.4)};
  auto short_plan = straight_path(0.5);
  const auto beyond_path = f_dwa_controller::build_plan_continued_risk_path(
    beyond_plan_end, short_plan, 2.0, 0.1, 1.4);
  ASSERT_EQ(beyond_path.size(), 21u);
  EXPECT_NEAR(beyond_path.back().pose.x, 2.0, 1.0e-12);
  for (std::size_t index = 1u; index < beyond_path.size(); ++index) {
    EXPECT_GE(
      beyond_path[index].pose.x + 1.0e-12,
      beyond_path[index - 1u].pose.x);
  }
}

TEST(ForwardObstacleCritic, FixedTimedPrefixIgnoresCurvatureSuffix)
{
  AnalyticForwardObstacleCritic critic;
  const double short_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.4, 0.0));
  const double canonical_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.6, -1.2));
  const double long_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.8, 1.2));

  EXPECT_GT(short_score, 0.0);
  EXPECT_LE(short_score, 1.0);
  EXPECT_NEAR(canonical_score, short_score, 1.0e-12);
  EXPECT_NEAR(long_score, short_score, 1.0e-12);
}

TEST(FootprintClearanceCritic, FixedTimedPrefixIgnoresCurvatureSuffix)
{
  StubFootprintClearanceCritic critic;
  critic.setRiskPath(2.5, 25);
  critic.setRiskSeedTime(1.4);
  critic.setFirstCollidingBand(0u);
  critic.setMinimumCollisionX(0.5);
  critic.setMaximumCollisionAbsY(1.0);

  const double short_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.4, 0.0));
  const double canonical_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.6, -1.2));
  const double long_score = critic.scoreTrajectory(
    timed_curvature_suffix_trajectory(1.8, 1.2));

  EXPECT_GT(short_score, 0.0);
  EXPECT_LE(short_score, 1.0);
  EXPECT_NEAR(canonical_score, short_score, 1.0e-12);
  EXPECT_NEAR(long_score, short_score, 1.0e-12);
}

TEST(FixedDistanceRiskPath, TimedPrefixHandlesDegenerateSpatialMotion)
{
  dwb_msgs::msg::Trajectory2D pure_rotation;
  geometry_msgs::msg::Pose2D initial_pose;
  initial_pose.x = 1.0;
  initial_pose.y = 2.0;
  geometry_msgs::msg::Pose2D rotated_pose = initial_pose;
  rotated_pose.theta = M_PI_2;
  pure_rotation.poses = {initial_pose, rotated_pose, rotated_pose};
  pure_rotation.time_offsets = {duration(0.0), duration(1.4)};
  const auto rotation_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    pure_rotation, 2.5, 0.1, 1.4);
  ASSERT_EQ(rotation_path.size(), 26u);
  EXPECT_NEAR(rotation_path.back().pose.x, 3.5, 1.0e-12);
  EXPECT_NEAR(rotation_path.back().pose.y, 2.0, 1.0e-12);

  dwb_msgs::msg::Trajectory2D near_zero_translation;
  geometry_msgs::msg::Pose2D near_zero_end;
  near_zero_end.x = 1.0e-8;
  near_zero_end.theta = M_PI;
  near_zero_translation.poses = {
    geometry_msgs::msg::Pose2D{}, near_zero_end, near_zero_end};
  near_zero_translation.time_offsets = {duration(0.0), duration(1.4)};
  const auto near_zero_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    near_zero_translation, 2.5, 0.1, 1.4);
  ASSERT_EQ(near_zero_path.size(), 26u);
  for (const auto & sample : near_zero_path) {
    EXPECT_TRUE(std::isfinite(sample.pose.x));
    EXPECT_TRUE(std::isfinite(sample.pose.y));
    EXPECT_TRUE(std::isfinite(sample.pose.theta));
    EXPECT_LT(std::hypot(sample.pose.x, sample.pose.y), 1.0e-6);
  }

  dwb_msgs::msg::Trajectory2D stop_then_move;
  geometry_msgs::msg::Pose2D stopped_pose;
  geometry_msgs::msg::Pose2D moving_pose;
  moving_pose.x = 0.45;
  moving_pose.y = 0.10;
  moving_pose.theta = 0.20;
  stop_then_move.poses = {
    stopped_pose, stopped_pose, moving_pose, moving_pose};
  stop_then_move.time_offsets = {
    duration(0.0), duration(0.5), duration(1.4)};
  const auto stop_then_move_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    stop_then_move, 2.5, 0.1, 1.4);
  ASSERT_EQ(stop_then_move_path.size(), 26u);
  EXPECT_NEAR(stop_then_move_path.back().arc_length, 2.5, 1.0e-12);
  for (const auto & sample : stop_then_move_path) {
    EXPECT_TRUE(std::isfinite(sample.pose.x));
    EXPECT_TRUE(std::isfinite(sample.pose.y));
    EXPECT_TRUE(std::isfinite(sample.pose.theta));
  }
}

TEST(FixedDistanceRiskPath, TimedPrefixRejectsUnsafeLayouts)
{
  dwb_msgs::msg::Trajectory2D no_timing;
  no_timing.poses.resize(2u);
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(
      no_timing, 2.5, 0.1, 1.4),
    std::invalid_argument);

  dwb_msgs::msg::Trajectory2D unsupported_layout;
  unsupported_layout.poses.resize(3u);
  unsupported_layout.time_offsets = {duration(0.0)};
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(
      unsupported_layout, 2.5, 0.1, 1.4),
    std::invalid_argument);

  dwb_msgs::msg::Trajectory2D non_monotonic;
  non_monotonic.poses.resize(3u);
  non_monotonic.time_offsets = {duration(0.0), duration(0.0)};
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(
      non_monotonic, 2.5, 0.1, 1.4),
    std::invalid_argument);

  dwb_msgs::msg::Trajectory2D too_short;
  too_short.poses.resize(3u);
  too_short.time_offsets = {duration(0.0), duration(1.0)};
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(
      too_short, 2.5, 0.1, 1.4),
    std::invalid_argument);
}

TEST(FixedDistanceRiskPath, TimedPrefixInterpolatesTheSeedPose)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  geometry_msgs::msg::Pose2D endpoint;
  endpoint.x = 1.5;
  trajectory.poses = {geometry_msgs::msg::Pose2D{}, endpoint, endpoint};
  trajectory.time_offsets = {duration(0.0), duration(1.5)};

  const auto risk_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    trajectory, 2.5, 0.1, 1.4);
  ASSERT_EQ(risk_path.size(), 26u);
  ASSERT_GT(risk_path.size(), 14u);
  EXPECT_NEAR(risk_path[14].arc_length, 1.4, 1.0e-12);
  EXPECT_NEAR(risk_path[14].pose.x, 1.4, 1.0e-12);
  EXPECT_NEAR(risk_path.back().arc_length, 2.5, 1.0e-12);
  EXPECT_NEAR(risk_path.back().pose.x, 2.5, 1.0e-12);
}

TEST(FixedDistanceRiskPath, RejectsEmptyAndNonFiniteInput)
{
  dwb_msgs::msg::Trajectory2D empty;
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(empty, 1.0, 0.1),
    std::invalid_argument);

  dwb_msgs::msg::Trajectory2D non_finite;
  non_finite.poses.resize(1);
  non_finite.poses.front().theta =
    std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    f_dwa_controller::build_fixed_distance_risk_path(
      non_finite, 1.0, 0.1),
    std::invalid_argument);
}

TEST(FixedDistanceRiskPath, ContinuesTerminalCurvatureAcrossExtension)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2);
  trajectory.poses.back().x = 0.5;
  trajectory.poses.back().theta = 0.25;

  const auto risk_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    trajectory, 1.0, 0.25);
  ASSERT_EQ(risk_path.size(), 5u);
  EXPECT_NEAR(risk_path[2].arc_length, 0.5, 1.0e-12);
  EXPECT_NEAR(risk_path[2].pose.x, 0.5, 1.0e-12);
  EXPECT_NEAR(risk_path[2].pose.y, 0.0, 1.0e-12);
  EXPECT_NEAR(risk_path[2].pose.theta, 0.25, 1.0e-12);
  EXPECT_NEAR(
    risk_path[3].pose.x,
    0.5 + (std::sin(0.375) - std::sin(0.25)) / 0.5,
    1.0e-12);
  EXPECT_NEAR(
    risk_path[3].pose.y,
    (std::cos(0.25) - std::cos(0.375)) / 0.5,
    1.0e-12);
  EXPECT_NEAR(risk_path[3].pose.theta, 0.375, 1.0e-12);
}

TEST(FixedDistanceRiskPath, PureRotationUsesInitialStraightProbe)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2);
  trajectory.poses.front().x = 1.0;
  trajectory.poses.front().y = 2.0;
  trajectory.poses.back().x = 1.0;
  trajectory.poses.back().y = 2.0;
  trajectory.poses.front().theta = 0.0;
  trajectory.poses.back().theta = M_PI_2;

  const auto risk_path =
    f_dwa_controller::build_fixed_distance_risk_path(
    trajectory, 0.5, 0.25);
  ASSERT_EQ(risk_path.size(), 3u);
  for (const auto & sample : risk_path) {
    EXPECT_NEAR(sample.pose.theta, 0.0, 1.0e-12);
  }
  EXPECT_NEAR(risk_path.back().pose.x, 1.5, 1.0e-12);
  EXPECT_NEAR(risk_path.back().pose.y, 2.0, 1.0e-12);
}

TEST(FixedDistanceRiskPath, StationaryCandidateUsesInitialHeading)
{
  dwb_msgs::msg::Trajectory2D stationary;
  stationary.poses.resize(3);
  for (auto & pose : stationary.poses) {
    pose.x = 1.0;
    pose.y = 2.0;
    pose.theta = M_PI_2;
  }
  const auto risk_path =
    f_dwa_controller::build_fixed_distance_risk_path(stationary, 1.0, 0.5);
  ASSERT_EQ(risk_path.size(), 3u);
  EXPECT_NEAR(risk_path.back().pose.x, 1.0, 1.0e-12);
  EXPECT_NEAR(risk_path.back().pose.y, 3.0, 1.0e-12);
  EXPECT_NEAR(risk_path.back().pose.theta, M_PI_2, 1.0e-12);
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

TEST(PathSubgoalDistCritic, ScoresNominalEndpointAgainstPathLookahead)
{
  f_dwa_controller::PathSubgoalDistCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, straight_path(4.0)));

  EXPECT_NEAR(critic.scoreTrajectory(trajectory_to(1.0, 1.0)), 0.5, 1.0e-9);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory_to(1.5, 1.0)), 0.0, 1.0e-9);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory_to(2.0, 1.0)), 0.5, 1.0e-9);
}

TEST(PathSubgoalDistCritic, AllowsForwardOvershootButRetainsLateralError)
{
  StubPathSubgoalDistCritic critic;
  critic.allowForwardOvershoot();
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, straight_path(4.0)));

  auto trajectory = trajectory_to(2.0, 1.0);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.0, 1.0e-9);
  trajectory.poses.back().y = 0.4;
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.4, 1.0e-9);
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

TEST(TerminalApproachCritic, IsStrictlyNeutralOutsideOuterDistance)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;

  pose.x = -1.5;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(fixed_step_terminal_trajectory(1.4, 0.2, -0.3)),
    0.0);

  pose.x = -1.501;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  EXPECT_DOUBLE_EQ(
    critic.scoreTrajectory(fixed_step_terminal_trajectory(1.4, 0.2, -0.3)),
    0.0);
}

TEST(TerminalApproachCritic, UsesSmoothTransitionAndFullWeightWindow)
{
  StubTerminalApproachCritic critic;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  const auto trajectory = fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  const double full_cost = std::abs(-0.3 + 0.2 * 0.2 / (2.0 * 0.15));

  geometry_msgs::msg::Pose2D pose;
  pose.x = -0.65;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), full_cost, 1.0e-12);

  pose.x = -1.075;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.5 * full_cost, 1.0e-12);
}

TEST(TerminalApproachCritic, FixedEvaluationPrefixIsHorizonInvariant)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  pose.x = -0.65;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  const auto short_horizon =
    fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  const auto canonical_horizon =
    fixed_step_terminal_trajectory(1.6, 0.2, -0.3, 0.0, 0.6);
  const auto long_horizon =
    fixed_step_terminal_trajectory(1.8, 0.2, -0.3, 0.0, 0.05);
  const double score = critic.scoreTrajectory(short_horizon);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(canonical_horizon), score);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(long_horizon), score);
}

TEST(TerminalApproachCritic, UsesNav2IncludeLastTimeOffsetConvention)
{
  StubTerminalApproachCritic critic;
  const auto trajectory =
    fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  ASSERT_EQ(trajectory.time_offsets.size(), 29u);
  ASSERT_EQ(trajectory.poses.size(), 30u);
  EXPECT_NEAR(trajectory.poses[28].x, -0.3, 1.0e-12);
  EXPECT_DOUBLE_EQ(trajectory.poses[29].x, trajectory.poses[28].x);

  geometry_msgs::msg::Pose2D evaluation_pose;
  double evaluation_speed = 0.0;
  ASSERT_TRUE(critic.sampleEvaluationState(
      trajectory, evaluation_pose, evaluation_speed));
  EXPECT_NEAR(evaluation_pose.x, -0.3, 1.0e-12);
  EXPECT_NEAR(evaluation_speed, 0.2, 1.0e-12);
}

TEST(TerminalApproachCritic, StoppingProjectionRespectsHeadingAfterGoalPassage)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  pose.x = 0.2;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  const auto moving_away =
    fixed_step_terminal_trajectory(1.4, 0.2, 0.2, 0.0);
  const auto moving_toward =
    fixed_step_terminal_trajectory(1.4, 0.2, 0.2, M_PI);
  EXPECT_LT(
    critic.scoreTrajectory(moving_toward),
    critic.scoreTrajectory(moving_away));
}

TEST(TerminalApproachCritic, PrefersSpeedThatStopsNearGoal)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  pose.x = -0.1;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  const auto slow = fixed_step_terminal_trajectory(1.4, 0.1, -0.1);
  const auto fast = fixed_step_terminal_trajectory(1.4, 0.3, -0.1);
  EXPECT_LT(critic.scoreTrajectory(slow), critic.scoreTrajectory(fast));
}

TEST(TerminalApproachCritic, UsesTimedPosesInsteadOfRequestedVelocity)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  pose.x = -0.65;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  auto trajectory = fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  const double timed_pose_score = critic.scoreTrajectory(trajectory);
  trajectory.velocity.x = 1.2;
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory), timed_pose_score);
}

TEST(TerminalApproachCritic, MalformedTrajectoryIsNeutral)
{
  StubTerminalApproachCritic critic;
  geometry_msgs::msg::Pose2D pose;
  pose.x = -0.65;
  nav_2d_msgs::msg::Twist2D velocity;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, goal, path));

  auto missing_offset = fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  missing_offset.time_offsets.pop_back();
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(missing_offset), 0.0);

  auto non_monotonic = fixed_step_terminal_trajectory(1.4, 0.2, -0.3);
  non_monotonic.time_offsets[5] = non_monotonic.time_offsets[4];
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(non_monotonic), 0.0);
}

TEST(TerminalApproachCritic, RejectsInvalidParameters)
{
  StubTerminalApproachCritic critic;
  EXPECT_THROW(critic.setParameters(0.0, 1.5, 0.65, 0.15), std::invalid_argument);
  EXPECT_THROW(critic.setParameters(1.4, 0.65, 0.65, 0.15), std::invalid_argument);
  EXPECT_THROW(critic.setParameters(1.4, 1.5, 0.65, 0.0), std::invalid_argument);
}
