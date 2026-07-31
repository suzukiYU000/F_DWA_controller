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
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "f_dwa_controller/certified_dwb_local_planner.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr char kPluginName[] = "FollowPath";

nav2_util::LifecycleNode::SharedPtr make_node(
  const std::string & name,
  const bool coefficients_generated = true,
  const bool require_applied_command_state = false,
  const bool prefer_previous_selected_candidate = false,
  const double fir_prediction_pulse_duration = 0.0)
{
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("FollowPath.min_vel_x", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_x", 1.2),
    rclcpp::Parameter("FollowPath.min_vel_y", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_y", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_theta", 1.57),
    rclcpp::Parameter("FollowPath.min_speed_xy", 0.0),
    rclcpp::Parameter("FollowPath.max_speed_xy", 1.2),
    rclcpp::Parameter("FollowPath.min_speed_theta", 0.0),
    rclcpp::Parameter("FollowPath.acc_lim_x", 1.2),
    rclcpp::Parameter("FollowPath.acc_lim_y", 0.0),
    rclcpp::Parameter("FollowPath.acc_lim_theta", 1.57),
    rclcpp::Parameter("FollowPath.decel_lim_x", -1.2),
    rclcpp::Parameter("FollowPath.decel_lim_y", 0.0),
    rclcpp::Parameter("FollowPath.decel_lim_theta", -1.57),
    rclcpp::Parameter("FollowPath.vx_samples", 11),
    rclcpp::Parameter("FollowPath.vy_samples", 1),
    rclcpp::Parameter("FollowPath.vtheta_samples", 15),
    rclcpp::Parameter("FollowPath.sim_time", 2.4),
    rclcpp::Parameter("FollowPath.discretize_by_time", true),
    rclcpp::Parameter("FollowPath.time_granularity", 0.03),
    rclcpp::Parameter("FollowPath.native_input_control_period", 0.03),
    rclcpp::Parameter("FollowPath.max_linear_raw_input", 1.2),
    rclcpp::Parameter("FollowPath.max_angular_raw_input", 1.57),
    rclcpp::Parameter(
      "FollowPath.fir_prediction_pulse_duration",
      fir_prediction_pulse_duration),
    rclcpp::Parameter(
      "FollowPath.require_applied_command_state",
      require_applied_command_state)};
  parameters.emplace_back(
    "FollowPath.fir_coefficients",
    std::vector<double>{0.5, 0.3, 0.2});
  parameters.emplace_back(
    "FollowPath.fir_coefficients_generated", coefficients_generated);
  if (prefer_previous_selected_candidate) {
    parameters.emplace_back(
      "FollowPath.prefer_previous_selected_candidate", true);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);
  return std::make_shared<nav2_util::LifecycleNode>(name, "", options);
}

void expect_finite_trajectory(
  NativeInputTrajectoryGenerator & generator)
{
  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  ASSERT_TRUE(generator.hasMoreTwists());

  const nav_2d_msgs::msg::Twist2D first_command = generator.nextTwist();
  geometry_msgs::msg::Pose2D start_pose;
  const dwb_msgs::msg::Trajectory2D trajectory =
    generator.generateTrajectory(
    start_pose, current_velocity, first_command);

  EXPECT_EQ(trajectory.poses.size(), 82u);
  EXPECT_NEAR(trajectory.velocity.x, first_command.x, 1.0e-12);
  EXPECT_NEAR(trajectory.velocity.theta, first_command.theta, 1.0e-12);
  for (const geometry_msgs::msg::Pose2D & pose : trajectory.poses) {
    EXPECT_TRUE(std::isfinite(pose.x));
    EXPECT_TRUE(std::isfinite(pose.y));
    EXPECT_TRUE(std::isfinite(pose.theta));
  }

  std::vector<geometry_msgs::msg::Pose2D> stop_poses;
  std::vector<nav_2d_msgs::msg::Twist2D> stop_velocities;
  ASSERT_TRUE(
    generator.generate_stop_trajectory(
      start_pose, 267, 0.01, stop_poses, stop_velocities));
  ASSERT_EQ(stop_poses.size(), stop_velocities.size() + 1u);
  ASSERT_FALSE(stop_velocities.empty());
  EXPECT_NEAR(stop_velocities.front().x, first_command.x, 1.0e-12);
  EXPECT_NEAR(
    stop_velocities.front().theta, first_command.theta, 1.0e-12);
  EXPECT_LE(std::abs(stop_velocities.back().x), 0.01);
  EXPECT_LE(std::abs(stop_velocities.back().theta), 0.01);

  std::size_t candidate_count = 1;
  while (generator.hasMoreTwists()) {
    generator.nextTwist();
    ++candidate_count;
  }
  EXPECT_EQ(candidate_count, 165u);
}

PlanningSnapshot make_observable_zero_snapshot(
  const rclcpp::Time & stamp)
{
  PlanningSnapshot snapshot;
  snapshot.measurement_time = stamp;
  snapshot.activation_time = stamp;
  snapshot.current_state.activation_time = stamp;
  snapshot.activation_state.activation_time = stamp;
  snapshot.current_state.native_state_valid = true;
  snapshot.activation_state.native_state_valid = true;
  snapshot.dispatch_state_observed = true;
  snapshot.valid = true;
  return snapshot;
}

geometry_msgs::msg::Pose2D legacy_compute_new_position(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double time_step)
{
  geometry_msgs::msg::Pose2D pose;
  pose.x = start_pose.x +
    (velocity.x * std::cos(start_pose.theta) +
    velocity.y * std::cos(M_PI_2 + start_pose.theta)) * time_step;
  pose.y = start_pose.y +
    (velocity.x * std::sin(start_pose.theta) +
    velocity.y * std::sin(M_PI_2 + start_pose.theta)) * time_step;
  pose.theta =
    start_pose.theta + velocity.theta * time_step;
  return pose;
}

class FixedScoreCritic final : public dwb_core::TrajectoryCritic
{
public:
  FixedScoreCritic(
    std::string name,
    const double scale,
    const double raw_score)
  : raw_score_(raw_score)
  {
    name_ = std::move(name);
    scale_ = scale;
  }

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & /*trajectory*/) override
  {
    ++call_count_;
    return raw_score_;
  }

  [[nodiscard]] int call_count() const
  {
    return call_count_;
  }

private:
  double raw_score_{0.0};
  int call_count_{0};
};

class ScorePlannerAdapter final : public CertifiedDWBLocalPlanner
{
public:
  void set_test_critics(
    std::vector<dwb_core::TrajectoryCritic::Ptr> critics,
    const bool short_circuit)
  {
    critics_ = std::move(critics);
    short_circuit_trajectory_evaluation_ = short_circuit;
  }

  dwb_msgs::msg::TrajectoryScore reference_score(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const double best_score)
  {
    return dwb_core::DWBLocalPlanner::scoreTrajectory(
      trajectory, best_score);
  }
};

void expect_scores_equal(
  const dwb_msgs::msg::TrajectoryScore & expected,
  const dwb_msgs::msg::TrajectoryScore & actual)
{
  EXPECT_DOUBLE_EQ(actual.total, expected.total);
  EXPECT_EQ(actual.traj, expected.traj);
  ASSERT_EQ(actual.scores.size(), expected.scores.size());
  for (std::size_t index = 0; index < expected.scores.size(); ++index) {
    EXPECT_EQ(actual.scores[index].name, expected.scores[index].name);
    EXPECT_DOUBLE_EQ(
      actual.scores[index].scale, expected.scores[index].scale);
    EXPECT_DOUBLE_EQ(
      actual.scores[index].raw_score, expected.scores[index].raw_score);
  }
}

}  // namespace

class NativeInputTrajectoryGeneratorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(NativeInputTrajectoryGeneratorTest, AccelerationGeneratorRollsOut165Candidates)
{
  const auto node = make_node("acceleration_generator_test");
  AccelerationTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

TEST_F(NativeInputTrajectoryGeneratorTest, JerkGeneratorRollsOut165Candidates)
{
  const auto node = make_node("jerk_generator_test");
  JerkTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirGeneratorRollsOut165Candidates)
{
  const auto node = make_node("fir_generator_test");
  FirTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirFinitePulseDoesNotApplyHeldHorizonVelocityClamp)
{
  const auto node =
    make_node("fir_finite_pulse_test", true, false, false, 0.15);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  double maximum_first_linear_velocity = 0.0;
  std::size_t candidate_count = 0u;
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    maximum_first_linear_velocity =
      std::max(maximum_first_linear_velocity, command.x);
    ++candidate_count;
  }

  EXPECT_EQ(candidate_count, 165u);
  EXPECT_NEAR(maximum_first_linear_velocity, 0.018, 1.0e-12);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ReusedFirTrajectoryStorageIsBitExactForAllCandidates)
{
  const auto reference_node = make_node("fir_trajectory_reference_test");
  const auto reused_node = make_node("fir_trajectory_reused_test");
  FirTrajectoryGenerator reference_generator;
  FirTrajectoryGenerator reused_generator;
  reference_generator.initialize(reference_node, kPluginName);
  reused_generator.initialize(reused_node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  reference_generator.startNewIteration(current_velocity);
  reused_generator.startNewIteration(current_velocity);
  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.31;
  start_pose.y = -0.17;
  start_pose.theta = 0.42;
  dwb_msgs::msg::Trajectory2D reused_trajectory;
  std::size_t candidate_count = 0u;
  while (reference_generator.hasMoreTwists()) {
    ASSERT_TRUE(reused_generator.hasMoreTwists());
    const auto reference_command = reference_generator.nextTwist();
    const auto reused_command = reused_generator.nextTwist();
    ASSERT_EQ(reference_command, reused_command);
    const auto reference_trajectory =
      reference_generator.generateTrajectory(
      start_pose, current_velocity, reference_command);
    reused_generator.generate_trajectory_into(
      start_pose, reused_command, reused_trajectory);
    EXPECT_EQ(reused_trajectory, reference_trajectory);
    ++candidate_count;
  }
  EXPECT_FALSE(reused_generator.hasMoreTwists());
  EXPECT_EQ(candidate_count, 165u);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ExactScorePathMatchesPinnedDwbScoring)
{
  auto first =
    std::make_shared<FixedScoreCritic>("FirstCritic", 2.0, 1.25);
  auto disabled =
    std::make_shared<FixedScoreCritic>("DisabledCritic", 0.0, 100.0);
  auto last =
    std::make_shared<FixedScoreCritic>("LastCritic", 0.5, 3.5);
  ScorePlannerAdapter planner;
  planner.set_test_critics({first, disabled, last}, true);

  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 0.31;
  trajectory.velocity.theta = -0.27;
  trajectory.poses.resize(3u);
  trajectory.poses[1u].x = 0.13;
  trajectory.poses[2u].y = -0.21;
  trajectory.time_offsets.push_back(
    rclcpp::Duration::from_seconds(0.0));
  trajectory.time_offsets.push_back(
    rclcpp::Duration::from_seconds(0.03));

  const dwb_msgs::msg::TrajectoryScore expected =
    planner.reference_score(trajectory, -1.0);
  const dwb_msgs::msg::TrajectoryScore actual =
    planner.scoreTrajectory(trajectory, -1.0);

  expect_scores_equal(expected, actual);
  EXPECT_EQ(first->call_count(), 2);
  EXPECT_EQ(disabled->call_count(), 0);
  EXPECT_EQ(last->call_count(), 2);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ExactScorePathPreservesPinnedDwbShortCircuit)
{
  auto first =
    std::make_shared<FixedScoreCritic>("FirstCritic", 1.0, 2.0);
  auto skipped =
    std::make_shared<FixedScoreCritic>("SkippedCritic", 1.0, 4.0);
  ScorePlannerAdapter planner;
  planner.set_test_critics({first, skipped}, true);
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);

  const dwb_msgs::msg::TrajectoryScore expected =
    planner.reference_score(trajectory, 1.0);
  const dwb_msgs::msg::TrajectoryScore actual =
    planner.scoreTrajectory(trajectory, 1.0);

  expect_scores_equal(expected, actual);
  ASSERT_EQ(actual.scores.size(), 1u);
  EXPECT_EQ(first->call_count(), 2);
  EXPECT_EQ(skipped->call_count(), 0);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  AngularPoseCacheMatchesLegacyReferenceForAllFirCandidates)
{
  constexpr int kRolloutStepCount = 80;
  constexpr int kLinearSampleCount = 11;
  constexpr int kAngularSampleCount = 15;
  constexpr double kTimeStep = 0.03;
  const std::vector<double> fir_coefficients{0.5, 0.3, 0.2};
  const std::vector<double> initial_history(
    fir_coefficients.size() - 1u, 0.0);
  const AxisState initial_state;

  AxisLimits linear_limits;
  linear_limits.velocity_min = 0.0;
  linear_limits.velocity_max = 1.2;
  linear_limits.acceleration_min = -1.2;
  linear_limits.acceleration_max = 1.2;
  linear_limits.native_input_min = -1.2;
  linear_limits.native_input_max = 1.2;
  AxisLimits angular_limits;
  angular_limits.velocity_min = -1.57;
  angular_limits.velocity_max = 1.57;
  angular_limits.acceleration_min = -1.57;
  angular_limits.acceleration_max = 1.57;
  angular_limits.native_input_min = -1.57;
  angular_limits.native_input_max = 1.57;

  const auto build_rollouts =
    [&fir_coefficients, &initial_history, kRolloutStepCount, kTimeStep](
    const AxisLimits & limits,
    const std::vector<double> & native_inputs)
    {
      std::vector<std::vector<AxisState>> rollouts;
      rollouts.reserve(native_inputs.size());
      for (const double native_input : native_inputs) {
        AxisState state;
        std::vector<double> history = initial_history;
        std::vector<AxisState> states;
        states.reserve(kRolloutStepCount);
        for (int step_index = 0;
          step_index < kRolloutStepCount; ++step_index)
        {
          if (!apply_projected_fir_step_in_place(
              state, limits, fir_coefficients, history,
              native_input, kTimeStep))
          {
            throw std::runtime_error(
                    "legacy FIR reference rollout became infeasible");
          }
          states.push_back(state);
        }
        rollouts.push_back(std::move(states));
      }
      return rollouts;
    };

  const FeasibleInterval linear_interval =
    held_fir_input_interval(
    initial_state, linear_limits, fir_coefficients, initial_history,
    kTimeStep, kRolloutStepCount);
  const FeasibleInterval angular_interval =
    held_fir_input_interval(
    initial_state, angular_limits, fir_coefficients, initial_history,
    kTimeStep, kRolloutStepCount);
  ASSERT_TRUE(linear_interval.feasible);
  ASSERT_TRUE(angular_interval.feasible);
  const std::vector<double> linear_inputs =
    uniform_samples(linear_interval, kLinearSampleCount);
  const std::vector<double> angular_inputs =
    uniform_samples(angular_interval, kAngularSampleCount);
  const auto linear_rollouts =
    build_rollouts(linear_limits, linear_inputs);
  const auto angular_rollouts =
    build_rollouts(angular_limits, angular_inputs);
  const HeldFirAffineResponse linear_affine_response =
    prepare_held_fir_affine_response(
    initial_state, linear_limits, fir_coefficients, initial_history,
    kTimeStep, kRolloutStepCount);
  const HeldFirAffineResponse angular_affine_response =
    prepare_held_fir_affine_response(
    initial_state, angular_limits, fir_coefficients, initial_history,
    kTimeStep, kRolloutStepCount);
  const auto build_affine_rollouts =
    [](
    const HeldFirAffineResponse & response,
    const AxisLimits & limits,
    const std::vector<double> & native_inputs)
    {
      std::vector<std::vector<AxisState>> rollouts;
      rollouts.reserve(native_inputs.size());
      for (const double native_input : native_inputs) {
        std::vector<AxisState> states;
        if (!sample_held_fir_affine_response(
            response, limits, native_input, states))
        {
          throw std::runtime_error(
                  "affine FIR reference rollout became infeasible");
        }
        rollouts.push_back(std::move(states));
      }
      return rollouts;
    };
  const auto linear_affine_rollouts =
    build_affine_rollouts(
    linear_affine_response, linear_limits, linear_inputs);
  const auto angular_affine_rollouts =
    build_affine_rollouts(
    angular_affine_response, angular_limits, angular_inputs);

  const auto node = make_node("fir_angular_pose_cache_test");
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);

  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.37;
  start_pose.y = -0.19;
  start_pose.theta = 0.41;
  std::size_t expected_canonical_index = 0u;
  std::size_t reference_best_index = 0u;
  std::size_t affine_best_index = 0u;
  double reference_best_score = std::numeric_limits<double>::infinity();
  double affine_best_score = std::numeric_limits<double>::infinity();
  while (generator.hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D command = generator.nextTwist();
    const auto canonical_index =
      generator.active_candidate_canonical_index();
    ASSERT_TRUE(canonical_index.has_value());
    ASSERT_EQ(*canonical_index, expected_canonical_index);
    const dwb_msgs::msg::Trajectory2D trajectory =
      generator.generateTrajectory(
      start_pose, current_velocity, command);
    ASSERT_EQ(trajectory.poses.size(), 82u);

    geometry_msgs::msg::Pose2D reference_pose = start_pose;
    geometry_msgs::msg::Pose2D exact_integration_reference = start_pose;
    EXPECT_DOUBLE_EQ(trajectory.poses.front().x, reference_pose.x);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().y, reference_pose.y);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().theta, reference_pose.theta);
    ASSERT_EQ(
      trajectory.time_offsets.size(),
      static_cast<std::size_t>(kRolloutStepCount) + 1u);
    const std::size_t linear_index =
      *canonical_index / static_cast<std::size_t>(kAngularSampleCount);
    const std::size_t angular_index =
      *canonical_index % static_cast<std::size_t>(kAngularSampleCount);
    double reference_running_time = 0.0;
    EXPECT_NEAR(
      command.x, linear_rollouts[linear_index].front().velocity, 1.0e-12);
    EXPECT_NEAR(
      command.theta,
      angular_rollouts[angular_index].front().velocity, 1.0e-12);
    for (int step_index = 0;
      step_index < kRolloutStepCount; ++step_index)
    {
      nav_2d_msgs::msg::Twist2D velocity;
      velocity.x =
        linear_rollouts[linear_index][step_index].velocity;
      velocity.theta =
        angular_rollouts[angular_index][step_index].velocity;
      reference_pose =
        legacy_compute_new_position(
        reference_pose, velocity, kTimeStep);
      nav_2d_msgs::msg::Twist2D affine_velocity;
      affine_velocity.x =
        linear_affine_rollouts[linear_index][step_index].velocity;
      affine_velocity.theta =
        angular_affine_rollouts[angular_index][step_index].velocity;
      exact_integration_reference =
        legacy_compute_new_position(
        exact_integration_reference, affine_velocity, kTimeStep);
      const auto & cached_pose =
        trajectory.poses[static_cast<std::size_t>(step_index) + 1u];
      EXPECT_NEAR(cached_pose.x, reference_pose.x, 1.0e-12);
      EXPECT_NEAR(cached_pose.y, reference_pose.y, 1.0e-12);
      EXPECT_NEAR(cached_pose.theta, reference_pose.theta, 1.0e-12);
      EXPECT_DOUBLE_EQ(cached_pose.x, exact_integration_reference.x);
      EXPECT_DOUBLE_EQ(cached_pose.y, exact_integration_reference.y);
      EXPECT_DOUBLE_EQ(
        cached_pose.theta, exact_integration_reference.theta);
      EXPECT_EQ(
        rclcpp::Duration(
          trajectory.time_offsets[
            static_cast<std::size_t>(step_index)]).nanoseconds(),
        rclcpp::Duration::from_seconds(
          reference_running_time).nanoseconds());
      reference_running_time += kTimeStep;
    }
    EXPECT_NEAR(trajectory.poses.back().x, reference_pose.x, 1.0e-12);
    EXPECT_NEAR(trajectory.poses.back().y, reference_pose.y, 1.0e-12);
    EXPECT_NEAR(
      trajectory.poses.back().theta, reference_pose.theta, 1.0e-12);
    EXPECT_EQ(
      rclcpp::Duration(trajectory.time_offsets.back()).nanoseconds(),
      rclcpp::Duration::from_seconds(
        reference_running_time).nanoseconds());

    const auto endpoint_score =
      [](const geometry_msgs::msg::Pose2D & pose)
      {
        constexpr double kTargetX = 1.73;
        constexpr double kTargetY = 0.28;
        constexpr double kTargetTheta = -0.31;
        return std::pow(pose.x - kTargetX, 2) +
               std::pow(pose.y - kTargetY, 2) +
               0.4 * std::pow(pose.theta - kTargetTheta, 2);
      };
    const double reference_score = endpoint_score(reference_pose);
    const double affine_score = endpoint_score(trajectory.poses.back());
    if (reference_score < reference_best_score) {
      reference_best_score = reference_score;
      reference_best_index = *canonical_index;
    }
    if (affine_score < affine_best_score) {
      affine_best_score = affine_score;
      affine_best_index = *canonical_index;
    }
    ++expected_canonical_index;
  }
  EXPECT_EQ(expected_canonical_index, 165u);
  EXPECT_EQ(affine_best_index, reference_best_index);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirCachedStopMaterializationPreservesTrajectory)
{
  const auto node = make_node("fir_cached_stop_test");
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  ASSERT_TRUE(generator.hasMoreTwists());
  generator.nextTwist();
  const auto canonical_index =
    generator.active_candidate_canonical_index();
  ASSERT_TRUE(canonical_index.has_value());
  const auto active_command_state =
    generator.active_candidate_command_state();
  ASSERT_TRUE(active_command_state.has_value());

  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.25;
  start_pose.y = -0.1;
  start_pose.theta = 0.2;
  std::vector<geometry_msgs::msg::Pose2D> lightweight_poses;
  std::vector<nav_2d_msgs::msg::Twist2D> lightweight_velocities;
  ASSERT_TRUE(
    generator.generate_stop_trajectory(
      start_pose, 267, 0.01, lightweight_poses,
      lightweight_velocities));
  ASSERT_EQ(lightweight_poses.size(), lightweight_velocities.size() + 1u);
  geometry_msgs::msg::Pose2D legacy_pose = start_pose;
  EXPECT_DOUBLE_EQ(lightweight_poses.front().x, legacy_pose.x);
  EXPECT_DOUBLE_EQ(lightweight_poses.front().y, legacy_pose.y);
  EXPECT_DOUBLE_EQ(lightweight_poses.front().theta, legacy_pose.theta);
  for (std::size_t index = 0;
    index < lightweight_velocities.size(); ++index)
  {
    legacy_pose =
      legacy_compute_new_position(
      legacy_pose, lightweight_velocities[index], 0.03);
    EXPECT_DOUBLE_EQ(lightweight_poses[index + 1u].x, legacy_pose.x);
    EXPECT_DOUBLE_EQ(lightweight_poses[index + 1u].y, legacy_pose.y);
    EXPECT_DOUBLE_EQ(
      lightweight_poses[index + 1u].theta, legacy_pose.theta);
  }

  ASSERT_TRUE(generator.hasMoreTwists());
  generator.nextTwist();

  std::vector<geometry_msgs::msg::Pose2D> materialized_poses;
  std::vector<nav_2d_msgs::msg::Twist2D> materialized_velocities;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
  materialized_states;
  ASSERT_TRUE(
    generator.generate_stop_trajectory_for_candidate(
      *canonical_index, start_pose, 267, 0.01, materialized_poses,
      materialized_velocities, &materialized_states));

  ASSERT_EQ(lightweight_poses.size(), materialized_poses.size());
  ASSERT_EQ(lightweight_velocities.size(), materialized_velocities.size());
  ASSERT_EQ(materialized_states.size(), materialized_velocities.size());
  ASSERT_FALSE(materialized_states.empty());
  EXPECT_EQ(
    materialized_states.front().linear_fir_history,
    active_command_state->linear_fir_history);
  EXPECT_EQ(
    materialized_states.front().angular_fir_history,
    active_command_state->angular_fir_history);
  EXPECT_DOUBLE_EQ(
    materialized_states.front().linear_state.velocity,
    active_command_state->linear_state.velocity);
  EXPECT_DOUBLE_EQ(
    materialized_states.front().angular_state.velocity,
    active_command_state->angular_state.velocity);
  for (std::size_t index = 0; index < materialized_poses.size(); ++index) {
    EXPECT_DOUBLE_EQ(lightweight_poses[index].x, materialized_poses[index].x);
    EXPECT_DOUBLE_EQ(lightweight_poses[index].y, materialized_poses[index].y);
    EXPECT_DOUBLE_EQ(
      lightweight_poses[index].theta, materialized_poses[index].theta);
  }
  for (std::size_t index = 0;
    index < materialized_velocities.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(
      lightweight_velocities[index].x,
      materialized_velocities[index].x);
    EXPECT_DOUBLE_EQ(
      lightweight_velocities[index].theta,
      materialized_velocities[index].theta);
    EXPECT_DOUBLE_EQ(
      materialized_states[index].command_velocity.x,
      materialized_velocities[index].x);
    EXPECT_DOUBLE_EQ(
      materialized_states[index].command_velocity.theta,
      materialized_velocities[index].theta);
    EXPECT_TRUE(materialized_states[index].valid);
  }
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirStopPoseCacheMatchesLegacyForAllCandidates)
{
  const auto node = make_node("fir_stop_pose_cache_test");
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = -0.43;
  start_pose.y = 0.27;
  start_pose.theta = -0.61;
  std::size_t candidate_count = 0u;
  while (generator.hasMoreTwists()) {
    generator.nextTwist();
    std::vector<geometry_msgs::msg::Pose2D> poses;
    std::vector<nav_2d_msgs::msg::Twist2D> velocities;
    ASSERT_TRUE(
      generator.generate_stop_trajectory(
        start_pose, 267, 0.01, poses, velocities));
    ASSERT_EQ(poses.size(), velocities.size() + 1u);
    std::vector<geometry_msgs::msg::Pose2D> pose_only_trajectory;
    ASSERT_TRUE(
      generator.generate_stop_poses(
        start_pose, 267, 0.01, pose_only_trajectory));
    ASSERT_EQ(pose_only_trajectory.size(), poses.size());
    for (std::size_t pose_index = 0u;
      pose_index < poses.size(); ++pose_index)
    {
      EXPECT_DOUBLE_EQ(
        pose_only_trajectory[pose_index].x, poses[pose_index].x);
      EXPECT_DOUBLE_EQ(
        pose_only_trajectory[pose_index].y, poses[pose_index].y);
      EXPECT_DOUBLE_EQ(
        pose_only_trajectory[pose_index].theta, poses[pose_index].theta);
    }

    geometry_msgs::msg::Pose2D legacy_pose = start_pose;
    EXPECT_DOUBLE_EQ(poses.front().x, legacy_pose.x);
    EXPECT_DOUBLE_EQ(poses.front().y, legacy_pose.y);
    EXPECT_DOUBLE_EQ(poses.front().theta, legacy_pose.theta);
    for (std::size_t step_index = 0;
      step_index < velocities.size(); ++step_index)
    {
      legacy_pose =
        legacy_compute_new_position(
        legacy_pose, velocities[step_index], 0.03);
      EXPECT_DOUBLE_EQ(poses[step_index + 1u].x, legacy_pose.x);
      EXPECT_DOUBLE_EQ(poses[step_index + 1u].y, legacy_pose.y);
      EXPECT_DOUBLE_EQ(
        poses[step_index + 1u].theta, legacy_pose.theta);
    }
    ++candidate_count;
  }
  EXPECT_EQ(candidate_count, 165u);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  PreviousSelectionKeepsCanonicalOrderByDefault)
{
  const auto node = make_node("default_candidate_order_test");
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);

  constexpr std::size_t kSelectedCanonicalIndex = 60u;
  for (std::size_t index = 0u; index <= kSelectedCanonicalIndex; ++index) {
    generator.nextTwist();
  }
  ASSERT_EQ(
    generator.active_candidate_canonical_index(),
    kSelectedCanonicalIndex);
  generator.select_command_for_dispatch(
    generator.active_candidate_command_state());

  generator.startNewIteration(current_velocity);
  generator.nextTwist();

  EXPECT_EQ(generator.active_candidate_canonical_index(), 0u);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  PreviousSelectionIsEvaluatedFirstWhenEnabled)
{
  const auto node = make_node("warm_start_order_test", true, false, true);
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);

  constexpr std::size_t kSelectedCanonicalIndex = 60u;
  nav_2d_msgs::msg::Twist2D selected_velocity;
  for (std::size_t index = 0u; index <= kSelectedCanonicalIndex; ++index) {
    selected_velocity = generator.nextTwist();
  }
  ASSERT_EQ(
    generator.active_candidate_canonical_index(),
    kSelectedCanonicalIndex);
  generator.select_command_for_dispatch(
    generator.active_candidate_command_state());

  generator.startNewIteration(current_velocity);
  const nav_2d_msgs::msg::Twist2D first_velocity = generator.nextTwist();

  EXPECT_EQ(
    generator.active_candidate_canonical_index(),
    kSelectedCanonicalIndex);
  EXPECT_DOUBLE_EQ(first_velocity.x, selected_velocity.x);
  EXPECT_DOUBLE_EQ(first_velocity.theta, selected_velocity.theta);
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirGeneratorRejectsUngeneratedCoefficients)
{
  const auto node = make_node("ungenerated_fir_test", false);
  FirTrajectoryGenerator generator;

  EXPECT_THROW(generator.initialize(node, kPluginName), std::invalid_argument);
}

TEST_F(NativeInputTrajectoryGeneratorTest, TrialResetRetainsFirDesign)
{
  const auto node = make_node("fir_trial_reset_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  EXPECT_FALSE(generator.hasMoreTwists());

  generator.reset_trial_state();

  expect_finite_trajectory(generator);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  PlanningSnapshotMakesFirCandidateIndependentOfOdomVelocity)
{
  const auto node = make_node("fir_snapshot_source_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));

  nav_2d_msgs::msg::Twist2D first_odom;
  first_odom.x = 0.1;
  generator.startNewIteration(first_odom);
  ASSERT_TRUE(generator.hasMoreTwists());
  const nav_2d_msgs::msg::Twist2D first_command =
    generator.nextTwist();

  nav_2d_msgs::msg::Twist2D second_odom;
  second_odom.x = 1.0;
  second_odom.theta = 0.5;
  generator.startNewIteration(second_odom);
  ASSERT_TRUE(generator.hasMoreTwists());
  const nav_2d_msgs::msg::Twist2D second_command =
    generator.nextTwist();

  EXPECT_DOUBLE_EQ(first_command.x, second_command.x);
  EXPECT_DOUBLE_EQ(first_command.theta, second_command.theta);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirStateAdvancesFromSelectedCommandMetadata)
{
  const auto node = make_node("fir_native_ledger_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  nav_2d_msgs::msg::Twist2D odom_velocity;
  generator.startNewIteration(odom_velocity);
  ASSERT_TRUE(generator.hasMoreTwists());
  const nav_2d_msgs::msg::Twist2D command = generator.nextTwist();
  const auto selected_state =
    generator.active_candidate_command_state();
  ASSERT_TRUE(selected_state.has_value());
  generator.select_command_for_dispatch(selected_state);
  generator.commit_selected_command(command, node->now());

  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = command.x;
  dispatch.command.angular.z = command.theta;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot after_dispatch =
    make_observable_zero_snapshot(node->now());
  after_dispatch.current_state.velocity = command;
  after_dispatch.activation_state.velocity = command;
  generator.enrich_planning_snapshot(after_dispatch);
  EXPECT_TRUE(after_dispatch.valid);
  EXPECT_TRUE(after_dispatch.current_state.native_state_valid);
  EXPECT_NEAR(
    after_dispatch.current_state.linear_acceleration,
    selected_state->linear_state.acceleration, 1.0e-12);
  EXPECT_EQ(
    after_dispatch.current_state.linear_fir_history,
    selected_state->linear_fir_history);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirStateRejectsUncorrelatedNonzeroDispatch)
{
  const auto node = make_node("fir_uncorrelated_dispatch_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = 0.2;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(snapshot.current_state.native_state_valid);
}

}  // namespace f_dwa_controller
