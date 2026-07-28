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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
  const bool require_applied_command_state = false)
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
    rclcpp::Parameter("FollowPath.vtheta_samples", 11),
    rclcpp::Parameter("FollowPath.sim_time", 2.4),
    rclcpp::Parameter("FollowPath.discretize_by_time", true),
    rclcpp::Parameter("FollowPath.time_granularity", 0.03),
    rclcpp::Parameter("FollowPath.native_input_control_period", 0.03),
    rclcpp::Parameter("FollowPath.max_linear_raw_input", 1.2),
    rclcpp::Parameter("FollowPath.max_angular_raw_input", 1.57),
    rclcpp::Parameter(
      "FollowPath.require_applied_command_state",
      require_applied_command_state)};
  parameters.emplace_back(
    "FollowPath.fir_coefficients",
    std::vector<double>{0.5, 0.3, 0.2});
  parameters.emplace_back(
    "FollowPath.fir_coefficients_generated", coefficients_generated);
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
  EXPECT_EQ(candidate_count, 121u);
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

TEST_F(NativeInputTrajectoryGeneratorTest, AccelerationGeneratorRollsOut121Candidates)
{
  const auto node = make_node("acceleration_generator_test");
  AccelerationTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

TEST_F(NativeInputTrajectoryGeneratorTest, JerkGeneratorRollsOut121Candidates)
{
  const auto node = make_node("jerk_generator_test");
  JerkTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirGeneratorRollsOut121Candidates)
{
  const auto node = make_node("fir_generator_test");
  FirTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
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
