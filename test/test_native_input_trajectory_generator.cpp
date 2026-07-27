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
  const bool load_f8_config = false)
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
    rclcpp::Parameter("FollowPath.require_applied_command_state", false)};
  if (!load_f8_config) {
    parameters.emplace_back(
      "FollowPath.fir_coefficients",
      std::vector<double>{0.5, 0.3, 0.2});
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);
  if (load_f8_config) {
    options.arguments(
        {
          "--ros-args", "--params-file",
          std::string(F_DWA_CONTROLLER_SOURCE_DIR) + "/config/f_dwa.yaml"
      });
  }
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

TEST_F(NativeInputTrajectoryGeneratorTest, NamedF8ConfigRollsOutAndStops)
{
  const auto node = make_node("controller_server", true);
  FirTrajectoryGenerator generator;

  generator.initialize(node, kPluginName);

  expect_finite_trajectory(generator);
}

}  // namespace f_dwa_controller
