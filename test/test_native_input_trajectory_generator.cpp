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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "f_dwa_controller/certified_dwb_local_planner.hpp"
#include "f_dwa_controller/equal_effect_fir_sampling.hpp"
#include "f_dwa_controller/mean_speed_critic.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/v_dwb_trajectory_generators.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr char kPluginName[] = "FollowPath";

TEST(PlanningSnapshotTime, ClampsObservedSubcycleFutureSkewToZeroAge)
{
  const rclcpp::Time measurement_time(1000000000LL, RCL_ROS_TIME);
  const rclcpp::Time observed_dispatch_time(
    1004000000LL, RCL_ROS_TIME);

  const auto age = observed_dispatch_age_seconds(
    measurement_time, observed_dispatch_time, 0.05);

  ASSERT_TRUE(age.has_value());
  EXPECT_DOUBLE_EQ(*age, 0.0);
}

TEST(PlanningSnapshotTime, RejectsObservedStampBeyondOneControlPeriod)
{
  const rclcpp::Time measurement_time(1000000000LL, RCL_ROS_TIME);
  const rclcpp::Time observed_dispatch_time(
    1051000000LL, RCL_ROS_TIME);

  EXPECT_FALSE(observed_dispatch_age_seconds(
      measurement_time, observed_dispatch_time, 0.05).has_value());
}

PlanningSnapshot make_observable_zero_snapshot(
  const rclcpp::Time & stamp);
geometry_msgs::msg::Pose2D legacy_compute_new_position(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  double time_step);

nav2_util::LifecycleNode::SharedPtr make_node(
  const std::string & name,
  const bool coefficients_generated = true,
  const bool require_applied_command_state = false,
  const bool prefer_previous_selected_candidate = false,
  const double fir_prediction_pulse_duration = 0.0,
  const double maximum_linear_velocity = 1.2,
  const double maximum_angular_velocity = 1.57,
  const double control_period = 0.03)
{
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("FollowPath.min_vel_x", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_x", maximum_linear_velocity),
    rclcpp::Parameter("FollowPath.min_vel_y", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_y", 0.0),
    rclcpp::Parameter("FollowPath.max_vel_theta", maximum_angular_velocity),
    rclcpp::Parameter("FollowPath.min_speed_xy", 0.0),
    rclcpp::Parameter("FollowPath.max_speed_xy", maximum_linear_velocity),
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
    rclcpp::Parameter("FollowPath.time_granularity", control_period),
    rclcpp::Parameter("FollowPath.native_input_control_period", control_period),
    rclcpp::Parameter("FollowPath.max_linear_jerk", 1.57),
    rclcpp::Parameter("FollowPath.max_angular_jerk", 1.57),
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

void expect_direct_stop_trajectory(
  NativeInputTrajectoryGenerator & generator,
  const nav2_util::LifecycleNode::SharedPtr & node,
  const bool check_jerk_transition,
  const bool check_fir_state)
{
  constexpr double kControlPeriod = 0.03;
  constexpr double kMaximumLinearJerk = 1.57;
  constexpr double kMaximumAngularJerk = 1.57;
  PlanningSnapshot snapshot = make_observable_zero_snapshot(node->now());
  snapshot.current_state.velocity.x = 0.35;
  snapshot.current_state.velocity.theta = -0.12;
  snapshot.current_state.linear_acceleration = check_fir_state ? 0.0 : 0.30;
  snapshot.current_state.angular_acceleration = check_fir_state ? 0.0 : -0.40;
  if (check_fir_state) {
    snapshot.current_state.linear_fir_history.assign(2u, 0.0);
    snapshot.current_state.angular_fir_history.assign(2u, 0.0);
  }
  snapshot.activation_state = snapshot.current_state;
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.current_state.velocity);

  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.4;
  start_pose.y = -0.2;
  start_pose.theta = 0.3;
  std::vector<geometry_msgs::msg::Pose2D> poses;
  std::vector<nav_2d_msgs::msg::Twist2D> velocities;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState> states;
  ASSERT_TRUE(generator.generate_direct_stop_trajectory(
      start_pose, 267, 0.01, poses, velocities, states));
  ASSERT_EQ(states.size(), velocities.size());
  ASSERT_EQ(poses.size(), velocities.size() + 1u);
  ASSERT_GE(states.size(), 2u);

  geometry_msgs::msg::Pose2D integrated_pose = start_pose;
  for (std::size_t index = 0u; index < states.size(); ++index) {
    const auto & state = states[index];
    EXPECT_TRUE(state.valid);
    EXPECT_EQ(state.command_velocity, velocities[index]);
    EXPECT_LE(std::abs(state.command_velocity.x), 0.6 + 1.0e-12);
    EXPECT_LE(std::abs(state.command_velocity.theta), 0.6 + 1.0e-12);
    EXPECT_LE(std::abs(state.linear_state.acceleration), 1.2 + 1.0e-12);
    EXPECT_LE(std::abs(state.angular_state.acceleration), 1.57 + 1.0e-12);
    if (check_fir_state) {
      EXPECT_EQ(state.linear_fir_history.size(), 2u);
      EXPECT_EQ(state.angular_fir_history.size(), 2u);
    }
    integrated_pose = legacy_compute_new_position(
      integrated_pose, velocities[index], kControlPeriod);
    EXPECT_DOUBLE_EQ(poses[index + 1u].x, integrated_pose.x);
    EXPECT_DOUBLE_EQ(poses[index + 1u].y, integrated_pose.y);
    EXPECT_DOUBLE_EQ(poses[index + 1u].theta, integrated_pose.theta);
  }
  if (check_jerk_transition) {
    EXPECT_LE(
      std::abs(states.front().linear_state.acceleration - 0.30),
      kMaximumLinearJerk * kControlPeriod + 1.0e-12);
    EXPECT_LE(
      std::abs(states.front().angular_state.acceleration + 0.40),
      kMaximumAngularJerk * kControlPeriod + 1.0e-12);
  }
  EXPECT_DOUBLE_EQ(states.back().command_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(states.back().command_velocity.theta, 0.0);
  EXPECT_DOUBLE_EQ(states.back().linear_state.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(states.back().angular_state.acceleration, 0.0);
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

std::vector<double> trajectory_signature(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  std::vector<double> signature;
  signature.reserve(3u + trajectory.poses.size() * 3u);
  signature.push_back(trajectory.velocity.x);
  signature.push_back(trajectory.velocity.y);
  signature.push_back(trajectory.velocity.theta);
  for (const auto & pose : trajectory.poses) {
    signature.push_back(pose.x);
    signature.push_back(pose.y);
    signature.push_back(pose.theta);
  }
  return signature;
}

void expect_axis_state_within_limits(
  const AxisState & state,
  const AxisLimits & limits)
{
  constexpr double kTolerance = 1.0e-10;
  EXPECT_GE(state.velocity, limits.velocity_min - kTolerance);
  EXPECT_LE(state.velocity, limits.velocity_max + kTolerance);
  EXPECT_GE(state.acceleration, limits.acceleration_min - kTolerance);
  EXPECT_LE(state.acceleration, limits.acceleration_max + kTolerance);
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

class EndpointScoreCritic final : public dwb_core::TrajectoryCritic
{
public:
  EndpointScoreCritic(
    std::string name,
    const double scale,
    const geometry_msgs::msg::Pose2D & target,
    std::vector<std::string> & call_trace)
  : target_(target), call_trace_(call_trace)
  {
    name_ = std::move(name);
    scale_ = scale;
  }

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    call_trace_.push_back(name_);
    ++call_count_;
    if (trajectory.poses.empty()) {
      return 1.0e6;
    }
    const auto & endpoint = trajectory.poses.back();
    return 0.01 + std::pow(endpoint.x - target_.x, 2) +
           0.7 * std::pow(endpoint.y - target_.y, 2) +
           0.3 * std::pow(endpoint.theta - target_.theta, 2);
  }

  [[nodiscard]] std::size_t call_count() const
  {
    return call_count_;
  }

private:
  geometry_msgs::msg::Pose2D target_;
  std::vector<std::string> & call_trace_;
  std::size_t call_count_{0u};
};

class ScriptedVelocityGenerator final : public dwb_core::TrajectoryGenerator
{
public:
  explicit ScriptedVelocityGenerator(std::vector<double> velocities)
  : velocities_(std::move(velocities)) {}

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & /*node*/,
    const std::string & /*plugin_name*/) override
  {}

  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & /*current_velocity*/) override
  {
    index_ = 0u;
    generated_velocities_.clear();
  }

  bool hasMoreTwists() override
  {
    return index_ < velocities_.size();
  }

  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    nav_2d_msgs::msg::Twist2D twist;
    twist.x = velocities_.at(index_++);
    return twist;
  }

  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & /*start_velocity*/,
    const nav_2d_msgs::msg::Twist2D & command_velocity) override
  {
    generated_velocities_.push_back(command_velocity.x);
    dwb_msgs::msg::Trajectory2D trajectory;
    trajectory.velocity = command_velocity;
    trajectory.poses.push_back(start_pose);
    geometry_msgs::msg::Pose2D endpoint = start_pose;
    endpoint.x += command_velocity.x;
    trajectory.poses.push_back(endpoint);
    trajectory.time_offsets.push_back(
      rclcpp::Duration::from_seconds(0.0));
    return trajectory;
  }

  void setSpeedLimit(
    const double & /*speed_limit*/,
    const bool & /*percentage*/) override
  {}

  [[nodiscard]] const std::vector<double> & generated_velocities() const
  {
    return generated_velocities_;
  }

private:
  std::vector<double> velocities_;
  std::vector<double> generated_velocities_;
  std::size_t index_{0u};
};

class ScriptedScoreCritic final : public dwb_core::TrajectoryCritic
{
public:
  ScriptedScoreCritic(
    std::string name,
    std::vector<double> raw_scores,
    const int throwing_candidate,
    std::vector<std::string> & call_trace)
  : raw_scores_(std::move(raw_scores)),
    throwing_candidate_(throwing_candidate),
    call_trace_(call_trace)
  {
    name_ = std::move(name);
    scale_ = 1.0;
  }

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    const int candidate = static_cast<int>(
      std::lround(trajectory.velocity.x * 10.0));
    call_trace_.push_back(name_ + ":" + std::to_string(candidate));
    if (candidate == throwing_candidate_) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "scripted rejection");
    }
    return raw_scores_.at(static_cast<std::size_t>(candidate));
  }

private:
  std::vector<double> raw_scores_;
  int throwing_candidate_{-1};
  std::vector<std::string> & call_trace_;
};

class BaseCorePlannerAdapter final : public dwb_core::DWBLocalPlanner
{
public:
  void set_test_components(
    dwb_core::TrajectoryGenerator::Ptr generator,
    std::vector<dwb_core::TrajectoryCritic::Ptr> critics)
  {
    traj_generator_ = std::move(generator);
    critics_ = std::move(critics);
    short_circuit_trajectory_evaluation_ = true;
    debug_trajectory_details_ = false;
  }

  dwb_msgs::msg::TrajectoryScore run_core()
  {
    geometry_msgs::msg::Pose2D pose;
    nav_2d_msgs::msg::Twist2D velocity;
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
    return dwb_core::DWBLocalPlanner::coreScoringAlgorithm(
      pose, velocity, results);
  }
};

class ScorePlannerAdapter final : public CertifiedDWBLocalPlanner
{
public:
  using TestDiagnosticPublication = DiagnosticPublication;
  using TestTerminalStopAssessment = TerminalStopAssessment;

  void set_test_critics(
    std::vector<dwb_core::TrajectoryCritic::Ptr> critics,
    const bool short_circuit)
  {
    critics_ = std::move(critics);
    short_circuit_trajectory_evaluation_ = short_circuit;
  }

  void set_test_components(
    dwb_core::TrajectoryGenerator::Ptr generator,
    std::vector<dwb_core::TrajectoryCritic::Ptr> critics)
  {
    traj_generator_ = std::move(generator);
    critics_ = std::move(critics);
    short_circuit_trajectory_evaluation_ = true;
    debug_trajectory_details_ = false;
  }

  dwb_msgs::msg::TrajectoryScore run_local_core()
  {
    geometry_msgs::msg::Pose2D pose;
    nav_2d_msgs::msg::Twist2D velocity;
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
    return coreScoringAlgorithm(pose, velocity, results);
  }

  dwb_msgs::msg::TrajectoryScore run_terminal_core(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
  {
    return coreScoringAlgorithm(pose, velocity, results);
  }

  dwb_msgs::msg::TrajectoryScore reference_score(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const double best_score)
  {
    return dwb_core::DWBLocalPlanner::scoreTrajectory(
      trajectory, best_score);
  }

  dwb_msgs::msg::TrajectoryScore total_only_score(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const double best_score)
  {
    dwb_msgs::msg::TrajectoryScore score;
    score_trajectory_components(
      trajectory, best_score, score, false, nullptr);
    return score;
  }

  visualization_msgs::msg::MarkerArray candidate_markers(
    const dwb_msgs::msg::LocalPlanEvaluation & evaluation) const
  {
    return build_candidate_markers(evaluation);
  }

  static bool coalesce_stale_marker(
    std::deque<TestDiagnosticPublication> & publications,
    TestDiagnosticPublication publication)
  {
    return coalesce_stale_marker_publication(
      publications, std::move(publication));
  }

  static constexpr std::size_t maximum_pending_full_evaluations()
  {
    return kMaximumPendingFullEvaluations;
  }

  static bool full_evaluation_capacity(
    const std::size_t pending_full_evaluations)
  {
    return has_full_evaluation_capacity(pending_full_evaluations);
  }

  static uint64_t clearance_bucket(
    const double risk,
    const double admissible_risk,
    const double resolution)
  {
    return clearance_constraint_bucket(
      risk, admissible_risk, resolution);
  }

  static double zero_scale_clearance_diagnostic(
    const bool is_primary,
    const bool is_trigger,
    const std::optional<double> primary_risk,
    const std::optional<double> trigger_risk)
  {
    return zero_scale_clearance_diagnostic_score(
      is_primary, is_trigger, false, primary_risk, trigger_risk,
      std::nullopt);
  }

  static bool clearance_prefers_candidate(
    const bool candidate_has_meaningful_progress,
    const bool best_has_meaningful_progress,
    const uint64_t candidate_risk_bucket,
    const uint64_t best_risk_bucket,
    const double candidate_total,
    const double best_total,
    const std::size_t candidate_index,
    const std::size_t best_index)
  {
    return clearance_constraint_prefers_candidate(
      candidate_has_meaningful_progress, best_has_meaningful_progress,
      0u, 0u,
      candidate_risk_bucket, best_risk_bucket,
      candidate_total, best_total, candidate_index, best_index);
  }

  static bool clearance_guard_prefers_candidate(
    const uint64_t candidate_guard_bucket,
    const uint64_t best_guard_bucket,
    const uint64_t candidate_risk_bucket,
    const uint64_t best_risk_bucket,
    const double candidate_total,
    const double best_total)
  {
    return clearance_constraint_prefers_candidate(
      true, true, candidate_guard_bucket, best_guard_bucket,
      candidate_risk_bucket, best_risk_bucket,
      candidate_total, best_total, 0u, 1u);
  }

  static bool clearance_is_active_for_pair(
    const bool enabled,
    const double best_total,
    const uint64_t candidate_trigger_bucket,
    const uint64_t best_trigger_bucket)
  {
    return clearance_constraint_is_active_for_pair(
      enabled, best_total, candidate_trigger_bucket, best_trigger_bucket);
  }

  static bool has_meaningful_subgoal_progress(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const nav_2d_msgs::msg::Path2D & path,
    const geometry_msgs::msg::Pose2D & subgoal,
    const double minimum_distance_progress,
    const double minimum_heading_progress)
  {
    return trajectory_has_meaningful_subgoal_progress(
      trajectory, path, subgoal, minimum_distance_progress,
      minimum_heading_progress);
  }

  static bool has_observable_motion(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const double minimum_translation,
    const double minimum_rotation)
  {
    return trajectory_has_observable_motion(
      trajectory, minimum_translation, minimum_rotation);
  }

  static bool executable_progress(
    const bool stop_translation,
    const bool rollout_translation,
    const bool stop_heading,
    const bool rollout_heading,
    const bool stop_heading_motion)
  {
    return receding_horizon_progress_is_executable(
      stop_translation, rollout_translation, stop_heading, rollout_heading,
      stop_heading_motion);
  }

  static bool has_observable_rotation(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const double minimum_rotation)
  {
    return pose_sequence_has_observable_rotation(poses, minimum_rotation);
  }

  static bool has_observable_translation(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const double minimum_translation)
  {
    return pose_sequence_has_observable_translation(
      poses, minimum_translation);
  }

  static bool preserves_turn(
    const double candidate_angular_velocity,
    const double established_angular_velocity)
  {
    return preserves_established_turn_direction(
      candidate_angular_velocity, established_angular_velocity);
  }

  static bool terminal_plan_fallback(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & terminal_pose,
    const double capture_distance)
  {
    return terminal_plan_fallback_is_applicable(
      pose, terminal_pose, capture_distance);
  }

  static bool terminal_goal_hold(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & goal_pose,
    const double capture_distance,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const double stop_velocity_threshold)
  {
    return terminal_goal_hold_is_applicable(
      pose, goal_pose, capture_distance, velocity,
      stop_velocity_threshold);
  }

  static TestTerminalStopAssessment terminal_stop_assessment(
    const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
    const geometry_msgs::msg::Pose2D & goal_pose,
    const double terminal_path_heading,
    const double capture_distance,
    const double capture_yaw_tolerance,
    const double maximum_overshoot)
  {
    return assess_terminal_stop(
      stop_poses, goal_pose, terminal_path_heading, capture_distance,
      capture_yaw_tolerance, maximum_overshoot);
  }

  static bool terminal_prefers_candidate(
    const bool candidate_captures_goal,
    const bool best_captures_goal,
    const double candidate_total,
    const double best_total,
    const std::size_t candidate_index,
    const std::size_t best_index)
  {
    return terminal_stop_prefers_candidate(
      candidate_captures_goal, best_captures_goal,
      candidate_total, best_total, candidate_index, best_index);
  }

  static bool terminal_stop_scoring_enabled(
    const bool certification_enabled,
    const bool stop_admissibility_enabled,
    const double goal_distance_scale,
    const bool target_pose_valid)
  {
    return should_score_terminal_stop(
      certification_enabled, stop_admissibility_enabled,
      goal_distance_scale, target_pose_valid);
  }

  static bool recovery_prefers_candidate(
    const double candidate_collision_time,
    const double best_collision_time,
    const double candidate_clearance_risk,
    const double best_clearance_risk,
    const double candidate_path_departure_cost,
    const double best_path_departure_cost,
    const std::size_t candidate_index,
    const std::size_t best_index)
  {
    return receding_horizon_recovery_prefers_candidate(
      candidate_collision_time, best_collision_time,
      candidate_clearance_risk, best_clearance_risk,
      candidate_path_departure_cost, best_path_departure_cost,
      candidate_index, best_index);
  }

  static bool least_violation_prefers_candidate(
    const double candidate_collision_time,
    const double best_collision_time,
    const double candidate_residual_weighted_cost,
    const double best_residual_weighted_cost,
    const std::size_t candidate_index,
    const std::size_t best_index)
  {
    return least_violation_recovery_prefers_candidate(
      candidate_collision_time, best_collision_time,
      candidate_residual_weighted_cost, best_residual_weighted_cost,
      candidate_index, best_index);
  }

  static bool recovery_preserves_uncertainty_reserve(
    const double collision_time,
    const uint64_t clearance_guard_bucket,
    const double approach_risk,
    const double maximum_approach_risk,
    const double minimum_collision_horizon)
  {
    return recovery_candidate_preserves_uncertainty_reserve(
      collision_time, clearance_guard_bucket, approach_risk,
      maximum_approach_risk, minimum_collision_horizon);
  }

  using ProgressRank = ProgressEscapeRank;

  static double reserve_approach_limit(
    const bool recovers_initial_clearance)
  {
    return uncertainty_reserve_approach_limit(
      recovers_initial_clearance);
  }

  static bool consumes_reserve(
    const double initial_clearance,
    const double terminal_clearance,
    const double uncertainty_margin,
    const double tolerance)
  {
    return consumes_uncertainty_reserve(
      initial_clearance, terminal_clearance, uncertainty_margin, tolerance);
  }

  static bool progress_escape_prefers(
    const ProgressRank & candidate,
    const ProgressRank & best)
  {
    return progress_escape_prefers_candidate(candidate, best);
  }

  static bool progress_escape_replaces_weighted_winner(
    const bool candidate_found,
    const bool selected_progress_was_evaluated,
    const bool selected_has_receding_horizon_progress)
  {
    return progress_escape_should_replace_weighted_winner(
      candidate_found, selected_progress_was_evaluated,
      selected_has_receding_horizon_progress);
  }

  static bool legal_escape_prefers_candidate(
    const uint64_t candidate_guard_bucket,
    const uint64_t best_guard_bucket,
    const uint64_t candidate_risk_bucket,
    const uint64_t best_risk_bucket,
    const double candidate_approach_risk,
    const double best_approach_risk,
    const bool candidate_preserves_turn_direction,
    const bool best_preserves_turn_direction,
    const double candidate_heading_excursion,
    const double best_heading_excursion,
    const double candidate_translation_distance,
    const double best_translation_distance,
    const std::size_t candidate_index,
    const std::size_t best_index)
  {
    return legal_avoidance_escape_prefers_candidate(
      candidate_guard_bucket, best_guard_bucket,
      candidate_risk_bucket, best_risk_bucket,
      candidate_approach_risk, best_approach_risk,
      candidate_preserves_turn_direction, best_preserves_turn_direction,
      candidate_heading_excursion, best_heading_excursion,
      candidate_translation_distance, best_translation_distance,
      candidate_index, best_index);
  }

  static std::optional<double> fuse_clearance_risks(
    const std::optional<double> primary_risk,
    const std::optional<double> guard_risk)
  {
    return fused_clearance_risk(primary_risk, guard_risk);
  }

  static double collision_time(
    const CertificationResult & result,
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const double maximum_swept_distance,
    const double control_period)
  {
    return predicted_collision_time(
      result, poses, footprint, maximum_swept_distance, control_period);
  }

  static double rejection_collision_time(
    const std::string & detail,
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const double maximum_swept_distance,
    const double control_period)
  {
    return predicted_collision_time_from_obstacle_rejection(
      detail, poses, footprint, maximum_swept_distance, control_period);
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

TEST_F(NativeInputTrajectoryGeneratorTest, MeanSpeedTargetRefreshesAtStoppedTrialReset)
{
  const auto node = make_node("mean_speed_target_reset_test");
  MeanSpeedCritic critic;
  critic.initialize(node, "MeanSpeed", kPluginName, nullptr);
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose, velocity, pose, path));
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(2u);
  trajectory.poses.back().x = 0.4;
  trajectory.time_offsets.emplace_back();
  trajectory.time_offsets.back().sec = 1;
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 0.8, 1.0e-12);
  for (const double target : {0.5, 1.5}) {
    const double previous_cost = critic.scoreTrajectory(trajectory);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
        "FollowPath.MeanSpeed.target_speed", target)).successful);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory), previous_cost);
    critic.reset();
    EXPECT_NEAR(critic.scoreTrajectory(trajectory), target - 0.4, 1.0e-12);
  }
}

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

TEST_F(NativeInputTrajectoryGeneratorTest, ExplicitFirTapCountMatchesExecutedCoefficients)
{
  auto node = make_node("explicit_fir_taps");
  node->declare_parameter("FollowPath.fir_effective_taps", 3);
  FirTrajectoryGenerator generator;
  EXPECT_NO_THROW(generator.initialize(node, kPluginName));
  node->set_parameter(rclcpp::Parameter("FollowPath.fir_effective_taps", 92));
  EXPECT_THROW(generator.reset_trial_state(), std::invalid_argument);
  node->set_parameter(rclcpp::Parameter("FollowPath.fir_effective_taps", 3));
  EXPECT_NO_THROW(generator.reset_trial_state());
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  DirectStopPreservesAccelerationJerkAndFirState)
{
  const auto acceleration_node = make_node("acceleration_direct_stop_test");
  AccelerationTrajectoryGenerator acceleration_generator;
  acceleration_generator.initialize(acceleration_node, kPluginName);
  expect_direct_stop_trajectory(
    acceleration_generator, acceleration_node, false, false);

  const auto jerk_node = make_node("jerk_direct_stop_test");
  JerkTrajectoryGenerator jerk_generator;
  jerk_generator.initialize(jerk_node, kPluginName);
  expect_direct_stop_trajectory(jerk_generator, jerk_node, true, false);

  const auto fir_node = make_node("fir_direct_stop_test");
  FirTrajectoryGenerator fir_generator;
  fir_generator.initialize(fir_node, kPluginName);
  expect_direct_stop_trajectory(fir_generator, fir_node, false, true);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  AccelerationSamples165DistinctFullHorizonFeasibleRolloutsAtVelocityLimit)
{
  constexpr double kControlPeriod = 0.05;
  constexpr int kRolloutStepCount = 48;
  const auto node = make_node(
    "acceleration_horizon_sampling_test", true, false, false, 0.0,
    0.6, 0.6, kControlPeriod);
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  PlanningSnapshot acceleration_snapshot =
    make_observable_zero_snapshot(node->now());
  acceleration_snapshot.current_state.velocity.x = 0.6;
  acceleration_snapshot.activation_state.velocity.x = 0.6;
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(acceleration_snapshot));

  const AxisLimits linear_limits{0.0, 0.6, -1.2, 1.2, -1.2, 1.2};
  const AxisLimits angular_limits{-0.6, 0.6, -1.57, 1.57, -1.57, 1.57};
  const AxisState initial_linear_state{0.6, 0.0};
  const AxisState initial_angular_state{0.0, 0.0};
  const FeasibleInterval linear_interval =
    held_acceleration_input_interval(
    initial_linear_state, linear_limits, kControlPeriod,
    kRolloutStepCount);
  const FeasibleInterval angular_interval =
    held_acceleration_input_interval(
    initial_angular_state, angular_limits, kControlPeriod,
    kRolloutStepCount);
  ASSERT_TRUE(linear_interval.feasible);
  ASSERT_TRUE(angular_interval.feasible);
  EXPECT_NEAR(linear_interval.lower, -0.25, 1.0e-12);
  EXPECT_NEAR(linear_interval.upper, 0.0, 1.0e-12);
  EXPECT_NEAR(angular_interval.lower, -0.25, 1.0e-12);
  EXPECT_NEAR(angular_interval.upper, 0.25, 1.0e-12);
  const auto expected_linear_inputs = uniform_samples(linear_interval, 11);
  const auto expected_angular_inputs = uniform_samples(angular_interval, 15);

  nav_2d_msgs::msg::Twist2D current_velocity;
  current_velocity.x = 0.6;
  generator.startNewIteration(current_velocity);
  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.37;
  start_pose.y = -0.19;
  start_pose.theta = 0.41;
  std::set<std::vector<double>> distinct_rollouts;
  std::size_t candidate_count = 0u;
  double minimum_linear_input = std::numeric_limits<double>::infinity();
  double maximum_linear_input = -std::numeric_limits<double>::infinity();
  double minimum_angular_input = std::numeric_limits<double>::infinity();
  double maximum_angular_input = -std::numeric_limits<double>::infinity();
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    const auto command_state = generator.active_candidate_command_state();
    ASSERT_TRUE(command_state.has_value());
    const double linear_input = command_state->linear_state.acceleration;
    const double angular_input = command_state->angular_state.acceleration;
    const auto canonical_index =
      generator.active_candidate_canonical_index();
    ASSERT_TRUE(canonical_index.has_value());
    ASSERT_EQ(*canonical_index, candidate_count);
    const std::size_t linear_index = *canonical_index / 15u;
    const std::size_t angular_index = *canonical_index % 15u;
    ASSERT_LT(linear_index, expected_linear_inputs.size());
    ASSERT_LT(angular_index, expected_angular_inputs.size());
    EXPECT_NEAR(
      linear_input, expected_linear_inputs[linear_index], 1.0e-12);
    EXPECT_NEAR(
      angular_input, expected_angular_inputs[angular_index], 1.0e-12);
    minimum_linear_input = std::min(minimum_linear_input, linear_input);
    maximum_linear_input = std::max(maximum_linear_input, linear_input);
    minimum_angular_input = std::min(minimum_angular_input, angular_input);
    maximum_angular_input = std::max(maximum_angular_input, angular_input);

    const auto trajectory = generator.generateTrajectory(
      start_pose, current_velocity, command);
    ASSERT_EQ(trajectory.poses.size(), 50u);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().x, start_pose.x);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().y, start_pose.y);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().theta, start_pose.theta);
    distinct_rollouts.insert(trajectory_signature(trajectory));

    AxisState linear_state = initial_linear_state;
    AxisState angular_state = initial_angular_state;
    geometry_msgs::msg::Pose2D reference_pose = start_pose;
    for (int remaining_steps = kRolloutStepCount;
      remaining_steps > 0; --remaining_steps)
    {
      const auto linear_step = project_held_acceleration_step(
        linear_state, linear_limits, linear_input, kControlPeriod,
        remaining_steps);
      const auto angular_step = project_held_acceleration_step(
        angular_state, angular_limits, angular_input, kControlPeriod,
        remaining_steps);
      ASSERT_TRUE(linear_step.feasible);
      ASSERT_TRUE(angular_step.feasible);
      EXPECT_NEAR(linear_step.applied_native_input, linear_input, 1.0e-12);
      EXPECT_NEAR(angular_step.applied_native_input, angular_input, 1.0e-12);
      linear_state = linear_step.state;
      angular_state = angular_step.state;
      expect_axis_state_within_limits(linear_state, linear_limits);
      expect_axis_state_within_limits(angular_state, angular_limits);
      nav_2d_msgs::msg::Twist2D reference_velocity;
      reference_velocity.x = linear_state.velocity;
      reference_velocity.theta = angular_state.velocity;
      reference_pose = legacy_compute_new_position(
        reference_pose, reference_velocity, kControlPeriod);
      const std::size_t pose_index =
        static_cast<std::size_t>(
        kRolloutStepCount - remaining_steps + 1);
      ASSERT_LT(pose_index, trajectory.poses.size());
      EXPECT_NEAR(trajectory.poses[pose_index].x, reference_pose.x, 1.0e-12);
      EXPECT_NEAR(trajectory.poses[pose_index].y, reference_pose.y, 1.0e-12);
      EXPECT_NEAR(
        trajectory.poses[pose_index].theta, reference_pose.theta, 1.0e-12);
    }
    const auto final_pose_index =
      static_cast<std::size_t>(kRolloutStepCount);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().x, trajectory.poses[final_pose_index].x);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().y, trajectory.poses[final_pose_index].y);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().theta,
      trajectory.poses[final_pose_index].theta);
    ++candidate_count;
  }

  EXPECT_EQ(candidate_count, 165u);
  EXPECT_EQ(distinct_rollouts.size(), 165u);
  EXPECT_NEAR(minimum_linear_input, linear_interval.lower, 1.0e-12);
  EXPECT_NEAR(maximum_linear_input, linear_interval.upper, 1.0e-12);
  EXPECT_NEAR(minimum_angular_input, angular_interval.lower, 1.0e-12);
  EXPECT_NEAR(maximum_angular_input, angular_interval.upper, 1.0e-12);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  JerkSamples165DistinctFullHorizonFeasibleRolloutsAtVelocityLimit)
{
  constexpr double kControlPeriod = 0.05;
  constexpr int kRolloutStepCount = 48;
  constexpr double kMaximumJerk = 1.57;
  const auto node = make_node(
    "jerk_horizon_sampling_test", true, false, false, 0.0,
    0.6, 0.6, kControlPeriod);
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  PlanningSnapshot jerk_snapshot =
    make_observable_zero_snapshot(node->now());
  jerk_snapshot.current_state.velocity.x = 0.6;
  jerk_snapshot.activation_state.velocity.x = 0.6;
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(jerk_snapshot));

  const AxisLimits linear_limits{
    0.0, 0.6, -1.2, 1.2, -kMaximumJerk, kMaximumJerk};
  const AxisLimits angular_limits{
    -0.6, 0.6, -1.57, 1.57, -kMaximumJerk, kMaximumJerk};
  const AxisState initial_linear_state{0.6, 0.0};
  const AxisState initial_angular_state{0.0, 0.0};
  const FeasibleInterval linear_interval = held_jerk_input_interval(
    initial_linear_state, linear_limits, kControlPeriod,
    kRolloutStepCount);
  const FeasibleInterval angular_interval = held_jerk_input_interval(
    initial_angular_state, angular_limits, kControlPeriod,
    kRolloutStepCount);
  ASSERT_TRUE(linear_interval.feasible);
  ASSERT_TRUE(angular_interval.feasible);
  EXPECT_NEAR(linear_interval.lower, -0.2040816326530612, 1.0e-12);
  EXPECT_NEAR(linear_interval.upper, 0.0, 1.0e-12);
  EXPECT_NEAR(angular_interval.lower, -0.2040816326530612, 1.0e-12);
  EXPECT_NEAR(angular_interval.upper, 0.2040816326530612, 1.0e-12);
  const auto expected_linear_inputs = uniform_samples(linear_interval, 11);
  const auto expected_angular_inputs = uniform_samples(angular_interval, 15);

  nav_2d_msgs::msg::Twist2D current_velocity;
  current_velocity.x = 0.6;
  generator.startNewIteration(current_velocity);
  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.37;
  start_pose.y = -0.19;
  start_pose.theta = 0.41;
  std::set<std::vector<double>> distinct_rollouts;
  std::size_t candidate_count = 0u;
  double minimum_linear_input = std::numeric_limits<double>::infinity();
  double maximum_linear_input = -std::numeric_limits<double>::infinity();
  double minimum_angular_input = std::numeric_limits<double>::infinity();
  double maximum_angular_input = -std::numeric_limits<double>::infinity();
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    const auto command_state = generator.active_candidate_command_state();
    ASSERT_TRUE(command_state.has_value());
    const double linear_input =
      command_state->linear_state.acceleration / kControlPeriod;
    const double angular_input =
      command_state->angular_state.acceleration / kControlPeriod;
    const auto canonical_index =
      generator.active_candidate_canonical_index();
    ASSERT_TRUE(canonical_index.has_value());
    ASSERT_EQ(*canonical_index, candidate_count);
    const std::size_t linear_index = *canonical_index / 15u;
    const std::size_t angular_index = *canonical_index % 15u;
    ASSERT_LT(linear_index, expected_linear_inputs.size());
    ASSERT_LT(angular_index, expected_angular_inputs.size());
    EXPECT_NEAR(
      linear_input, expected_linear_inputs[linear_index], 1.0e-12);
    EXPECT_NEAR(
      angular_input, expected_angular_inputs[angular_index], 1.0e-12);
    EXPECT_LE(std::abs(linear_input), kMaximumJerk + 1.0e-12);
    EXPECT_LE(std::abs(angular_input), kMaximumJerk + 1.0e-12);
    minimum_linear_input = std::min(minimum_linear_input, linear_input);
    maximum_linear_input = std::max(maximum_linear_input, linear_input);
    minimum_angular_input = std::min(minimum_angular_input, angular_input);
    maximum_angular_input = std::max(maximum_angular_input, angular_input);

    const auto trajectory = generator.generateTrajectory(
      start_pose, current_velocity, command);
    ASSERT_EQ(trajectory.poses.size(), 50u);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().x, start_pose.x);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().y, start_pose.y);
    EXPECT_DOUBLE_EQ(trajectory.poses.front().theta, start_pose.theta);
    distinct_rollouts.insert(trajectory_signature(trajectory));

    AxisState linear_state = initial_linear_state;
    AxisState angular_state = initial_angular_state;
    geometry_msgs::msg::Pose2D reference_pose = start_pose;
    for (int remaining_steps = kRolloutStepCount;
      remaining_steps > 0; --remaining_steps)
    {
      const auto linear_step = project_held_jerk_step(
        linear_state, linear_limits, linear_input, kControlPeriod,
        remaining_steps);
      const auto angular_step = project_held_jerk_step(
        angular_state, angular_limits, angular_input, kControlPeriod,
        remaining_steps);
      ASSERT_TRUE(linear_step.feasible);
      ASSERT_TRUE(angular_step.feasible);
      EXPECT_NEAR(linear_step.applied_native_input, linear_input, 1.0e-12);
      EXPECT_NEAR(angular_step.applied_native_input, angular_input, 1.0e-12);
      EXPECT_LE(
        std::abs(linear_step.applied_native_input),
        kMaximumJerk + 1.0e-12);
      EXPECT_LE(
        std::abs(angular_step.applied_native_input),
        kMaximumJerk + 1.0e-12);
      linear_state = linear_step.state;
      angular_state = angular_step.state;
      expect_axis_state_within_limits(linear_state, linear_limits);
      expect_axis_state_within_limits(angular_state, angular_limits);
      nav_2d_msgs::msg::Twist2D reference_velocity;
      reference_velocity.x = linear_state.velocity;
      reference_velocity.theta = angular_state.velocity;
      reference_pose = legacy_compute_new_position(
        reference_pose, reference_velocity, kControlPeriod);
      const std::size_t pose_index =
        static_cast<std::size_t>(
        kRolloutStepCount - remaining_steps + 1);
      ASSERT_LT(pose_index, trajectory.poses.size());
      EXPECT_NEAR(trajectory.poses[pose_index].x, reference_pose.x, 1.0e-12);
      EXPECT_NEAR(trajectory.poses[pose_index].y, reference_pose.y, 1.0e-12);
      EXPECT_NEAR(
        trajectory.poses[pose_index].theta, reference_pose.theta, 1.0e-12);
    }
    const auto final_pose_index =
      static_cast<std::size_t>(kRolloutStepCount);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().x, trajectory.poses[final_pose_index].x);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().y, trajectory.poses[final_pose_index].y);
    EXPECT_DOUBLE_EQ(
      trajectory.poses.back().theta,
      trajectory.poses[final_pose_index].theta);
    ++candidate_count;
  }

  EXPECT_EQ(candidate_count, 165u);
  EXPECT_EQ(distinct_rollouts.size(), 165u);
  EXPECT_NEAR(minimum_linear_input, linear_interval.lower, 1.0e-12);
  EXPECT_NEAR(maximum_linear_input, linear_interval.upper, 1.0e-12);
  EXPECT_NEAR(minimum_angular_input, angular_interval.lower, 1.0e-12);
  EXPECT_NEAR(maximum_angular_input, angular_interval.upper, 1.0e-12);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  AccelerationAndJerkRetainOneZeroCandidateAtRest)
{
  constexpr double kControlPeriod = 0.05;
  const auto acceleration_node = make_node(
    "acceleration_zero_candidate_test", true, false, false, 0.0,
    0.6, 0.6, kControlPeriod);
  const auto jerk_node = make_node(
    "jerk_zero_candidate_test", true, false, false, 0.0,
    0.6, 0.6, kControlPeriod);
  AccelerationTrajectoryGenerator acceleration_generator;
  JerkTrajectoryGenerator jerk_generator;
  acceleration_generator.initialize(acceleration_node, kPluginName);
  jerk_generator.initialize(jerk_node, kPluginName);

  const auto zero_candidate_indices = [](
    NativeInputTrajectoryGenerator & generator)
    {
      nav_2d_msgs::msg::Twist2D current_velocity;
      generator.startNewIteration(current_velocity);
      std::size_t candidate_count = 0u;
      std::vector<std::size_t> indices;
      while (generator.hasMoreTwists()) {
        const auto command = generator.nextTwist();
        if (std::abs(command.x) <= 1.0e-12 &&
          std::abs(command.theta) <= 1.0e-12)
        {
          const auto canonical_index =
            generator.active_candidate_canonical_index();
          EXPECT_TRUE(canonical_index.has_value());
          if (canonical_index.has_value()) {
            indices.push_back(*canonical_index);
          }
        }
        ++candidate_count;
      }
      EXPECT_EQ(candidate_count, 165u);
      return indices;
    };

  EXPECT_EQ(
    zero_candidate_indices(acceleration_generator),
    std::vector<std::size_t>({7u}));
  EXPECT_EQ(
    zero_candidate_indices(jerk_generator),
    std::vector<std::size_t>({7u}));
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

TEST_F(NativeInputTrajectoryGeneratorTest, FirDurationBankMatchesIndependentScalarRollouts)
{
  const std::vector<double> durations{0.12, 0.30, 0.60, 0.0};
  const auto node = make_node("fir_duration_bank_test", true, false, false, durations.front());
  node->declare_parameter("FollowPath.fir_prediction_pulse_durations", durations);
  FirTrajectoryGenerator bank;
  bank.initialize(node, kPluginName);
  for (const double initial_velocity : {0.0, 0.6, 1.19}) {
    auto snapshot = make_observable_zero_snapshot(node->now());
    snapshot.activation_state.velocity.x = initial_velocity;
    snapshot.activation_state.velocity.theta = -0.2;
    snapshot.activation_state.linear_fir_history = {0.02, -0.03};
    snapshot.activation_state.angular_fir_history = {-0.08, 0.03};
    const auto shared_snapshot = std::make_shared<const PlanningSnapshot>(snapshot);
    bank.set_planning_snapshot(shared_snapshot);
    bank.startNewIteration(snapshot.activation_state.velocity);
    geometry_msgs::msg::Pose2D pose;
    pose.theta = 0.42;
    std::size_t count = 0u;
    for (const double duration : durations) {
      const auto scalar_node = make_node("fir_duration_scalar_test", true, false, false, duration);
      FirTrajectoryGenerator scalar;
      scalar.initialize(scalar_node, kPluginName);
      scalar.set_planning_snapshot(shared_snapshot);
      scalar.startNewIteration(snapshot.activation_state.velocity);
      while (scalar.hasMoreTwists()) {
        ASSERT_TRUE(bank.hasMoreTwists());
        const auto scalar_command = scalar.nextTwist();
        const auto bank_command = bank.nextTwist();
        ASSERT_EQ(bank_command, scalar_command);
        EXPECT_EQ(bank.active_candidate_canonical_index(), count++);
        EXPECT_EQ(bank.generateTrajectory(pose, nav_2d_msgs::msg::Twist2D(), bank_command),
          scalar.generateTrajectory(pose, nav_2d_msgs::msg::Twist2D(), scalar_command));
        const auto bank_state = bank.active_candidate_command_state();
        const auto scalar_state = scalar.active_candidate_command_state();
        ASSERT_TRUE(bank_state && scalar_state);
        EXPECT_EQ(bank_state->linear_fir_history, scalar_state->linear_fir_history);
        EXPECT_EQ(bank_state->angular_fir_history, scalar_state->angular_fir_history);
        EXPECT_DOUBLE_EQ(bank_state->linear_state.acceleration,
            scalar_state->linear_state.acceleration);
        EXPECT_DOUBLE_EQ(bank_state->angular_state.acceleration,
            scalar_state->angular_state.acceleration);
        const auto diagnostics = bank.active_candidate_diagnostics();
        ASSERT_TRUE(diagnostics);
        EXPECT_NEAR(diagnostics->linear_prediction_input_duration, duration > 0.0 ? duration : 2.4,
          1.0e-12);
        EXPECT_DOUBLE_EQ(diagnostics->angular_prediction_input_duration,
          diagnostics->linear_prediction_input_duration);
        std::vector<geometry_msgs::msg::Pose2D> bank_stop, scalar_stop;
        std::vector<nav_2d_msgs::msg::Twist2D> bank_velocities, scalar_velocities;
        const bool scalar_stoppable = scalar.generate_stop_trajectory(pose, 267, 0.01, scalar_stop,
            scalar_velocities);
        const bool bank_stoppable = bank.generate_stop_trajectory(pose, 267, 0.01, bank_stop,
            bank_velocities);
        ASSERT_EQ(bank_stoppable, scalar_stoppable);
        EXPECT_EQ(bank_stop, scalar_stop);
        EXPECT_EQ(bank_velocities, scalar_velocities);
      }
    }
    EXPECT_GT(count, 165u);
    EXPECT_FALSE(bank.hasMoreTwists());
    EXPECT_EQ(count, bank.candidate_count());
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirDurationBankPreservesFilterThroughPulseEnd)
{
  for (const bool independent : {false, true}) {
    const auto node = make_node("fir_duration_filter_test", true, false, false, 0.12);
    node->declare_parameter("FollowPath.fir_prediction_pulse_durations",
      std::vector<double>{0.30, 0.60});
    node->declare_parameter("FollowPath.fir_independent_pulse_durations", independent);
    FirTrajectoryGenerator generator;
    generator.initialize(node, kPluginName);
    generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
    ASSERT_EQ(generator.candidate_count(), independent ? 1485u : 495u);
    std::vector<dwb_msgs::msg::Trajectory2D> maximum_turn_trajectories;
    std::vector<nav_2d_msgs::msg::Twist2D> maximum_turn_commands;
    while (generator.hasMoreTwists()) {
      const auto command = generator.nextTwist();
      const auto diagnostics = generator.active_candidate_diagnostics();
      ASSERT_TRUE(diagnostics);
      const int linear_input_steps =
        static_cast<int>(std::lround(diagnostics->linear_prediction_input_duration /
        0.03));
      const int angular_input_steps =
        static_cast<int>(std::lround(diagnostics->angular_prediction_input_duration /
        0.03));
      const auto trajectory = generator.generateTrajectory(geometry_msgs::msg::Pose2D(),
        nav_2d_msgs::msg::Twist2D(), command);
      std::vector<double> linear_history(2u, 0.0), angular_history(2u, 0.0);
      nav_2d_msgs::msg::Twist2D velocity;
      geometry_msgs::msg::Pose2D pose;
      for (int step = 0; step < 80; ++step) {
        const double linear_input = step <
          linear_input_steps ? diagnostics->linear_native_input : 0.0;
        const double angular_input = step <
          angular_input_steps ? diagnostics->angular_native_input : 0.0;
        const double linear_acceleration = fir_acceleration({0.5, 0.3, 0.2}, linear_history,
          linear_input);
        const double angular_acceleration = fir_acceleration({0.5, 0.3, 0.2}, angular_history,
          angular_input);
        velocity.x += 0.03 * linear_acceleration;
        velocity.theta += 0.03 * angular_acceleration;
        EXPECT_GE(velocity.x, -1.0e-12);
        EXPECT_LE(velocity.x, 1.2 + 1.0e-12);
        EXPECT_LE(std::abs(velocity.theta), 1.57 + 1.0e-12);
        EXPECT_LE(std::abs(linear_acceleration), 1.2 + 1.0e-12);
        EXPECT_LE(std::abs(angular_acceleration), 1.57 + 1.0e-12);
        pose = legacy_compute_new_position(pose, velocity, 0.03);
        EXPECT_NEAR(trajectory.poses[step + 1u].x, pose.x, 1.0e-12);
        EXPECT_NEAR(trajectory.poses[step + 1u].y, pose.y, 1.0e-12);
        EXPECT_NEAR(trajectory.poses[step + 1u].theta, pose.theta, 1.0e-12);
        push_fir_input(linear_history, linear_input);
        push_fir_input(angular_history, angular_input);
      }
      if (diagnostics->canonical_index % 165u == 164u) {
        maximum_turn_commands.push_back(command);
        maximum_turn_trajectories.push_back(trajectory);
      }
    }
    ASSERT_EQ(maximum_turn_trajectories.size(), independent ? 9u : 3u);
    EXPECT_EQ(maximum_turn_commands[0], maximum_turn_commands[1]);
    EXPECT_EQ(maximum_turn_commands[1], maximum_turn_commands[2]);
    EXPECT_GT(std::abs(maximum_turn_trajectories[0].poses.back().y -
    maximum_turn_trajectories[1].poses.back().y), 0.05);
    EXPECT_GT(std::abs(maximum_turn_trajectories[1].poses.back().y -
    maximum_turn_trajectories[2].poses.back().y), 0.05);
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirDurationBankDeduplicatesControllerTicks)
{
  const auto node = make_node("fir_duration_ticks_test", true, false, false, 0.12);
  node->declare_parameter("FollowPath.fir_prediction_pulse_durations",
    std::vector<double>{0.10, 0.12, 0.119, 0.0, 2.4, 0.0});
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.candidate_count(), 330u);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
    "FollowPath.fir_prediction_pulse_durations", std::vector<double>{0.30, 0.60})).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.candidate_count(), 495u);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
    "FollowPath.fir_independent_pulse_durations", true)).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.candidate_count(), 1485u);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
    "FollowPath.fir_prediction_pulse_durations", std::vector<double>{})).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.candidate_count(), 165u);
}

TEST_F(NativeInputTrajectoryGeneratorTest, FirDurationBankRejectsInvalidRuntimeValuesAtomically)
{
  const auto node = make_node("fir_duration_invalid_test", true, false, false, 0.12);
  node->declare_parameter("FollowPath.fir_prediction_pulse_durations", std::vector<double>{0.30});
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  for (const double invalid : {-0.1, 2.41, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
      "FollowPath.fir_prediction_pulse_durations", std::vector<double>{invalid})).successful);
    EXPECT_THROW(generator.reset_trial_state(), std::invalid_argument);
    generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
    EXPECT_EQ(generator.candidate_count(), 330u);
  }
}

// Explicit opt-in benchmark: no hardware or ROS graph interaction. Supply the
// real Python-designed taps, not the short three-tap unit-test filter.
TEST_F(NativeInputTrajectoryGeneratorTest, DISABLED_FirSamplingBenchmark)
{
  const char * coefficients_text = std::getenv("F_DWA_BENCHMARK_COEFFICIENTS");
  ASSERT_NE(coefficients_text, nullptr);
  std::istringstream coefficients_stream(coefficients_text);
  std::vector<double> coefficients;
  std::string token;
  while (std::getline(coefficients_stream, token, ',')) {
    coefficients.push_back(std::stod(token));
  }
  ASSERT_FALSE(coefficients.empty());
  struct SamplingCase
  {
    const char * name;
    int linear_samples;
    int angular_samples;
    std::vector<double> durations;
    bool independent{false};
    };
  const std::vector<SamplingCase> cases{
    {"baseline", 11, 15, {}},
    {"duration3", 11, 15, {0.20, 0.40}},
    {"amplitude_dense", 31, 57, {}},
    {"hybrid3", 21, 29, {0.20, 0.40}},
    {"independent3", 11, 15, {0.20, 0.40}, true},
    {"independent3_dense", 11, 21, {0.20, 0.40}, true},
    {"duration12", 11, 15, {0.20, 0.30, 0.40, 0.50, 0.60, 0.80, 1.0, 1.2, 1.5, 2.0, 2.5}}
  };
  using Clock = std::chrono::steady_clock;
  const auto milliseconds = [](const auto & begin, const auto & end) {
      return std::chrono::duration<double, std::milli>(end - begin).count();
    };
  const auto percentile = [](std::vector<double> values, const double fraction) {
      std::sort(values.begin(), values.end());
      return values[static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1u];
    };
  std::printf(
      "sampling_case,initial_v,candidates,fir_taps,axis_p50_ms,nominal_p50_ms,"
      "stop_p50_ms,total_p50_ms,total_p95_ms\n");
  for (const auto & sampling : cases) {
    const auto node = make_node("fir_sampling_benchmark", true, false, false, 0.10, 0.8, 0.8, 0.05);
    node->declare_parameter("FollowPath.fir_prediction_pulse_durations", sampling.durations);
    node->declare_parameter("FollowPath.fir_independent_pulse_durations", sampling.independent);
    node->declare_parameter("FollowPath.sim_time", 2.5);
    node->declare_parameter("FollowPath.vx_samples", sampling.linear_samples);
    node->declare_parameter("FollowPath.vtheta_samples", sampling.angular_samples);
    node->declare_parameter("FollowPath.fir_coefficients", coefficients);
    ASSERT_TRUE(node->set_parameters_atomically({
        rclcpp::Parameter("FollowPath.sim_time", 2.5),
        rclcpp::Parameter("FollowPath.vx_samples", sampling.linear_samples),
        rclcpp::Parameter("FollowPath.vtheta_samples", sampling.angular_samples),
        rclcpp::Parameter("FollowPath.fir_coefficients", coefficients)}).successful);
    FirTrajectoryGenerator generator;
    generator.initialize(node, kPluginName);
    for (const double speed : {0.0, 0.4, 0.75}) {
      auto snapshot = make_observable_zero_snapshot(node->now());
      snapshot.activation_state.velocity.x = speed;
      snapshot.activation_state.velocity.theta = 0.1;
      snapshot.activation_state.linear_fir_history.assign(coefficients.size() - 1u, 0.0);
      snapshot.activation_state.angular_fir_history.assign(coefficients.size() - 1u, 0.0);
      generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
      std::vector<double> axis_times, nominal_times, stop_times, total_times;
      dwb_msgs::msg::Trajectory2D trajectory;
      std::vector<geometry_msgs::msg::Pose2D> stop_poses;
      std::size_t count = 0u;
      for (int iteration = 0; iteration < 35; ++iteration) {
        const auto begin = Clock::now();
        generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
        const auto axis_end = Clock::now();
        while (generator.hasMoreTwists()) {
          const auto command = generator.nextTwist();
          generator.generate_trajectory_into(geometry_msgs::msg::Pose2D(), command, trajectory);
        }
        const auto nominal_end = Clock::now();
        count = generator.candidate_count();
        for (std::size_t index = 0; index < count; ++index) {
          static_cast<void>(generator.generate_stop_poses_for_candidate(index,
              geometry_msgs::msg::Pose2D(), 160, 0.01, stop_poses));
        }
        const auto stop_end = Clock::now();
        if (iteration >= 5) {
          axis_times.push_back(milliseconds(begin, axis_end));
          nominal_times.push_back(milliseconds(axis_end, nominal_end));
          stop_times.push_back(milliseconds(nominal_end, stop_end));
          total_times.push_back(milliseconds(begin, stop_end));
        }
      }
      std::printf("%s,%.2f,%zu,%zu,%.6f,%.6f,%.6f,%.6f,%.6f\n", sampling.name, speed,
        count, coefficients.size(), percentile(axis_times, 0.50), percentile(nominal_times, 0.50),
        percentile(stop_times, 0.50), percentile(total_times, 0.50), percentile(total_times, 0.95));
    }
  }
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
  ClearanceConstraintKeepsAdmissibleRiskInOneEpsilonSet)
{
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(0.0, 0.05, 0.01), 0u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(0.05, 0.05, 0.01), 0u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(0.0501, 0.05, 0.01), 1u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(0.06, 0.05, 0.01), 1u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(0.0601, 0.05, 0.01), 2u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(1.0, 0.05, 0.01), 95u);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceCostAboveOneRemainsRankedWithoutHardRejection)
{
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(1.10, 0.05, 0.01), 105u);
  EXPECT_EQ(ScorePlannerAdapter::clearance_bucket(1.20, 0.05, 0.01), 115u);
  EXPECT_EQ(
    ScorePlannerAdapter::clearance_bucket(
      std::numeric_limits<double>::max(), 0.05, 0.01),
    std::numeric_limits<uint64_t>::max());
  EXPECT_THROW(
    ScorePlannerAdapter::clearance_bucket(
      std::numeric_limits<double>::infinity(), 0.05, 0.01),
    std::invalid_argument);
  EXPECT_THROW(
    ScorePlannerAdapter::clearance_bucket(
      std::numeric_limits<double>::quiet_NaN(), 0.05, 0.01),
    std::invalid_argument);

  auto critic = std::make_shared<FixedScoreCritic>("FootprintClearance", 175.0, 1.20);
  ScorePlannerAdapter planner;
  planner.set_test_critics({critic}, false);
  dwb_msgs::msg::Trajectory2D trajectory;
  const auto direct = planner.scoreTrajectory(trajectory, -1.0);
  const auto total_only = planner.total_only_score(trajectory, -1.0);
  ASSERT_EQ(direct.scores.size(), 1u);
  EXPECT_DOUBLE_EQ(direct.total, 210.0);
  EXPECT_DOUBLE_EQ(total_only.total, direct.total);
  EXPECT_FLOAT_EQ(direct.scores[0].raw_score, 1.20f);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::zero_scale_clearance_diagnostic(
      true, false, 1.20, std::nullopt), 1.20);
  EXPECT_EQ(critic->call_count(), 2);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintKeepsStrictBoundaryWithoutRankingMapQuantization)
{
  constexpr double admissible_risk = 0.0001;
  constexpr double violation_resolution = 0.01;

  // Preserve the earlier obstacle-avoidance distinction at the admissible
  // boundary while treating the latest run's tiny map-induced differences as
  // members of the same weighted-score set.
  EXPECT_EQ(
    ScorePlannerAdapter::clearance_bucket(
      0.000034, admissible_risk, violation_resolution),
    0u);
  EXPECT_EQ(
    ScorePlannerAdapter::clearance_bucket(
      0.000359, admissible_risk, violation_resolution),
    1u);
  EXPECT_EQ(
    ScorePlannerAdapter::clearance_bucket(
      0.31086189, admissible_risk, violation_resolution),
    ScorePlannerAdapter::clearance_bucket(
      0.31091166, admissible_risk, violation_resolution));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintSeparatesLatestSensorObstacleCandidates)
{
  constexpr double admissible_risk = 0.0001;
  constexpr double violation_resolution = 0.01;
  const uint64_t safer_bucket = ScorePlannerAdapter::clearance_bucket(
    0.7972353101, admissible_risk, violation_resolution);
  const uint64_t selected_bucket = ScorePlannerAdapter::clearance_bucket(
    0.8039881587, admissible_risk, violation_resolution);

  EXPECT_EQ(safer_bucket, 80u);
  EXPECT_EQ(selected_bucket, 81u);
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, true, safer_bucket, selected_bucket,
      208.6572113, 208.1932526, 60u, 90u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ZeroScaleClearanceTriggerRetainsItsPrecomputedDiagnosticRisk)
{
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::zero_scale_clearance_diagnostic(
      false, true, 0.80, 0.60),
    0.60);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::zero_scale_clearance_diagnostic(
      true, true, 0.80, 0.60),
    0.80);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::zero_scale_clearance_diagnostic(
      false, false, 0.80, 0.60),
    0.0);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintRejectsInvalidNormalization)
{
  EXPECT_THROW(
    ScorePlannerAdapter::clearance_bucket(-0.01, 0.10, 0.01),
    std::invalid_argument);
  EXPECT_THROW(
    ScorePlannerAdapter::clearance_bucket(0.50, 1.01, 0.01),
    std::invalid_argument);
  EXPECT_THROW(
    ScorePlannerAdapter::clearance_bucket(0.50, 0.10, 0.0),
    std::invalid_argument);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintRanksRiskBeforeWeightedScore)
{
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, false, 3u, 4u, 120.0, 100.0, 8u, 9u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, false, 5u, 4u, 10.0, 100.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, false, 4u, 4u, 99.0, 100.0, 10u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, false, 4u, 4u, 100.0, 100.0, 8u, 9u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintRanksGoalProgressBeforeWeightedScoreWithinRiskBand)
{
  EXPECT_FALSE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, false, 8u, 3u, 140.0, 100.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, true, 2u, 8u, 80.0, 140.0, 8u, 9u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, false, 8u, 0u, 10.0, 140.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, true, 0u, 8u, 180.0, 140.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, true, 7u, 8u, 150.0, 140.0, 8u, 9u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, true, 9u, 8u, 100.0, 140.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, false, 0u, 0u, 175.58, 113.35, 127u, 74u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_prefers_candidate(
      false, true, 0u, 0u, 80.0, 140.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_prefers_candidate(
      true, false, 0u, 0u, 100.0, 100.0, 8u, 9u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintAppliesMappedWallGuardBeforeDynamicRisk)
{
  EXPECT_TRUE(ScorePlannerAdapter::clearance_guard_prefers_candidate(
      0u, 2u, 8u, 3u, 150.0, 100.0));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_guard_prefers_candidate(
      3u, 1u, 2u, 8u, 80.0, 140.0));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_guard_prefers_candidate(
      0u, 0u, 7u, 8u, 150.0, 140.0));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintTriggerIsPairLocalInsteadOfSticky)
{
  EXPECT_FALSE(ScorePlannerAdapter::clearance_is_active_for_pair(
      false, 100.0, 2u, 0u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_is_active_for_pair(
      true, -1.0, 2u, 0u));
  EXPECT_FALSE(ScorePlannerAdapter::clearance_is_active_for_pair(
      true, 100.0, 0u, 0u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_is_active_for_pair(
      true, 100.0, 2u, 0u));
  EXPECT_TRUE(ScorePlannerAdapter::clearance_is_active_for_pair(
      true, 100.0, 0u, 2u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ClearanceConstraintRequiresProgressTowardThePathSubgoal)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.resize(3u);
  nav_2d_msgs::msg::Path2D path;
  path.poses.resize(2u);
  path.poses[1u].x = 1.0;
  geometry_msgs::msg::Pose2D subgoal;
  subgoal.x = 1.0;
  trajectory.poses[0u].theta = 0.4;
  trajectory.poses[1u].theta = 0.4;
  trajectory.poses[2u].theta = 0.4;
  trajectory.poses[1u].x = 0.09;
  EXPECT_FALSE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.10, 0.15));

  trajectory.poses[1u].x = 0.10;
  trajectory.poses[2u].x = 0.10;
  EXPECT_TRUE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.10, 0.15));

  trajectory.poses[1u].x = -0.10;
  trajectory.poses[2u].x = -0.10;
  EXPECT_FALSE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.10, 0.15));

  trajectory.poses[1u].x = 0.0;
  trajectory.poses[2u].theta = 0.20;
  EXPECT_TRUE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.10, 0.15));
  EXPECT_TRUE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.0, 0.15));

  trajectory.poses[2u].theta = 0.60;
  EXPECT_FALSE(ScorePlannerAdapter::has_meaningful_subgoal_progress(
      trajectory, path, subgoal, 0.10, 0.15));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  AvoidanceMotionUsesTheCompleteNativeRollout)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 6.0e-5;
  trajectory.velocity.theta = 1.0e-3;
  trajectory.poses.resize(3u);

  EXPECT_FALSE(ScorePlannerAdapter::has_observable_motion(
      trajectory, 5.0e-4, 5.0e-4));

  trajectory.poses[1u].theta = 2.0e-4;
  trajectory.poses[2u].theta = 0.70;
  EXPECT_TRUE(ScorePlannerAdapter::has_observable_motion(
      trajectory, 5.0e-4, 5.0e-4));

  trajectory.poses[2u].theta = 0.0;
  trajectory.poses[1u].x = 0.01;
  EXPECT_TRUE(ScorePlannerAdapter::has_observable_motion(
      trajectory, 5.0e-4, 5.0e-4));

  trajectory.poses[1u].x = 0.0;
  trajectory.poses[2u].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ScorePlannerAdapter::has_observable_motion(
      trajectory, 5.0e-4, 5.0e-4));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  HeadingOnlyRecoveryRequiresExecutableStopEvidence)
{
  EXPECT_FALSE(ScorePlannerAdapter::executable_progress(
      false, false, false, true, false));
  EXPECT_TRUE(ScorePlannerAdapter::executable_progress(
      false, false, false, true, true));
  EXPECT_TRUE(ScorePlannerAdapter::executable_progress(
      false, false, true, false, false));
  EXPECT_TRUE(ScorePlannerAdapter::executable_progress(
      false, true, false, false, false));
  EXPECT_TRUE(ScorePlannerAdapter::executable_progress(
      true, false, false, false, false));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  StopSequenceRotationMustBeFiniteAndObservable)
{
  std::vector<geometry_msgs::msg::Pose2D> poses(3u);
  poses[1u].theta = 2.0e-4;
  poses[2u].theta = -7.0e-4;
  EXPECT_TRUE(ScorePlannerAdapter::has_observable_rotation(poses, 5.0e-4));

  poses[2u].theta = -4.0e-4;
  EXPECT_FALSE(ScorePlannerAdapter::has_observable_rotation(poses, 5.0e-4));
  poses[2u].theta = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ScorePlannerAdapter::has_observable_rotation(poses, 5.0e-4));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  StopSequenceTranslationSeparatesApproachFromInPlaceRotation)
{
  std::vector<geometry_msgs::msg::Pose2D> poses(3u);
  poses[1u].theta = -0.02;
  poses[2u].theta = -0.10;
  EXPECT_FALSE(ScorePlannerAdapter::has_observable_translation(
      poses, 5.0e-4));

  poses[1u].x = 2.0e-4;
  poses[2u].x = 7.0e-4;
  EXPECT_TRUE(ScorePlannerAdapter::has_observable_translation(
      poses, 5.0e-4));

  poses[2u].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ScorePlannerAdapter::has_observable_translation(
      poses, 5.0e-4));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ExceptionalAvoidanceRetainsAnEstablishedTurnDirection)
{
  EXPECT_TRUE(ScorePlannerAdapter::preserves_turn(-0.001, 0.0));
  EXPECT_TRUE(ScorePlannerAdapter::preserves_turn(-0.001, -1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::preserves_turn(0.001, -1.0e-6));
  EXPECT_TRUE(ScorePlannerAdapter::preserves_turn(0.0, -1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::preserves_turn(
      std::numeric_limits<double>::quiet_NaN(), 0.1));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TerminalStopAssessmentUsesReferencePathTangentForOvershoot)
{
  geometry_msgs::msg::Pose2D goal;
  goal.x = 5.0;
  goal.y = 2.0;
  goal.theta = 1.2;
  std::vector<geometry_msgs::msg::Pose2D> stop_poses(1u);
  stop_poses.back().x = 5.10;
  stop_poses.back().y = 2.05;
  stop_poses.back().theta = 1.2;

  auto assessment = ScorePlannerAdapter::terminal_stop_assessment(
    stop_poses, goal, 0.0, 0.25, M_PI, 0.25);
  ASSERT_TRUE(assessment.available);
  EXPECT_TRUE(assessment.captures_goal);
  EXPECT_FALSE(assessment.crosses_terminal_limit);
  EXPECT_NEAR(assessment.longitudinal_error, 0.10, 1.0e-12);
  EXPECT_NEAR(assessment.lateral_error, 0.05, 1.0e-12);

  stop_poses.back().x = 5.26;
  assessment = ScorePlannerAdapter::terminal_stop_assessment(
    stop_poses, goal, 0.0, 0.25, M_PI, 0.25);
  EXPECT_FALSE(assessment.captures_goal);
  EXPECT_TRUE(assessment.crosses_terminal_limit);

  stop_poses.back().x = 5.0;
  stop_poses.back().y = 2.30;
  assessment = ScorePlannerAdapter::terminal_stop_assessment(
    stop_poses, goal, 0.0, 0.25, M_PI, 0.25);
  EXPECT_FALSE(assessment.captures_goal);
  EXPECT_FALSE(assessment.crosses_terminal_limit);
  EXPECT_NEAR(assessment.longitudinal_error, 0.0, 1.0e-12);
  EXPECT_NEAR(assessment.lateral_error, 0.30, 1.0e-12);

  // A curved path can put an earlier leg on the positive side of the
  // endpoint's tangent plane. It is not an endpoint overshoot while it remains
  // outside the Goal capture corridor.
  stop_poses.back().x = 5.30;
  assessment = ScorePlannerAdapter::terminal_stop_assessment(
    stop_poses, goal, 0.0, 0.25, M_PI, 0.25);
  EXPECT_FALSE(assessment.captures_goal);
  EXPECT_FALSE(assessment.crosses_terminal_limit);
  EXPECT_NEAR(assessment.longitudinal_error, 0.30, 1.0e-12);
  EXPECT_NEAR(assessment.lateral_error, 0.30, 1.0e-12);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  PositionOnlyGoalCaptureAcceptsRecordedYawErrorsAndOppositeHeading)
{
  geometry_msgs::msg::Pose2D goal;
  std::vector<geometry_msgs::msg::Pose2D> stop_poses(1u);
  stop_poses.back().x = 0.0561;
  for (const double yaw : {-M_PI, -0.4308, -0.2950, 0.0, M_PI}) {
    stop_poses.back().theta = yaw;
    const auto assessment = ScorePlannerAdapter::terminal_stop_assessment(
      stop_poses, goal, 0.0, 0.25, M_PI, 0.25);
    ASSERT_TRUE(assessment.available);
    EXPECT_TRUE(assessment.captures_goal) << "yaw=" << yaw;
  }
  for (const double radius : {0.1, 0.25, 0.4}) {
    stop_poses.back().x = radius;
    auto assessment = ScorePlannerAdapter::terminal_stop_assessment(
      stop_poses, goal, 0.0, radius, M_PI, radius);
    EXPECT_TRUE(assessment.captures_goal);
    stop_poses.back().x = radius + 0.001;
    assessment = ScorePlannerAdapter::terminal_stop_assessment(
      stop_poses, goal, 0.0, radius, M_PI, radius);
    EXPECT_FALSE(assessment.captures_goal);
  }
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TerminalPlanFallbackAppliesOnlyInsideGoalPositionTolerance)
{
  geometry_msgs::msg::Pose2D terminal_pose;
  terminal_pose.x = 0.7041669926966279;
  terminal_pose.y = -3.5416669017026834;
  geometry_msgs::msg::Pose2D stopped_pose;
  stopped_pose.x = 0.879495380832244;
  stopped_pose.y = -3.57449105893422;

  EXPECT_TRUE(ScorePlannerAdapter::terminal_plan_fallback(
      stopped_pose, terminal_pose, 0.25));
  stopped_pose.x = terminal_pose.x + 0.250001;
  stopped_pose.y = terminal_pose.y;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_plan_fallback(
      stopped_pose, terminal_pose, 0.25));
  EXPECT_FALSE(ScorePlannerAdapter::terminal_plan_fallback(
      stopped_pose, terminal_pose, 0.0));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TerminalGoalHoldRequiresObservedStopInsideTolerance)
{
  geometry_msgs::msg::Pose2D goal_pose;
  goal_pose.x = 5.0;
  geometry_msgs::msg::Pose2D stopped_pose = goal_pose;
  stopped_pose.x -= 0.25;
  nav_2d_msgs::msg::Twist2D velocity;

  EXPECT_TRUE(ScorePlannerAdapter::terminal_goal_hold(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  velocity.x = 0.010001;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_hold(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  velocity.x = 0.0;
  velocity.theta = 0.010001;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_hold(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  velocity.theta = 0.0;
  stopped_pose.x = goal_pose.x - 0.250001;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_hold(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TerminalGoalCaptureOutranksWeightedScoreWithoutChangingWeights)
{
  EXPECT_TRUE(ScorePlannerAdapter::terminal_prefers_candidate(
      true, false, 200.0, 10.0, 8u, 9u));
  EXPECT_FALSE(ScorePlannerAdapter::terminal_prefers_candidate(
      false, true, 1.0, 200.0, 8u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::terminal_prefers_candidate(
      true, true, 99.0, 100.0, 10u, 9u));
  EXPECT_TRUE(ScorePlannerAdapter::terminal_prefers_candidate(
      false, false, 100.0, 100.0, 8u, 9u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  MovingFirTerminalStopKeepsReplanningWithStableAndCorrectedLocalization)
{
  for (const double correction : {0.0, -0.6}) {
    SCOPED_TRACE(correction);
    const auto node = make_node(
    "fir_terminal_replan_test", true, true, false, 0.1, 1.0, 1.0, 0.05);
    node->declare_parameter("publish_zero_velocity", true);
    node->declare_parameter(
      "FollowPath.trajectory_generator_name", "dwb_plugins::StandardTrajectoryGenerator");
    node->declare_parameter("FollowPath.critics", std::vector<std::string>{"GoalDist"});
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_distance", 0.3);
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_yaw_tolerance", M_PI);

    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides({
        {"global_frame", "odom"}, {"plugins", std::vector<std::string>{}},
        {"filters", std::vector<std::string>{}}, {"track_unknown_space", false}});
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap->getLayeredCostmap()->resizeMap(200u, 200u, 0.05, -5.0, -5.0);
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "terminal_replan_test", true));

    ScorePlannerAdapter planner;
    planner.configure(node, kPluginName, tf, costmap);
    // This test isolates terminal commitment from weighted critic selection.
    auto generator = std::make_shared<FirTrajectoryGenerator>();
    generator->initialize(node, kPluginName);
    planner.set_test_components(generator, {});
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 0.3;
    goal.pose.orientation.w = 1.0;
    path.poses.push_back(goal);
    planner.setPlan(path);
    auto snapshot = make_observable_zero_snapshot(node->now());
    snapshot.current_state.velocity.x = 0.4;
    snapshot.current_state.linear_fir_history = {-0.1, -0.1};
    snapshot.current_state.angular_fir_history = {0.0, 0.0};
    snapshot.activation_state = snapshot.current_state;
    generator->set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
    geometry_msgs::msg::Pose2D pose;
    auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    const auto first = planner.run_terminal_core(pose, snapshot.current_state.velocity, results);
    ASSERT_GT(first.traj.velocity.x, 0.01);
    ASSERT_GT(results->twists.size(), 1u);

    const rclcpp::Time issued(1, 0, RCL_ROS_TIME);
    generator->commit_selected_command(first.traj.velocity, issued);
    f_dwa_controller::msg::CommandDispatch applied;
    applied.header.stamp = rclcpp::Time(1, 10000000, RCL_ROS_TIME);
    applied.has_sequence = true;
    applied.command.linear.x = first.traj.velocity.x;
    applied.command.angular.z = first.traj.velocity.theta;
    generator->observe_command_dispatch(applied);
    auto next = make_observable_zero_snapshot(rclcpp::Time(1, 50000000, RCL_ROS_TIME));
    next.current_state.velocity = first.traj.velocity;
    next.activation_state = next.current_state;
    generator->enrich_planning_snapshot(next);
    ASSERT_TRUE(next.valid);
    const auto history = next.activation_state.linear_fir_history;
    ASSERT_TRUE(std::any_of(history.begin(), history.end(), [](double x) {return x != 0.0;}));
    generator->set_planning_snapshot(std::make_shared<const PlanningSnapshot>(next));
    pose.x += first.traj.velocity.x * 0.05;
    pose.theta += first.traj.velocity.theta * 0.05;

    transform.transform.translation.x = correction;
    ASSERT_TRUE(tf->setTransform(transform, "terminal_replan_test", true));
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    const auto replanned = planner.run_terminal_core(pose, next.activation_state.velocity, results);
    EXPECT_GT(results->twists.size(), 1u);
    EXPECT_GT(replanned.traj.velocity.x, 0.01);
    EXPECT_FALSE(std::any_of(replanned.scores.begin(), replanned.scores.end(),
      [](const auto & score) {return score.name == "RetainedTerminalStop";}));
  // Replanning must not clear the actually dispatched FIR state.
    auto observed = make_observable_zero_snapshot(node->now());
    generator->enrich_planning_snapshot(observed);
    EXPECT_TRUE(observed.valid);
    EXPECT_EQ(observed.current_state.linear_fir_history, history);
    planner.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  StopAdmissibilityEnablesConfiguredTerminalStopRanking)
{
  EXPECT_TRUE(ScorePlannerAdapter::terminal_stop_scoring_enabled(
      false, true, 24.0, true));
  EXPECT_TRUE(ScorePlannerAdapter::terminal_stop_scoring_enabled(
      true, false, 24.0, true));
  EXPECT_FALSE(ScorePlannerAdapter::terminal_stop_scoring_enabled(
      false, false, 24.0, true));
  EXPECT_FALSE(ScorePlannerAdapter::terminal_stop_scoring_enabled(
      false, true, 0.0, true));
  EXPECT_FALSE(ScorePlannerAdapter::terminal_stop_scoring_enabled(
      false, true, 24.0, false));
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
  LocalVelocityCoreMatchesBaseCandidateOrderShortCircuitTieAndCommand)
{
  const std::vector<double> candidate_velocities{0.1, 0.2, 0.3, 0.4, 0.5};
  const std::vector<double> first_scores{0.0, 10.0, 0.0, 1.0, 4.0, 1.0};
  const std::vector<double> second_scores{0.0, 0.0, 0.0, 2.0, 99.0, 2.0};
  std::vector<std::string> base_trace;
  std::vector<std::string> local_trace;
  auto base_generator =
    std::make_shared<ScriptedVelocityGenerator>(candidate_velocities);
  auto local_generator =
    std::make_shared<ScriptedVelocityGenerator>(candidate_velocities);
  BaseCorePlannerAdapter base;
  base.set_test_components(
    base_generator,
    {std::make_shared<ScriptedScoreCritic>(
        "First", first_scores, 2, base_trace),
      std::make_shared<ScriptedScoreCritic>(
        "Second", second_scores, -1, base_trace)});
  ScorePlannerAdapter local;
  local.set_test_components(
    local_generator,
    {std::make_shared<ScriptedScoreCritic>(
        "First", first_scores, 2, local_trace),
      std::make_shared<ScriptedScoreCritic>(
        "Second", second_scores, -1, local_trace)});

  const auto base_best = base.run_core();
  const auto local_best = local.run_local_core();

  EXPECT_EQ(local_generator->generated_velocities(), candidate_velocities);
  EXPECT_EQ(
    local_generator->generated_velocities(),
    base_generator->generated_velocities());
  EXPECT_EQ(local_trace, base_trace);
  EXPECT_EQ(
    std::count(local_trace.begin(), local_trace.end(), "Second:4"), 0);
  EXPECT_EQ(
    std::count(local_trace.begin(), local_trace.end(), "Second:2"), 0);
  EXPECT_DOUBLE_EQ(local_best.total, base_best.total);
  EXPECT_DOUBLE_EQ(local_best.total, 3.0);
  EXPECT_DOUBLE_EQ(local_best.traj.velocity.x, base_best.traj.velocity.x);
  // Candidates 0.3 and 0.5 tie at 3.0. Base DWB retains the first, and the
  // local loop must retain the same command rather than changing the study.
  EXPECT_DOUBLE_EQ(local_best.traj.velocity.x, 0.3);
  EXPECT_EQ(
    trajectory_signature(local_best.traj),
    trajectory_signature(base_best.traj));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  LocalVelocityCoreMatchesBaseAllIllegalException)
{
  const std::vector<double> candidate_velocities{0.2};
  const std::vector<double> scores{0.0, 0.0, 0.0};
  std::vector<std::string> base_trace;
  std::vector<std::string> local_trace;
  auto base_generator =
    std::make_shared<ScriptedVelocityGenerator>(candidate_velocities);
  auto local_generator =
    std::make_shared<ScriptedVelocityGenerator>(candidate_velocities);
  BaseCorePlannerAdapter base;
  base.set_test_components(
    base_generator,
    {std::make_shared<ScriptedScoreCritic>(
        "Reject", scores, 2, base_trace)});
  ScorePlannerAdapter local;
  local.set_test_components(
    local_generator,
    {std::make_shared<ScriptedScoreCritic>(
        "Reject", scores, 2, local_trace)});

  std::string base_error;
  std::string local_error;
  try {
    (void)base.run_core();
    FAIL() << "base DWB unexpectedly accepted an illegal candidate";
  } catch (const dwb_core::NoLegalTrajectoriesException & exception) {
    base_error = exception.what();
  }
  try {
    (void)local.run_local_core();
    FAIL() << "local DWB unexpectedly accepted an illegal candidate";
  } catch (const dwb_core::NoLegalTrajectoriesException & exception) {
    local_error = exception.what();
  }

  EXPECT_EQ(local_trace, base_trace);
  EXPECT_EQ(local_error, base_error);
  EXPECT_EQ(
    local_generator->generated_velocities(),
    base_generator->generated_velocities());
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TotalOnlySweepPreservesCandidatesSignaturesAndCanonicalBest)
{
  const auto node = make_node("total_only_candidate_sweep_test");
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.23;
  start_pose.y = -0.11;
  start_pose.theta = 0.37;

  struct Candidate
  {
    std::size_t canonical_index;
    dwb_msgs::msg::Trajectory2D trajectory;
    std::vector<double> signature;
  };
  std::vector<Candidate> candidates;
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    const auto canonical_index =
      generator.active_candidate_canonical_index();
    ASSERT_TRUE(canonical_index.has_value());
    auto trajectory = generator.generateTrajectory(
      start_pose, current_velocity, command);
    candidates.push_back(
      Candidate{
        *canonical_index, trajectory, trajectory_signature(trajectory)});
  }
  ASSERT_EQ(candidates.size(), 165u);
  std::set<std::vector<double>> distinct_signatures;
  for (const auto & candidate : candidates) {
    distinct_signatures.insert(candidate.signature);
  }
  ASSERT_EQ(distinct_signatures.size(), candidates.size());

  geometry_msgs::msg::Pose2D early_target;
  early_target.x = 0.8;
  early_target.y = 0.05;
  early_target.theta = -0.25;
  geometry_msgs::msg::Pose2D final_target;
  final_target.x = 1.4;
  final_target.y = 0.28;
  final_target.theta = -0.35;
  std::vector<std::string> detailed_call_trace;
  std::vector<std::string> total_only_call_trace;
  auto detailed_first = std::make_shared<EndpointScoreCritic>(
    "FirstEndpointCritic", 1.8, early_target, detailed_call_trace);
  auto detailed_disabled =
    std::make_shared<FixedScoreCritic>("DisabledCritic", 0.0, 100.0);
  auto detailed_last = std::make_shared<EndpointScoreCritic>(
    "LastEndpointCritic", 0.6, final_target, detailed_call_trace);
  auto total_only_first = std::make_shared<EndpointScoreCritic>(
    "FirstEndpointCritic", 1.8, early_target, total_only_call_trace);
  auto total_only_disabled =
    std::make_shared<FixedScoreCritic>("DisabledCritic", 0.0, 100.0);
  auto total_only_last = std::make_shared<EndpointScoreCritic>(
    "LastEndpointCritic", 0.6, final_target, total_only_call_trace);
  ScorePlannerAdapter detailed_planner;
  detailed_planner.set_test_critics(
    {detailed_first, detailed_disabled, detailed_last}, true);
  ScorePlannerAdapter total_only_planner;
  total_only_planner.set_test_critics(
    {total_only_first, total_only_disabled, total_only_last}, true);

  double detailed_best_score = -1.0;
  double total_only_best_score = -1.0;
  std::size_t detailed_best_index =
    std::numeric_limits<std::size_t>::max();
  std::size_t total_only_best_index =
    std::numeric_limits<std::size_t>::max();
  std::size_t detailed_candidate_count = 0u;
  std::size_t total_only_candidate_count = 0u;
  for (const auto & candidate : candidates) {
    const auto detailed_score = detailed_planner.scoreTrajectory(
      candidate.trajectory, detailed_best_score);
    const auto total_only_score = total_only_planner.total_only_score(
      candidate.trajectory, total_only_best_score);
    ++detailed_candidate_count;
    ++total_only_candidate_count;

    EXPECT_DOUBLE_EQ(total_only_score.total, detailed_score.total);
    EXPECT_TRUE(total_only_score.scores.empty());
    EXPECT_EQ(total_only_score.scores.capacity(), 0u);
    EXPECT_EQ(
      trajectory_signature(candidate.trajectory), candidate.signature);

    if (detailed_best_score < 0.0 ||
      detailed_score.total < detailed_best_score ||
      (detailed_score.total == detailed_best_score &&
      candidate.canonical_index < detailed_best_index))
    {
      detailed_best_score = detailed_score.total;
      detailed_best_index = candidate.canonical_index;
    }
    if (total_only_best_score < 0.0 ||
      total_only_score.total < total_only_best_score ||
      (total_only_score.total == total_only_best_score &&
      candidate.canonical_index < total_only_best_index))
    {
      total_only_best_score = total_only_score.total;
      total_only_best_index = candidate.canonical_index;
    }
  }

  EXPECT_EQ(detailed_candidate_count, candidates.size());
  EXPECT_EQ(total_only_candidate_count, candidates.size());
  EXPECT_DOUBLE_EQ(total_only_best_score, detailed_best_score);
  EXPECT_EQ(total_only_best_index, detailed_best_index);
  EXPECT_NE(
    detailed_best_index, std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(total_only_call_trace, detailed_call_trace);
  EXPECT_EQ(detailed_first->call_count(), total_only_first->call_count());
  EXPECT_EQ(detailed_last->call_count(), total_only_last->call_count());
  EXPECT_LT(detailed_last->call_count(), detailed_first->call_count());
  EXPECT_EQ(detailed_disabled->call_count(), 0);
  EXPECT_EQ(total_only_disabled->call_count(), 0);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  MarkerOnlySweepKeepsCriticScoreStorageUnallocated)
{
  auto first = std::make_shared<FixedScoreCritic>(
    "FirstCriticWithANameLongEnoughToRequireStringStorage", 1.75, 0.5);
  auto disabled = std::make_shared<FixedScoreCritic>(
    "DisabledCriticWithANameLongEnoughToRequireStringStorage", 0.0, 9.0);
  auto last = std::make_shared<FixedScoreCritic>(
    "LastCriticWithANameLongEnoughToRequireStringStorage", 0.25, 0.75);
  ScorePlannerAdapter planner;
  planner.set_test_critics({first, disabled, last}, false);

  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 0.2;
  trajectory.poses.resize(4u);
  trajectory.time_offsets.resize(3u);
  const auto full_diagnostic = planner.scoreTrajectory(trajectory, -1.0);
  ASSERT_EQ(full_diagnostic.scores.size(), 3u);
  EXPECT_EQ(full_diagnostic.scores[0u].name, first->getName());
  EXPECT_EQ(full_diagnostic.scores[1u].name, disabled->getName());
  EXPECT_EQ(full_diagnostic.scores[2u].name, last->getName());

  constexpr std::size_t kCommonCandidateCount = 165u;
  for (std::size_t index = 0u; index < kCommonCandidateCount; ++index) {
    const auto marker_only = planner.total_only_score(trajectory, -1.0);
    EXPECT_DOUBLE_EQ(marker_only.total, full_diagnostic.total);
    EXPECT_TRUE(marker_only.scores.empty());
    // No CriticScore vector storage means no per-critic name/string payload
    // can be allocated on a marker-only control cycle.
    EXPECT_EQ(marker_only.scores.capacity(), 0u);
  }
  EXPECT_EQ(disabled->call_count(), 0u);
  EXPECT_EQ(first->call_count(), kCommonCandidateCount + 1u);
  EXPECT_EQ(last->call_count(), kCommonCandidateCount + 1u);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RealtimeCandidateMarkersColorLegalTrajectoriesByWeightedTotal)
{
  ScorePlannerAdapter planner;
  dwb_msgs::msg::LocalPlanEvaluation evaluation;
  evaluation.header.frame_id = "odom";
  evaluation.best_index = 0;

  const auto trajectory_score = [](const double total, const double y) {
      dwb_msgs::msg::TrajectoryScore score;
      score.total = total;
      score.traj.poses.resize(2u);
      score.traj.poses.back().x = 0.5;
      score.traj.poses.back().y = y;
      return score;
    };
  evaluation.twists.push_back(trajectory_score(1.0, 0.0));
  evaluation.twists.push_back(trajectory_score(2.0, 0.1));
  evaluation.twists.push_back(trajectory_score(10.0, 0.2));
  auto pruned = trajectory_score(1.5, 0.3);
  dwb_msgs::msg::CriticScore short_circuit;
  short_circuit.name = "__short_circuit__";
  pruned.scores.push_back(short_circuit);
  evaluation.twists.push_back(pruned);
  evaluation.twists.push_back(trajectory_score(-1.0, -0.1));

  const auto markers = planner.candidate_markers(evaluation);
  const auto valid = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [](const auto & marker) {
      return marker.ns == "dwb_candidates_valid";
    });
  const auto rejected = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [](const auto & marker) {
      return marker.ns == "dwb_candidates_rejected";
    });
  const auto selected = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [](const auto & marker) {
      return marker.ns == "dwb_candidate_selected";
    });

  ASSERT_NE(valid, markers.markers.end());
  ASSERT_NE(rejected, markers.markers.end());
  ASSERT_NE(selected, markers.markers.end());
  ASSERT_EQ(valid->points.size(), 6u);
  ASSERT_EQ(valid->colors.size(), valid->points.size());
  // Legal candidates are ranked by every finite accumulated cost. A
  // short-circuited score remains a lower bound, so the 1.5 candidate is blue,
  // the 2.0 candidate is cyan and the 10.0 candidate is yellow.
  EXPECT_LT(valid->colors.front().r, valid->colors.front().b);
  EXPECT_LT(valid->colors.front().g, valid->colors.front().b);
  EXPECT_GT(valid->colors[2u].r, valid->colors[2u].b);
  EXPECT_GT(valid->colors[2u].g, valid->colors[2u].b);
  EXPECT_GT(valid->colors.front().b, valid->colors[2u].b);
  EXPECT_LT(valid->colors.back().r, valid->colors.back().b);
  EXPECT_GT(valid->colors.back().b, valid->colors.front().b);
  EXPECT_FLOAT_EQ(valid->colors.front().a, 0.36F);
  EXPECT_TRUE(rejected->colors.empty());
  EXPECT_GT(rejected->color.r, rejected->color.g);
  EXPECT_GT(rejected->color.r, rejected->color.b);
  EXPECT_GT(selected->color.g, selected->color.r);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  CandidateMarkerDecimationPreservesSelectedEndpointsColorsAndEvaluation)
{
  ScorePlannerAdapter planner;
  dwb_msgs::msg::LocalPlanEvaluation evaluation;
  evaluation.best_index = 93;
  for (size_t candidate = 0u; candidate < 190u; ++candidate) {
    dwb_msgs::msg::TrajectoryScore score;
    score.total = candidate >= 120u ? -1.0 : static_cast<double>(candidate + 1u);
    if (candidate == 93u) {score.total = 0.0;}
    for (size_t step = 0u; step < 51u; ++step) {
      geometry_msgs::msg::Pose2D pose;
      pose.x = 0.05 * step;
      pose.y = static_cast<double>(candidate);
      score.traj.poses.push_back(pose);
    }
    evaluation.twists.push_back(score);
  }
  const auto original = evaluation;
  const auto markers = planner.candidate_markers(evaluation);
  EXPECT_EQ(evaluation, original);
  ASSERT_EQ(markers.markers.size(), 5u);
  const auto & valid = markers.markers[1];
  const auto & rejected = markers.markers[2];
  const auto & selected = markers.markers[3];
  ASSERT_EQ(valid.points.size(), 24u * 30u);
  ASSERT_EQ(rejected.points.size(), 23u * 30u);
  ASSERT_EQ(valid.colors.size(), valid.points.size());
  ASSERT_EQ(selected.points.size(), 51u);
  for (size_t step = 0u; step < selected.points.size(); ++step) {
    EXPECT_DOUBLE_EQ(selected.points[step].x, original.twists[93].traj.poses[step].x);
    EXPECT_DOUBLE_EQ(selected.points[step].y, 93.0);
  }
  for (const auto * marker : {&valid, &rejected}) {
    for (size_t start = 0u; start < marker->points.size(); start += 30u) {
      EXPECT_DOUBLE_EQ(marker->points[start].x, 0.0);
      EXPECT_DOUBLE_EQ(marker->points[start + 29u].x, 2.5);
      EXPECT_NE(marker->points[start].y, 93.0);
    }
  }
  EXPECT_GT(valid.colors.front().b, valid.colors.front().r);
  EXPECT_GT(valid.colors.back().r, valid.colors.back().b);
  EXPECT_GT(rejected.color.r, rejected.color.g);
  EXPECT_NE(markers.markers[4].text.find("120/190 valid"), std::string::npos);
  EXPECT_NE(markers.markers[4].text.find("shown: 48"), std::string::npos);
  EXPECT_NEAR(rclcpp::Duration(valid.lifetime).seconds(), 0.3, 1.0e-9);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  CandidateMarkerBudgetHandlesEmptySingleClassAndSmallBanks)
{
  ScorePlannerAdapter planner;
  const dwb_msgs::msg::LocalPlanEvaluation empty;
  EXPECT_EQ(planner.candidate_markers(empty).markers[1].points.size(), 0u);
  for (const bool legal : {false, true}) {
    for (const size_t count : {1u, 100u}) {
      const auto budget = std::min<size_t>(count, 48u);
      dwb_msgs::msg::LocalPlanEvaluation evaluation;
      evaluation.best_index = legal ? static_cast<int>(count / 2u) : -1;
      for (size_t index = 0u; index < count; ++index) {
        dwb_msgs::msg::TrajectoryScore score;
        score.total = legal ? 1.0 : -1.0;
        score.traj.poses.resize(3u);
        score.traj.poses[1u].x = 0.5;
        score.traj.poses[2u].x = 1.0;
        evaluation.twists.push_back(score);
      }
      const auto markers = planner.candidate_markers(evaluation);
      EXPECT_EQ(markers.markers[3].points.size(), legal ? 3u : 0u);
      EXPECT_EQ(
        markers.markers[1].points.size() + markers.markers[2].points.size(),
        (budget - (legal ? 1u : 0u)) * 4u);
    }
  }
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  MarkerOnlyPayloadProducesBitIdenticalPublicMarkers)
{
  ScorePlannerAdapter planner;
  dwb_msgs::msg::LocalPlanEvaluation full;
  full.header.frame_id = "odom";
  full.header.stamp.sec = 41;
  full.header.stamp.nanosec = 37u;
  full.best_index = 0;
  full.worst_index = 0;

  dwb_msgs::msg::TrajectoryScore selected;
  selected.total = 1.25;
  selected.traj.poses.resize(3u);
  selected.traj.poses[0u].x = 0.1;
  selected.traj.poses[1u].x = 0.2;
  selected.traj.poses[2u].x = 0.3;
  dwb_msgs::msg::CriticScore selected_detail;
  selected_detail.name = "PathAlign";
  selected_detail.scale = 8.0;
  selected_detail.raw_score = 0.125;
  selected.scores.push_back(selected_detail);
  full.twists.push_back(selected);

  dwb_msgs::msg::TrajectoryScore rejected;
  rejected.total = -1.0;
  rejected.traj.poses.resize(3u);
  rejected.traj.poses[0u].y = 0.1;
  rejected.traj.poses[1u].y = 0.2;
  rejected.traj.poses[2u].y = 0.3;
  dwb_msgs::msg::CriticScore rejection;
  rejection.name = "ObstacleFootprint";
  rejection.raw_score = -1.0;
  rejected.scores.push_back(rejection);
  dwb_msgs::msg::CriticScore rejection_detail;
  rejection_detail.name = "__rejection_detail__:collision";
  rejected.scores.push_back(rejection_detail);
  dwb_msgs::msg::CriticScore native_detail;
  native_detail.name = "__candidate_native__:canonical_index=9";
  rejected.scores.push_back(native_detail);
  full.twists.push_back(rejected);

  auto marker_only = full;
  marker_only.twists[0u].scores.clear();
  marker_only.twists[1u].scores.erase(
    marker_only.twists[1u].scores.begin() + 1,
    marker_only.twists[1u].scores.end());

  const auto full_markers = planner.candidate_markers(full);
  const auto lean_markers = planner.candidate_markers(marker_only);
  EXPECT_EQ(lean_markers, full_markers);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  DiagnosticBacklogPreservesFullEvaluationsAndCoalescesOnlyStaleMarkers)
{
  using Publication = ScorePlannerAdapter::TestDiagnosticPublication;
  auto first_full =
    std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  auto stale_marker =
    std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  auto second_full =
    std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  auto latest_marker =
    std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  first_full->header.stamp.sec = 1;
  stale_marker->header.stamp.sec = 2;
  second_full->header.stamp.sec = 3;
  latest_marker->header.stamp.sec = 4;

  std::deque<Publication> publications{
    Publication{first_full, true, true},
    Publication{stale_marker, false, true},
    Publication{second_full, true, true}};
  EXPECT_EQ(
    ScorePlannerAdapter::maximum_pending_full_evaluations(), 2u);
  EXPECT_TRUE(ScorePlannerAdapter::full_evaluation_capacity(0u));
  EXPECT_TRUE(ScorePlannerAdapter::full_evaluation_capacity(1u));
  EXPECT_FALSE(ScorePlannerAdapter::full_evaluation_capacity(2u));
  ASSERT_TRUE(ScorePlannerAdapter::coalesce_stale_marker(
      publications, Publication{latest_marker, false, true}));
  ASSERT_EQ(publications.size(), 3u);
  EXPECT_EQ(publications[0u].evaluation, first_full);
  EXPECT_TRUE(publications[0u].publish_full_evaluation);
  EXPECT_EQ(publications[1u].evaluation, second_full);
  EXPECT_TRUE(publications[1u].publish_full_evaluation);
  EXPECT_EQ(publications[2u].evaluation, latest_marker);
  EXPECT_FALSE(publications[2u].publish_full_evaluation);
  EXPECT_EQ(
    std::count_if(
      publications.begin(), publications.end(),
      [](const Publication & publication) {
        return publication.publish_full_evaluation;
      }),
    2);

  const auto third_full =
    std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  EXPECT_FALSE(ScorePlannerAdapter::coalesce_stale_marker(
      publications, Publication{third_full, true, true}));
  EXPECT_EQ(publications.size(), 3u);
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
  ActiveCandidateDiagnosticsExposeNativeInputAndIterationState)
{
  const auto node = make_node("active_candidate_diagnostics_test");
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D current_velocity;
  current_velocity.x = 0.2;
  current_velocity.theta = -0.1;
  generator.startNewIteration(current_velocity);

  ASSERT_FALSE(generator.active_candidate_diagnostics().has_value());
  const auto command = generator.nextTwist();
  const auto diagnostics = generator.active_candidate_diagnostics();
  ASSERT_TRUE(diagnostics.has_value());
  EXPECT_EQ(diagnostics->canonical_index, 0u);
  // J-DWA intentionally starts from the previously dispatched native state,
  // not the odometry argument. No command has been dispatched in this test,
  // so the observable ledger is the reset zero state.
  EXPECT_DOUBLE_EQ(diagnostics->initial_linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics->initial_angular_velocity, 0.0);
  EXPECT_TRUE(std::isfinite(diagnostics->linear_native_input));
  EXPECT_TRUE(std::isfinite(diagnostics->angular_native_input));
  EXPECT_TRUE(diagnostics->first_command_state.valid);
  EXPECT_DOUBLE_EQ(
    diagnostics->first_command_state.command_velocity.x, command.x);
  EXPECT_DOUBLE_EQ(
    diagnostics->first_command_state.command_velocity.theta,
    command.theta);
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

TEST_F(NativeInputTrajectoryGeneratorTest, TrialResetReloadsPredictionTime)
{
  const auto node = make_node("runtime_prediction_time_test");
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  const auto update = node->set_parameter(
    rclcpp::Parameter("FollowPath.sim_time", 1.2));
  ASSERT_TRUE(update.successful);
  generator.reset_trial_state();

  nav_2d_msgs::msg::Twist2D current_velocity;
  generator.startNewIteration(current_velocity);
  ASSERT_TRUE(generator.hasMoreTwists());
  const auto command = generator.nextTwist();
  geometry_msgs::msg::Pose2D start_pose;
  const auto trajectory = generator.generateTrajectory(
    start_pose, current_velocity, command);
  EXPECT_EQ(trajectory.poses.size(), 42u);
  ASSERT_FALSE(trajectory.time_offsets.empty());
  const auto & final_time = trajectory.time_offsets.back();
  EXPECT_NEAR(
    final_time.sec + final_time.nanosec * 1.0e-9, 1.2, 1.0e-12);
}

TEST_F(NativeInputTrajectoryGeneratorTest, TrialResetReloadsSamplingAndRejectsInvalidCounts)
{
  const auto node = make_node("runtime_sampling_test", true, false, false, 0.10,
      0.8, 0.8, 0.05);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_EQ(generator.candidate_count(), 165u);
  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.vx_samples", 13),
      rclcpp::Parameter("FollowPath.vtheta_samples", 19),
      rclcpp::Parameter("FollowPath.fir_prediction_pulse_durations",
      std::vector<double>{0.20})}).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_EQ(generator.candidate_count(), 494u);

  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 0)).successful);
  EXPECT_THROW(generator.reset_trial_state(), std::invalid_argument);
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.candidate_count(), 494u);
}

TEST_F(NativeInputTrajectoryGeneratorTest, TrialResetReloadsPredictionInputMode)
{
  const auto node = make_node(
    "runtime_prediction_input_test", true, false, false, 0.0,
    0.8, 0.8, 0.05);
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_TRUE(generator.hasMoreTwists());
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_FALSE(generator.active_candidate_diagnostics()->uses_recovery);

  ASSERT_TRUE(node->set_parameter(
      rclcpp::Parameter("FollowPath.native_input_recovery", true)).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_TRUE(generator.hasMoreTwists());
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_TRUE(generator.active_candidate_diagnostics()->uses_recovery);

  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.native_input_recovery", false),
      rclcpp::Parameter("FollowPath.native_input_pulse_duration", 0.2),
    }).successful);
  generator.reset_trial_state();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_TRUE(generator.hasMoreTwists());
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_FALSE(generator.active_candidate_diagnostics()->uses_recovery);
  EXPECT_DOUBLE_EQ(
    generator.active_candidate_diagnostics()->linear_prediction_input_duration,
    0.2);

  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
      "FollowPath.native_input_pulse_duration", -0.1)).successful);
  EXPECT_THROW(generator.reset_trial_state(), std::invalid_argument);
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_TRUE(generator.hasMoreTwists());
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_DOUBLE_EQ(
    generator.active_candidate_diagnostics()->linear_prediction_input_duration,
    0.2);
}

TEST_F(NativeInputTrajectoryGeneratorTest, ExpandedFirBankMatchesScalarRolloutsWhileMoving)
{
  const auto bank_node = make_node("expanded_fir_bank", true, false, false, 0.10,
      0.8, 0.8, 0.05);
  FirTrajectoryGenerator bank;
  bank.initialize(bank_node, kPluginName);
  ASSERT_TRUE(bank_node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.vx_samples", 13),
      rclcpp::Parameter("FollowPath.vtheta_samples", 19),
      rclcpp::Parameter("FollowPath.sim_time", 2.5),
      rclcpp::Parameter("FollowPath.fir_prediction_pulse_durations",
      std::vector<double>{0.20})}).successful);
  bank.reset_trial_state();
  for (const double speed : {0.0, 0.4, 0.75}) {
    auto snapshot = make_observable_zero_snapshot(bank_node->now());
    snapshot.activation_state.velocity.x = speed;
    snapshot.activation_state.velocity.theta = -0.2;
    snapshot.activation_state.linear_fir_history = {0.02, -0.03};
    snapshot.activation_state.angular_fir_history = {-0.08, 0.03};
    const auto shared = std::make_shared<const PlanningSnapshot>(snapshot);
    bank.set_planning_snapshot(shared);
    bank.startNewIteration(snapshot.activation_state.velocity);
    std::size_t count = 0;
    for (const double duration : {0.10, 0.20}) {
      const auto scalar_node = make_node("expanded_fir_scalar", true, false, false,
          duration, 0.8, 0.8, 0.05);
      FirTrajectoryGenerator scalar;
      scalar.initialize(scalar_node, kPluginName);
      ASSERT_TRUE(scalar_node->set_parameters_atomically({
          rclcpp::Parameter("FollowPath.vx_samples", 13),
          rclcpp::Parameter("FollowPath.vtheta_samples", 19),
          rclcpp::Parameter("FollowPath.sim_time", 2.5)}).successful);
      scalar.reset_trial_state();
      scalar.set_planning_snapshot(shared);
      scalar.startNewIteration(snapshot.activation_state.velocity);
      while (scalar.hasMoreTwists()) {
        ASSERT_TRUE(bank.hasMoreTwists());
        const auto expected = scalar.nextTwist();
        const auto actual = bank.nextTwist();
        ASSERT_EQ(actual, expected);
        const auto trajectory = bank.generateTrajectory(geometry_msgs::msg::Pose2D(),
            snapshot.activation_state.velocity, actual);
        EXPECT_EQ(trajectory, scalar.generateTrajectory(geometry_msgs::msg::Pose2D(),
            snapshot.activation_state.velocity, expected));
        EXPECT_EQ(bank.active_candidate_command_state()->linear_fir_history,
          scalar.active_candidate_command_state()->linear_fir_history);
        EXPECT_EQ(bank.active_candidate_command_state()->angular_fir_history,
          scalar.active_candidate_command_state()->angular_fir_history);
        ASSERT_FALSE(trajectory.time_offsets.empty());
        EXPECT_NEAR(rclcpp::Duration(trajectory.time_offsets.back()).seconds(), 2.5, 1.1e-9);
        ++count;
      }
    }
    EXPECT_FALSE(bank.hasMoreTwists());
    EXPECT_EQ(count, 494u);
  }
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
  VdwaSamplingReloadsOnlyAtTrialReset)
{
  const auto node = make_node("v_dwa_sampling_reset_test");
  VLimitedAccelTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D velocity;
  velocity.x = 0.30;
  velocity.theta = 0.30;
  const auto count = [&]() {
      generator.startNewIteration(velocity);
      size_t candidates = 0u;
      while (generator.hasMoreTwists()) {
        generator.nextTwist();
        ++candidates;
      }
      return candidates;
    };
  EXPECT_EQ(count(), 165u);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 10)).successful);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.vtheta_samples", 19)).successful);
  EXPECT_EQ(count(), 165u);
  generator.reset();
  EXPECT_EQ(count(), 190u);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 0)).successful);
  EXPECT_THROW(generator.reset(), std::invalid_argument);
  EXPECT_EQ(count(), 190u);
}

TEST_F(NativeInputTrajectoryGeneratorTest, VdwaRuntimeLimitsReachGuiSpeed)
{
  const auto node = make_node("v_runtime_limits", true, false, false, 0.0, 0.2, 0.2, 0.05);
  VLimitedAccelTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.max_vel_x", 0.8),
      rclcpp::Parameter("FollowPath.max_speed_xy", 0.8),
      rclcpp::Parameter("FollowPath.max_vel_theta", 0.8),
      rclcpp::Parameter("FollowPath.acc_lim_x", 1.6),
      rclcpp::Parameter("FollowPath.decel_lim_x", -1.6),
      rclcpp::Parameter("FollowPath.acc_lim_theta", 1.6),
      rclcpp::Parameter("FollowPath.decel_lim_theta", -1.6),
      rclcpp::Parameter("FollowPath.sim_time", 2.5)}).successful);
  generator.reset();
  nav_2d_msgs::msg::Twist2D velocity;
  for (int cycle = 0; cycle < 12; ++cycle) {
    generator.startNewIteration(velocity);
    ASSERT_TRUE(generator.hasMoreTwists());
    nav_2d_msgs::msg::Twist2D selected;
    while (generator.hasMoreTwists()) {
      const auto candidate = generator.nextTwist();
      EXPECT_LE(candidate.x, 0.8 + 1e-9);
      EXPECT_LE(std::abs(candidate.theta), 0.8 + 1e-9);
      EXPECT_LE(std::abs(candidate.x - velocity.x), 1.6 * 0.05 + 1e-9);
      EXPECT_LE(std::abs(candidate.theta - velocity.theta), 1.6 * 0.05 + 1e-9);
      if (candidate.x > selected.x ||
        (candidate.x == selected.x && candidate.theta > selected.theta))
      {
        selected = candidate;
      }
    }
    velocity = selected;
  }
  EXPECT_NEAR(velocity.x, 0.8, 1e-9);
  EXPECT_NEAR(velocity.theta, 0.8, 1e-9);
  const auto trajectory = generator.generateTrajectory(
    geometry_msgs::msg::Pose2D(), velocity, velocity);
  ASSERT_FALSE(trajectory.time_offsets.empty());
  EXPECT_NEAR(rclcpp::Duration(trajectory.time_offsets.back()).seconds(), 2.5, 1e-8);
  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.max_vel_x", 0.1),
      rclcpp::Parameter("FollowPath.max_speed_xy", 0.1),
      rclcpp::Parameter("FollowPath.max_vel_theta", 0.1)}).successful);
  generator.reset();
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());
  ASSERT_TRUE(generator.hasMoreTwists());
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    EXPECT_LE(candidate.x, 0.1 + 1e-9);
    EXPECT_LE(std::abs(candidate.theta), 0.1 + 1e-9);
  }
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  VdwaWindowContinuesFromCorrelatedActivationCommand)
{
  const auto node = make_node(
    "v_dwa_command_window_test", true, false, false, 0.0,
    0.6, 0.6, 0.05);
  VLimitedAccelTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  PlanningSnapshot snapshot = make_observable_zero_snapshot(node->now());
  snapshot.activation_state.velocity.x = 0.20;
  snapshot.activation_state.velocity.theta = 0.10;
  snapshot.activation_state.native_command_velocity.x = 0.60;
  snapshot.activation_state.native_command_velocity.theta = -0.20;
  snapshot.activation_state.native_command_velocity_valid = true;
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));

  generator.startNewIteration(snapshot.activation_state.velocity);
  ASSERT_TRUE(generator.hasMoreTwists());
  double minimum_linear_velocity = std::numeric_limits<double>::infinity();
  double maximum_linear_velocity = -std::numeric_limits<double>::infinity();
  double minimum_angular_velocity = std::numeric_limits<double>::infinity();
  double maximum_angular_velocity = -std::numeric_limits<double>::infinity();
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    minimum_linear_velocity = std::min(minimum_linear_velocity, command.x);
    maximum_linear_velocity = std::max(maximum_linear_velocity, command.x);
    minimum_angular_velocity = std::min(
      minimum_angular_velocity, command.theta);
    maximum_angular_velocity = std::max(
      maximum_angular_velocity, command.theta);
  }

  EXPECT_NEAR(minimum_linear_velocity, 0.54, 1.0e-12);
  EXPECT_NEAR(maximum_linear_velocity, 0.60, 1.0e-12);
  EXPECT_NEAR(minimum_angular_velocity, -0.2785, 1.0e-12);
  EXPECT_NEAR(maximum_angular_velocity, -0.1215, 1.0e-12);
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
  // The physical plant may still lag the robot-facing command.  Native
  // feasibility must continue from the correlated command state, not acquire
  // fictitious acceleration authority from that lag.
  after_dispatch.activation_state.velocity.x = 0.5;
  generator.enrich_planning_snapshot(after_dispatch);
  EXPECT_TRUE(after_dispatch.valid);
  EXPECT_TRUE(after_dispatch.current_state.native_state_valid);
  EXPECT_DOUBLE_EQ(after_dispatch.activation_state.velocity.x, 0.5);
  EXPECT_DOUBLE_EQ(
    after_dispatch.activation_state.native_command_velocity.x, command.x);
  EXPECT_DOUBLE_EQ(
    after_dispatch.activation_state.native_command_velocity.theta,
    command.theta);
  EXPECT_TRUE(
    after_dispatch.activation_state.native_command_velocity_valid);
  EXPECT_NEAR(
    after_dispatch.current_state.linear_acceleration,
    selected_state->linear_state.acceleration, 1.0e-12);
  EXPECT_EQ(
    after_dispatch.current_state.linear_fir_history,
    selected_state->linear_fir_history);

  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(after_dispatch));
  generator.startNewIteration(after_dispatch.activation_state.velocity);
  ASSERT_TRUE(generator.hasMoreTwists());
  generator.nextTwist();
  const auto diagnostics = generator.active_candidate_diagnostics();
  ASSERT_TRUE(diagnostics.has_value());
  EXPECT_DOUBLE_EQ(diagnostics->initial_linear_velocity, command.x);
  EXPECT_DOUBLE_EQ(diagnostics->initial_angular_velocity, command.theta);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TerminalControllerStopClosesCapturedFirState)
{
  const auto node = make_node(
    "fir_terminal_controller_stop_test", true, true, false, 0.0,
    1.2, 1.57, 0.05);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  f_dwa_controller::msg::CommandDispatch terminal_stop;
  terminal_stop.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  terminal_stop.has_sequence = true;
  ASSERT_TRUE(generator.observe_terminal_controller_stop(terminal_stop));

  PlanningSnapshot snapshot = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.current_state.native_state_valid);
  EXPECT_DOUBLE_EQ(snapshot.current_state.linear_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.current_state.angular_acceleration, 0.0);
  EXPECT_TRUE(std::all_of(
      snapshot.current_state.linear_fir_history.begin(),
      snapshot.current_state.linear_fir_history.end(),
      [](const double value) {return value == 0.0;}));
  EXPECT_TRUE(std::all_of(
      snapshot.current_state.angular_fir_history.begin(),
      snapshot.current_state.angular_fir_history.end(),
      [](const double value) {return value == 0.0;}));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RepeatedJerkTerminalGoalHoldsPreserveObservableNativeState)
{
  const auto node = make_node(
    "jerk_terminal_goal_hold_test", true, true, false, 0.0,
    1.2, 1.57, 0.05);
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot initial = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(initial);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(initial));
  nav_2d_msgs::msg::Twist2D zero_velocity;
  generator.startNewIteration(zero_velocity);

  nav_2d_msgs::msg::Twist2D nonzero_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  nonzero_state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (candidate.x > 1.0e-12) {
      nonzero_command = candidate;
      nonzero_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(nonzero_state.has_value());

  generator.select_command_for_dispatch(nonzero_state);
  generator.commit_selected_command(
    nonzero_command, rclcpp::Time(1, 0, RCL_ROS_TIME));
  f_dwa_controller::msg::CommandDispatch applied;
  applied.header.stamp = rclcpp::Time(1, 10000000, RCL_ROS_TIME);
  applied.command.linear.x = nonzero_command.x;
  applied.command.angular.z = nonzero_command.theta;
  applied.has_sequence = true;
  generator.observe_command_dispatch(applied);

  ASSERT_TRUE(generator.commit_expected_controller_stop(
      rclcpp::Time(1, 20000000, RCL_ROS_TIME)));
  ASSERT_TRUE(generator.commit_expected_controller_stop(
      rclcpp::Time(1, 30000000, RCL_ROS_TIME)));
  generator.reset();

  f_dwa_controller::msg::CommandDispatch first_hold;
  first_hold.header.stamp = rclcpp::Time(1, 40000000, RCL_ROS_TIME);
  first_hold.has_sequence = true;
  generator.observe_command_dispatch(first_hold);
  f_dwa_controller::msg::CommandDispatch second_hold = first_hold;
  second_hold.header.stamp = rclcpp::Time(1, 50000000, RCL_ROS_TIME);
  generator.observe_command_dispatch(second_hold);

  ASSERT_TRUE(generator.commit_observed_controller_stop_before_pending(
      rclcpp::Time(1, 60000000, RCL_ROS_TIME)));
  f_dwa_controller::msg::CommandDispatch controller_stop = first_hold;
  controller_stop.header.stamp = rclcpp::Time(1, 60000000, RCL_ROS_TIME);
  generator.observe_command_dispatch(controller_stop);

  PlanningSnapshot observed = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(observed);
  ASSERT_TRUE(observed.valid);
  EXPECT_TRUE(observed.current_state.native_state_valid);
  EXPECT_DOUBLE_EQ(
    observed.current_state.native_command_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(
    observed.current_state.native_command_velocity.theta, 0.0);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirDispatchSkipsControllerResultsThatWereNeverPublished)
{
  const auto node = make_node("fir_unpublished_result_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot snapshot = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());

  nav_2d_msgs::msg::Twist2D first_command;
  nav_2d_msgs::msg::Twist2D published_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  first_state;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  published_state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    const auto state = generator.active_candidate_command_state();
    if (!state.has_value()) {
      continue;
    }
    if (!first_state.has_value()) {
      first_command = candidate;
      first_state = state;
    } else {
      if (std::abs(candidate.x - first_command.x) > 1.0e-12 ||
        std::abs(candidate.theta - first_command.theta) > 1.0e-12)
      {
        published_command = candidate;
        published_state = state;
        break;
      }
    }
  }
  ASSERT_TRUE(first_state.has_value());
  ASSERT_TRUE(published_state.has_value());

  generator.select_command_for_dispatch(first_state);
  generator.commit_selected_command(first_command, node->now());
  generator.select_command_for_dispatch(published_state);
  generator.commit_selected_command(published_command, node->now());

  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = published_command.x;
  dispatch.command.angular.z = published_command.theta;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch, false, 1u);

  PlanningSnapshot observed = make_observable_zero_snapshot(node->now());
  observed.current_state.velocity = published_command;
  observed.activation_state.velocity = published_command;
  generator.enrich_planning_snapshot(observed);
  EXPECT_TRUE(observed.valid);
  EXPECT_TRUE(observed.current_state.native_state_valid);
  EXPECT_EQ(
    observed.current_state.linear_fir_history,
    published_state->linear_fir_history);
  EXPECT_EQ(
    observed.current_state.angular_fir_history,
    published_state->angular_fir_history);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  JerkExpectedControllerStopKeepsRepeatedFailureZerosInFifoOrder)
{
  const auto node = make_node("jerk_expected_stop_fifo_test", true, true);
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  nav_2d_msgs::msg::Twist2D odom_velocity;
  generator.startNewIteration(odom_velocity);

  nav_2d_msgs::msg::Twist2D nonzero_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  nonzero_state;
  while (generator.hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      nonzero_command = candidate;
      nonzero_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(nonzero_state.has_value());

  const rclcpp::Time issued_at = node->now();
  generator.select_command_for_dispatch(nonzero_state);
  generator.commit_selected_command(nonzero_command, issued_at);
  ASSERT_TRUE(generator.commit_expected_controller_stop(issued_at));
  ASSERT_TRUE(generator.commit_expected_controller_stop(issued_at));

  PlanningSnapshot retry_snapshot =
    make_observable_zero_snapshot(node->now());
  retry_snapshot.committed_commands.push_back(
    ScheduledCommand{issued_at, nonzero_command, false});
  retry_snapshot.committed_commands.push_back(
    ScheduledCommand{
      issued_at, nav_2d_msgs::msg::Twist2D(), true});
  retry_snapshot.committed_commands.push_back(
    ScheduledCommand{
      issued_at, nav_2d_msgs::msg::Twist2D(), true});
  generator.enrich_planning_snapshot(retry_snapshot);
  ASSERT_TRUE(retry_snapshot.valid);
  EXPECT_TRUE(retry_snapshot.activation_state.native_state_valid);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.activation_state.linear_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.activation_state.angular_acceleration, 0.0);

  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = nonzero_command.x;
  dispatch.command.angular.z = nonzero_command.theta;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);
  dispatch.header.stamp = node->now();
  dispatch.command = geometry_msgs::msg::Twist();
  generator.observe_command_dispatch(dispatch);
  dispatch.header.stamp = node->now();
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot after_dispatch =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(after_dispatch);
  EXPECT_TRUE(after_dispatch.valid);
  EXPECT_TRUE(after_dispatch.current_state.native_state_valid);
  EXPECT_DOUBLE_EQ(after_dispatch.current_state.linear_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(after_dispatch.current_state.angular_acceleration, 0.0);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirFailureStopPreservesHistoryAndPostBarrierMetadata)
{
  const auto node = make_node("fir_failure_stop_retry_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot initial_snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(initial_snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(initial_snapshot));
  nav_2d_msgs::msg::Twist2D odom_velocity;
  generator.startNewIteration(odom_velocity);

  nav_2d_msgs::msg::Twist2D initial_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  initial_state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      initial_command = candidate;
      initial_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(initial_state.has_value());

  const rclcpp::Time issued_at = node->now();
  generator.select_command_for_dispatch(initial_state);
  generator.commit_selected_command(initial_command, issued_at);
  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = initial_command.x;
  dispatch.command.angular.z = initial_command.theta;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);

  ASSERT_TRUE(generator.commit_expected_controller_stop(node->now()));
  PlanningSnapshot retry_snapshot =
    make_observable_zero_snapshot(node->now());
  retry_snapshot.current_state.velocity = initial_command;
  retry_snapshot.activation_state.velocity = nav_2d_msgs::msg::Twist2D();
  retry_snapshot.committed_commands.push_back(
    ScheduledCommand{
      node->now(), nav_2d_msgs::msg::Twist2D(), true});
  generator.enrich_planning_snapshot(retry_snapshot);
  ASSERT_TRUE(retry_snapshot.valid);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.current_state.velocity.x, initial_command.x);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.current_state.velocity.theta, initial_command.theta);
  EXPECT_DOUBLE_EQ(retry_snapshot.activation_state.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(retry_snapshot.activation_state.velocity.theta, 0.0);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.activation_state.linear_acceleration,
    -initial_command.x / 0.03);
  EXPECT_DOUBLE_EQ(
    retry_snapshot.activation_state.angular_acceleration,
    -initial_command.theta / 0.03);
  EXPECT_EQ(
    retry_snapshot.activation_state.linear_fir_history,
    initial_state->linear_fir_history);
  EXPECT_EQ(
    retry_snapshot.activation_state.angular_fir_history,
    initial_state->angular_fir_history);

  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(retry_snapshot));
  generator.startNewIteration(odom_velocity);
  nav_2d_msgs::msg::Twist2D retry_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  retry_state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      retry_command = candidate;
      retry_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(retry_state.has_value());
  generator.select_command_for_dispatch(retry_state);
  generator.commit_selected_command(retry_command, node->now());

  dispatch.header.stamp = node->now();
  dispatch.command = geometry_msgs::msg::Twist();
  generator.observe_command_dispatch(dispatch);
  dispatch.header.stamp = node->now();
  dispatch.command.linear.x = retry_command.x;
  dispatch.command.angular.z = retry_command.theta;
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot after_retry_dispatch =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(after_retry_dispatch);
  ASSERT_TRUE(after_retry_dispatch.valid);
  EXPECT_TRUE(after_retry_dispatch.current_state.native_state_valid);
  EXPECT_NEAR(
    after_retry_dispatch.current_state.linear_acceleration,
    retry_state->linear_state.acceleration, 1.0e-12);
  EXPECT_NEAR(
    after_retry_dispatch.current_state.angular_acceleration,
    retry_state->angular_state.acceleration, 1.0e-12);
  EXPECT_EQ(
    after_retry_dispatch.current_state.linear_fir_history,
    retry_state->linear_fir_history);
  EXPECT_EQ(
    after_retry_dispatch.current_state.angular_fir_history,
    retry_state->angular_fir_history);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirSequencedExternalStopPrecedesPendingCommandWithoutResettingHistory)
{
  const auto node = make_node("fir_sequenced_external_stop_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot initial = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(initial);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(initial));
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());

  nav_2d_msgs::msg::Twist2D applied_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  applied_state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (candidate.x > 1.0e-12) {
      applied_command = candidate;
      applied_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(applied_state.has_value());
  const rclcpp::Time issued_at(1, 0, RCL_ROS_TIME);
  generator.select_command_for_dispatch(applied_state);
  generator.commit_selected_command(applied_command, issued_at);

  f_dwa_controller::msg::CommandDispatch applied;
  applied.header.stamp = rclcpp::Time(1, 10000000, RCL_ROS_TIME);
  applied.command.linear.x = applied_command.x;
  applied.command.angular.z = applied_command.theta;
  applied.has_sequence = true;
  generator.observe_command_dispatch(applied);

  PlanningSnapshot next = make_observable_zero_snapshot(node->now());
  next.current_state.velocity = applied_command;
  next.activation_state.velocity = applied_command;
  generator.enrich_planning_snapshot(next);
  ASSERT_TRUE(next.valid);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(next));
  generator.startNewIteration(applied_command);
  ASSERT_TRUE(generator.hasMoreTwists());
  const auto unpublished_command = generator.nextTwist();
  const auto unpublished_state = generator.active_candidate_command_state();
  ASSERT_TRUE(unpublished_state.has_value());
  generator.select_command_for_dispatch(unpublished_state);
  generator.commit_selected_command(
    unpublished_command, rclcpp::Time(1, 20000000, RCL_ROS_TIME));

  ASSERT_TRUE(generator.commit_observed_controller_stop_before_pending(
      rclcpp::Time(1, 60000000, RCL_ROS_TIME)));
  f_dwa_controller::msg::CommandDispatch stopped;
  stopped.header.stamp = rclcpp::Time(1, 60000000, RCL_ROS_TIME);
  stopped.has_sequence = true;
  generator.observe_command_dispatch(stopped);

  PlanningSnapshot observed = make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(observed);
  ASSERT_TRUE(observed.valid);
  EXPECT_TRUE(observed.current_state.native_state_valid);
  EXPECT_DOUBLE_EQ(observed.current_state.velocity.x, 0.0);
  EXPECT_NEAR(
    observed.current_state.linear_acceleration,
    -applied_command.x / 0.05, 1.0e-12);
  EXPECT_EQ(
    observed.current_state.linear_fir_history,
    applied_state->linear_fir_history);
  EXPECT_EQ(
    observed.current_state.angular_fir_history,
    applied_state->angular_fir_history);

  f_dwa_controller::msg::CommandDispatch published_after_stop;
  published_after_stop.header.stamp =
    rclcpp::Time(1, 110000000, RCL_ROS_TIME);
  published_after_stop.command.linear.x = unpublished_command.x;
  published_after_stop.command.angular.z = unpublished_command.theta;
  published_after_stop.has_sequence = true;
  generator.observe_command_dispatch(published_after_stop);

  PlanningSnapshot after_pending = make_observable_zero_snapshot(node->now());
  after_pending.current_state.velocity = unpublished_command;
  after_pending.activation_state.velocity = unpublished_command;
  generator.enrich_planning_snapshot(after_pending);
  ASSERT_TRUE(after_pending.valid);
  EXPECT_TRUE(after_pending.current_state.native_state_valid);
  EXPECT_EQ(
    after_pending.current_state.linear_fir_history,
    unpublished_state->linear_fir_history);
  EXPECT_EQ(
    after_pending.current_state.angular_fir_history,
    unpublished_state->angular_fir_history);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirRejectsOutOfOrderZeroMatchingOnlyALaterExpectedStop)
{
  const auto node = make_node("fir_out_of_order_stop_test", true, true);
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

  nav_2d_msgs::msg::Twist2D nonzero_command;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  nonzero_state;
  while (generator.hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      nonzero_command = candidate;
      nonzero_state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(nonzero_state.has_value());

  const rclcpp::Time issued_at = node->now();
  generator.select_command_for_dispatch(nonzero_state);
  generator.commit_selected_command(nonzero_command, issued_at);
  ASSERT_TRUE(generator.commit_expected_controller_stop(issued_at));

  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = node->now();
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot after_out_of_order_dispatch =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(after_out_of_order_dispatch);
  EXPECT_FALSE(after_out_of_order_dispatch.valid);
  EXPECT_FALSE(
    after_out_of_order_dispatch.current_state.native_state_valid);
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

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  AccelerationSafetyReductionUsesActuallyAppliedCommandState)
{
  const auto node = make_node("acceleration_safety_reduction_test", true, true);
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  f_dwa_controller::msg::CommandDispatch reset;
  reset.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  reset.has_sequence = false;
  generator.observe_command_dispatch(reset);

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());

  nav_2d_msgs::msg::Twist2D issued;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState> state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      issued = candidate;
      state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(state.has_value());
  generator.select_command_for_dispatch(state);
  generator.commit_selected_command(
    issued, rclcpp::Time(1, 0, RCL_ROS_TIME));

  f_dwa_controller::msg::CommandDispatch applied;
  applied.header.stamp = rclcpp::Time(1, 50000000, RCL_ROS_TIME);
  applied.command.linear.x = 0.8 * issued.x;
  applied.command.angular.z = 0.8 * issued.theta;
  applied.has_sequence = true;
  generator.observe_command_dispatch(applied, true);

  PlanningSnapshot observed =
    make_observable_zero_snapshot(node->now());
  observed.current_state.velocity.x = applied.command.linear.x;
  observed.current_state.velocity.theta = applied.command.angular.z;
  observed.activation_state.velocity = observed.current_state.velocity;
  generator.enrich_planning_snapshot(observed);
  ASSERT_TRUE(observed.valid);
  EXPECT_TRUE(observed.current_state.native_state_valid);
  EXPECT_NEAR(observed.current_state.velocity.x, 0.8 * issued.x, 1.0e-12);
  EXPECT_NEAR(
    observed.current_state.velocity.theta,
    0.8 * issued.theta, 1.0e-12);
  EXPECT_NEAR(
    observed.current_state.linear_acceleration,
    0.8 * issued.x / 0.05, 1.0e-10);
  EXPECT_NEAR(
    observed.current_state.angular_acceleration,
    0.8 * issued.theta / 0.05, 1.0e-10);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  FirSafetyReductionPreservesRawHistoryAndUsesAppliedVelocity)
{
  const auto node = make_node("fir_safety_reduction_test", true, true);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  f_dwa_controller::msg::CommandDispatch reset;
  reset.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  reset.has_sequence = false;
  generator.observe_command_dispatch(reset);

  PlanningSnapshot snapshot =
    make_observable_zero_snapshot(node->now());
  generator.enrich_planning_snapshot(snapshot);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(nav_2d_msgs::msg::Twist2D());

  nav_2d_msgs::msg::Twist2D issued;
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState> state;
  while (generator.hasMoreTwists()) {
    const auto candidate = generator.nextTwist();
    if (std::abs(candidate.x) > 1.0e-12 ||
      std::abs(candidate.theta) > 1.0e-12)
    {
      issued = candidate;
      state = generator.active_candidate_command_state();
      break;
    }
  }
  ASSERT_TRUE(state.has_value());
  generator.select_command_for_dispatch(state);
  generator.commit_selected_command(
    issued, rclcpp::Time(1, 0, RCL_ROS_TIME));

  f_dwa_controller::msg::CommandDispatch applied;
  applied.header.stamp = rclcpp::Time(1, 50000000, RCL_ROS_TIME);
  applied.command.linear.x = 0.8 * issued.x;
  applied.command.angular.z = 0.8 * issued.theta;
  applied.has_sequence = true;
  generator.observe_command_dispatch(applied, true);

  PlanningSnapshot observed =
    make_observable_zero_snapshot(node->now());
  observed.current_state.velocity.x = applied.command.linear.x;
  observed.current_state.velocity.theta = applied.command.angular.z;
  observed.activation_state.velocity = observed.current_state.velocity;
  generator.enrich_planning_snapshot(observed);
  ASSERT_TRUE(observed.valid);
  EXPECT_TRUE(observed.current_state.native_state_valid);
  EXPECT_NEAR(observed.current_state.velocity.x, 0.8 * issued.x, 1.0e-12);
  EXPECT_NEAR(
    observed.current_state.velocity.theta,
    0.8 * issued.theta, 1.0e-12);
  EXPECT_EQ(
    observed.current_state.linear_fir_history,
    state->linear_fir_history);
  EXPECT_EQ(
    observed.current_state.angular_fir_history,
    state->angular_fir_history);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RecoveryRankingIsLexicographicAndRejectsInvalidMetrics)
{
  EXPECT_FALSE(
    ScorePlannerAdapter::fuse_clearance_risks(
      std::nullopt, std::nullopt).has_value());
  EXPECT_DOUBLE_EQ(
    *ScorePlannerAdapter::fuse_clearance_risks(0.2, std::nullopt), 0.2);
  EXPECT_DOUBLE_EQ(
    *ScorePlannerAdapter::fuse_clearance_risks(std::nullopt, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(
    *ScorePlannerAdapter::fuse_clearance_risks(0.2, 0.6), 0.6);

  const std::size_t no_candidate =
    std::numeric_limits<std::size_t>::max();
  EXPECT_TRUE(ScorePlannerAdapter::recovery_prefers_candidate(
      0.5, 0.0, 0.9, 0.0, 100.0, 0.0, 7u, no_candidate));

  // Collision-free horizon dominates every finite collision time even when
  // the secondary risks are worse.
  EXPECT_TRUE(ScorePlannerAdapter::recovery_prefers_candidate(
      std::numeric_limits<double>::infinity(), 1.2,
      0.9, 0.1, 100.0, 1.0, 8u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_prefers_candidate(
      1.1, 1.2, 0.0, 1.0, 0.0, 100.0, 6u, 7u));

  // Equal collision time falls through to fused clearance, then weighted
  // path departure, and finally the deterministic canonical index.
  EXPECT_TRUE(ScorePlannerAdapter::recovery_prefers_candidate(
      1.2, 1.2, 0.2, 0.3, 100.0, 1.0, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_prefers_candidate(
      1.2, 1.2, 0.3, 0.3, 0.5, 1.0, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_prefers_candidate(
      1.2, 1.2, 0.3, 0.3, 1.0, 1.0, 6u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_prefers_candidate(
      std::numeric_limits<double>::quiet_NaN(), 1.2,
      0.1, 0.2, 1.0, 2.0, 6u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_prefers_candidate(
      1.2, 1.2, -0.1, 0.2, 1.0, 2.0, 6u, 7u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RecoveryReserveAllowsOnlyRoundoffAtHighRisk)
{
  constexpr double minimum_horizon = 0.17;
  constexpr double maximum_approach_risk = 1.0e-9;
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u, 0.0,
      maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u, 1.0e-8,
      maximum_approach_risk, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u, 0.004,
      0.005, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u, 0.006,
      0.005, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      minimum_horizon, 0u, 1.0, maximum_approach_risk, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      minimum_horizon, 1u, maximum_approach_risk,
      maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      0.0, 0u, 0.0, maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      0.45, 1u, 0.1, maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::quiet_NaN(), 0u, 0.0,
      maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      0.5, 0u, std::numeric_limits<double>::quiet_NaN(),
      maximum_approach_risk, minimum_horizon));
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      0.5, 0u, 0.0, maximum_approach_risk, -0.1));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  LeastViolationRecoveryRanksMethodNativeCandidatesDeterministically)
{
  const std::size_t no_candidate =
    std::numeric_limits<std::size_t>::max();
  EXPECT_TRUE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.05, 0.0, 100.0, 0.0, 7u, no_candidate));

  // Avoidance time is the primary objective after the common obstacle hard
  // gate has rejected every candidate.
  EXPECT_TRUE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.10, 0.05, 100.0, 1.0, 8u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.05, 0.10, 0.0, 100.0, 6u, 7u));

  // Equal collision time falls through to the ordinary residual weighted
  // objective, then to stable canonical order.
  EXPECT_TRUE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.10, 0.10, 1.0, 2.0, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.10, 0.10, 2.0, 2.0, 6u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::least_violation_prefers_candidate(
      std::numeric_limits<double>::quiet_NaN(), 0.10,
      1.0, 2.0, 6u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.10, 0.10, -1.0, 2.0, 6u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::least_violation_prefers_candidate(
      0.10, 0.10, std::numeric_limits<double>::infinity(),
      2.0, 6u, 7u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ProgressReserveRequiresRecoveryBeforeFurtherApproach)
{
  constexpr double minimum_horizon = 0.17;
  const double no_recovery_limit =
    ScorePlannerAdapter::reserve_approach_limit(false);
  const double recovery_limit =
    ScorePlannerAdapter::reserve_approach_limit(true);
  EXPECT_DOUBLE_EQ(no_recovery_limit, 1.0e-9);
  EXPECT_DOUBLE_EQ(recovery_limit, 1.0);
  EXPECT_FALSE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u,
      0.95, no_recovery_limit, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u,
      0.95, recovery_limit, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 50u,
      0.0, no_recovery_limit, minimum_horizon));
  EXPECT_TRUE(ScorePlannerAdapter::recovery_preserves_uncertainty_reserve(
      std::numeric_limits<double>::infinity(), 0u,
      0.95, no_recovery_limit, minimum_horizon));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ReserveConsumptionRequiresBothEntryAndFurtherApproach)
{
  EXPECT_TRUE(ScorePlannerAdapter::consumes_reserve(
      0.25, 0.04, 0.10, 1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::consumes_reserve(
      0.25, 0.12, 0.10, 1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::consumes_reserve(
      0.04, 0.04, 0.10, 1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::consumes_reserve(
      0.04, 0.05, 0.10, 1.0e-6));
  EXPECT_FALSE(ScorePlannerAdapter::consumes_reserve(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.10, 1.0e-6));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  ProgressEscapeRanksPathProgressWithinOneLocalizationRiskClass)
{
  ScorePlannerAdapter::ProgressRank best;
  best.clearance_guard_bucket = 0u;
  best.has_translation_progress = true;
  best.progress_cost = 1.0;
  best.path_deviation_cost = 0.1;
  best.mean_path_distance_bucket = 0u;
  best.mean_path_distance_cost = 0.1;
  best.recovers_initial_clearance = true;
  best.approach_risk = 0.0;
  best.clearance_risk_bucket = 0u;
  best.avoidance_horizon_seconds =
    std::numeric_limits<double>::infinity();
  best.canonical_index = 10u;

  auto candidate = best;
  candidate.progress_cost = 0.5;
  candidate.path_deviation_cost = 10.0;
  candidate.recovers_initial_clearance = false;
  candidate.approach_risk = 0.9;
  candidate.canonical_index = 11u;
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));

  candidate = best;
  candidate.consumes_uncertainty_reserve = true;
  candidate.progress_cost = 2.0;
  candidate.canonical_index = 11u;
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));

  candidate = best;
  best.consumes_uncertainty_reserve = true;
  candidate.progress_cost = 2.0;
  candidate.canonical_index = 11u;
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));
  candidate.progress_cost = best.progress_cost;
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));
  best.consumes_uncertainty_reserve = false;

  candidate = best;
  candidate.clearance_guard_bucket = 1u;
  candidate.progress_cost = 2.0;
  candidate.canonical_index = 11u;
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));
  candidate.progress_cost = 0.0;
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));

  candidate = best;
  candidate.has_translation_progress = true;
  candidate.clearance_guard_bucket = 100u;
  candidate.progress_cost = 100.0;
  candidate.canonical_index = 11u;
  best.has_translation_progress = false;
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_prefers(candidate, best));

  ScorePlannerAdapter::ProgressRank absent;
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_prefers(candidate, absent));
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_prefers(absent, candidate));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  StopCertifiedProgressReplacesOnlyARecedingHorizonStall)
{
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_replaces_weighted_winner(
      false, true, false));
  EXPECT_TRUE(ScorePlannerAdapter::progress_escape_replaces_weighted_winner(
      true, true, false));
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_replaces_weighted_winner(
      true, false, false));
  EXPECT_FALSE(ScorePlannerAdapter::progress_escape_replaces_weighted_winner(
      true, true, true));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  LegalAvoidanceEscapePrefersSafetyThenExecutableProgress)
{
  const std::size_t no_candidate =
    std::numeric_limits<std::size_t>::max();
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      2u, 0u, 3u, 0u, 0.4, 0.0, false, false,
      0.2, 0.0, 0.1, 0.0,
      7u, no_candidate));
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 1u, 9u, 0u, 0.9, 0.1, false, true,
      0.1, 0.2, 0.2, 0.1, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 2u, 0.9, 0.1, false, true,
      0.1, 0.2, 0.2, 0.1, 8u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, 0.9, 0.1, true, false,
      0.1, 0.2, 0.2, 0.1, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, 0.1, 0.9, false, true,
      0.1, 0.2, 0.1, 0.2, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, 0.1, 0.1, true, true,
      0.3, 0.2, 0.2, 0.1, 8u, 7u));
  EXPECT_TRUE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, 0.1, 0.1, true, true,
      0.2, 0.3, 0.2, 0.1, 8u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, 0.1, 0.1, true, true,
      0.3, 0.2, 0.1, 0.2, 8u, 7u));
  EXPECT_FALSE(ScorePlannerAdapter::legal_escape_prefers_candidate(
      0u, 0u, 1u, 1u, -0.1, 0.1, true, true,
      0.2, 0.2, 0.1, 0.2, 8u, 7u));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RecoveryCollisionTimePreservesSweptInterpolationOrder)
{
  std::vector<geometry_msgs::msg::Pose2D> poses(2u);
  poses[1u].x = 0.25;
  std::vector<geometry_msgs::msg::Point> footprint(4u);
  footprint[0u].x = 0.1;
  footprint[0u].y = 0.1;
  footprint[1u].x = 0.1;
  footprint[1u].y = -0.1;
  footprint[2u].x = -0.1;
  footprint[2u].y = -0.1;
  footprint[3u].x = -0.1;
  footprint[3u].y = 0.1;

  CertificationResult result;
  result.failure = CertificationFailure::kLethalObstacle;
  result.has_failure_pose = true;
  result.failure_source_pose_index = 1u;
  result.failure_interpolation_index = 2u;
  EXPECT_NEAR(
    ScorePlannerAdapter::collision_time(
      result, poses, footprint, 0.1, 0.05),
    2.0 * 0.05 / 3.0, 1.0e-12);

  result.failure_source_pose_index = 0u;
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::collision_time(
      result, poses, footprint, 0.1, 0.05),
    0.0);
  result.safe = true;
  EXPECT_TRUE(std::isinf(
      ScorePlannerAdapter::collision_time(
      result, poses, footprint, 0.1, 0.05)));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RecoveryReusesObstacleRejectionCollisionTimeWithoutASecondSweep)
{
  std::vector<geometry_msgs::msg::Pose2D> poses(2u);
  poses[1u].x = 0.25;
  std::vector<geometry_msgs::msg::Point> footprint(4u);
  footprint[0u].x = 0.1;
  footprint[0u].y = 0.1;
  footprint[1u].x = 0.1;
  footprint[1u].y = -0.1;
  footprint[2u].x = -0.1;
  footprint[2u].y = -0.1;
  footprint[3u].x = -0.1;
  footprint[3u].y = 0.1;

  EXPECT_NEAR(
    ScorePlannerAdapter::rejection_collision_time(
      "Trajectory Hits Obstacle.;pose_index=1;subdivision=2;pose_x=0.2",
      poses, footprint, 0.1, 0.05),
    2.0 * 0.05 / 3.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::rejection_collision_time(
      "Trajectory Hits Obstacle.;pose_index=1;subdivision=0;pose_x=0.25",
      poses, footprint, 0.1, 0.05),
    0.05);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::rejection_collision_time(
      "Trajectory Hits Obstacle.;pose_index=0;subdivision=0",
      poses, footprint, 0.1, 0.05),
    0.0);
  EXPECT_DOUBLE_EQ(
    ScorePlannerAdapter::rejection_collision_time(
      "Trajectory Hits Obstacle.;pose_index=0;subdivision=0;"
      "physical_core_pose_index=1;physical_core_subdivision=0",
      poses, footprint, 0.1, 0.05),
    0.05);
  EXPECT_NEAR(
    ScorePlannerAdapter::rejection_collision_time(
      "Trajectory Hits Obstacle.;pose_index=0;subdivision=0;"
      "physical_core_pose_index=1;physical_core_subdivision=2",
      poses, footprint, 0.1, 0.05),
    2.0 * 0.05 / 3.0, 1.0e-12);
  EXPECT_TRUE(std::isnan(
      ScorePlannerAdapter::rejection_collision_time(
        "Trajectory Hits Obstacle.;pose_index=1",
        poses, footprint, 0.1, 0.05)));
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  JerkRecoveryMaterializationPreservesFirstStepAndDispatchState)
{
  constexpr double kControlPeriod = 0.05;
  constexpr double kMaximumJerk = 1.57;
  constexpr std::size_t kCandidateIndex = 73u;
  const auto node = make_node(
    "jerk_recovery_materialization_test", true, false, false, 0.0,
    0.6, 0.6, kControlPeriod);
  JerkTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  generator.reset_trial_state();

  PlanningSnapshot snapshot = make_observable_zero_snapshot(node->now());
  snapshot.current_state.velocity.x = 0.35;
  snapshot.current_state.velocity.theta = -0.12;
  snapshot.current_state.linear_acceleration = 0.30;
  snapshot.current_state.angular_acceleration = -0.40;
  snapshot.activation_state = snapshot.current_state;
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.current_state.velocity);
  ASSERT_EQ(generator.candidate_count(), 165u);

  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = 0.4;
  start_pose.y = -0.2;
  start_pose.theta = 0.3;
  dwb_msgs::msg::Trajectory2D materialized;
  NativeInputTrajectoryGenerator::NativeCommandState materialized_state;
  ASSERT_TRUE(generator.materialize_candidate(
      kCandidateIndex, start_pose, materialized, materialized_state));
  ASSERT_TRUE(materialized_state.valid);
  ASSERT_GE(materialized.poses.size(), 2u);
  EXPECT_LE(
    std::abs(
      (materialized_state.linear_state.acceleration -
      snapshot.activation_state.linear_acceleration) / kControlPeriod),
    kMaximumJerk + 1.0e-12);
  EXPECT_LE(
    std::abs(
      (materialized_state.angular_state.acceleration -
      snapshot.activation_state.angular_acceleration) / kControlPeriod),
    kMaximumJerk + 1.0e-12);

  bool active_candidate_matched = false;
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    const auto index = generator.active_candidate_canonical_index();
    ASSERT_TRUE(index.has_value());
    if (*index != kCandidateIndex) {
      continue;
    }
    const auto active_state = generator.active_candidate_command_state();
    ASSERT_TRUE(active_state.has_value());
    EXPECT_DOUBLE_EQ(command.x, materialized.velocity.x);
    EXPECT_DOUBLE_EQ(command.theta, materialized.velocity.theta);
    EXPECT_DOUBLE_EQ(
      active_state->linear_state.velocity,
      materialized_state.linear_state.velocity);
    EXPECT_DOUBLE_EQ(
      active_state->linear_state.acceleration,
      materialized_state.linear_state.acceleration);
    EXPECT_DOUBLE_EQ(
      active_state->angular_state.velocity,
      materialized_state.angular_state.velocity);
    EXPECT_DOUBLE_EQ(
      active_state->angular_state.acceleration,
      materialized_state.angular_state.acceleration);
    const auto active_trajectory = generator.generateTrajectory(
      start_pose, snapshot.current_state.velocity, command);
    EXPECT_EQ(active_trajectory, materialized);
    active_candidate_matched = true;
    break;
  }
  ASSERT_TRUE(active_candidate_matched);

  generator.select_command_for_dispatch(materialized_state);
  const rclcpp::Time issued_at(10, 0, RCL_ROS_TIME);
  generator.commit_selected_command(
    materialized_state.command_velocity, issued_at);
  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = issued_at;
  dispatch.command.linear.x = materialized_state.command_velocity.x;
  dispatch.command.angular.z = materialized_state.command_velocity.theta;
  dispatch.has_sequence = true;
  generator.observe_command_dispatch(dispatch);

  PlanningSnapshot observed = make_observable_zero_snapshot(node->now());
  observed.current_state.velocity = materialized_state.command_velocity;
  observed.activation_state.velocity = materialized_state.command_velocity;
  generator.enrich_planning_snapshot(observed);
  ASSERT_TRUE(observed.valid);
  ASSERT_TRUE(observed.current_state.native_state_valid);
  EXPECT_NEAR(
    observed.current_state.linear_acceleration,
    materialized_state.linear_state.acceleration, 1.0e-12);
  EXPECT_NEAR(
    observed.current_state.angular_acceleration,
    materialized_state.angular_state.acceleration, 1.0e-12);

  dwb_msgs::msg::Trajectory2D invalid_trajectory;
  NativeInputTrajectoryGenerator::NativeCommandState invalid_state;
  EXPECT_FALSE(generator.materialize_candidate(
      generator.candidate_count(), start_pose,
      invalid_trajectory, invalid_state));
  EXPECT_TRUE(invalid_trajectory.poses.empty());
  EXPECT_FALSE(invalid_state.valid);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  RecedingHorizonRecoveryHasDistinctRealtimeMarkerStatus)
{
  ScorePlannerAdapter planner;
  dwb_msgs::msg::LocalPlanEvaluation evaluation;
  evaluation.best_index = 0;
  dwb_msgs::msg::TrajectoryScore recovery;
  recovery.total = 0.0;
  recovery.traj.poses.resize(2u);
  recovery.traj.poses.back().x = 0.02;
  dwb_msgs::msg::CriticScore detail;
  detail.name = "RecedingHorizonRecovery";
  detail.raw_score = 1.0;
  recovery.scores.push_back(detail);
  evaluation.twists.push_back(recovery);

  const auto markers = planner.candidate_markers(evaluation);
  const auto status = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [](const auto & marker) {
      return marker.ns == "dwb_candidate_status_realtime";
    });
  ASSERT_NE(status, markers.markers.end());
  EXPECT_NE(status->text.find("verified one-step recovery"),
    std::string::npos);
  EXPECT_GT(status->color.r, status->color.g);
  EXPECT_GT(status->color.g, status->color.b);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  LeastViolationRecoveryIsNotReportedAsCollisionFree)
{
  ScorePlannerAdapter planner;
  dwb_msgs::msg::LocalPlanEvaluation evaluation;
  evaluation.best_index = 0;
  dwb_msgs::msg::TrajectoryScore recovery;
  recovery.total = 0.0;
  recovery.traj.poses.resize(2u);
  recovery.traj.poses.back().x = 0.02;
  dwb_msgs::msg::CriticScore detail;
  detail.name = "MethodNativeLeastViolationRecovery";
  detail.raw_score = 1.0;
  recovery.scores.push_back(detail);
  evaluation.twists.push_back(recovery);

  const auto markers = planner.candidate_markers(evaluation);
  const auto status = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [](const auto & marker) {
      return marker.ns == "dwb_candidate_status_realtime";
    });
  ASSERT_NE(status, markers.markers.end());
  EXPECT_NE(status->text.find("No collision-free trajectory"),
    std::string::npos);
  EXPECT_NE(status->text.find("maximum effort"), std::string::npos);
  EXPECT_GT(status->color.r, status->color.g);
  EXPECT_GT(status->color.g, status->color.b);
}

}  // namespace f_dwa_controller

namespace f_dwa_controller
{
TEST_F(NativeInputTrajectoryGeneratorTest, VelocityGoalStopRequiresSafeObservedAndPredictedCapture)
{
  struct Case
  {
    double x; double v; double w; bool blocked; bool expected_stop;
    double command_v{std::numeric_limits<double>::quiet_NaN()};
    double command_w{std::numeric_limits<double>::quiet_NaN()};
  };
  for (const auto & example : std::vector<Case>{
      {0.12, 0.074, 0.4, false, true}, {0.35, 0.074, 0.4, false, false},
      {0.27, 0.5, 0.0, false, false}, {0.12, 0.074, 0.4, true, false},
      {0.12, 0.0, 0.0, false, true},
      {0.12, 0.04, 0.03, false, true, 0.20, 0.30},
      {0.12, 0.30, 0.40, false, true, 0.08, -0.12}})
  {
    SCOPED_TRACE(example.x);
    SCOPED_TRACE(example.blocked);
    const auto node = make_node("goal_velocity_stop_regression");
    node->declare_parameter("publish_zero_velocity", true);
    node->declare_parameter("FollowPath.trajectory_generator_name",
      "dwb_plugins::StandardTrajectoryGenerator");
    node->declare_parameter("FollowPath.critics", std::vector<std::string>{"GoalDist"});
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_distance", 0.3);
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_yaw_tolerance", M_PI);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides({
        {"global_frame", "odom"}, {"plugins", std::vector<std::string>{}},
        {"filters", std::vector<std::string>{}}, {"track_unknown_space", false}});
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap->getLayeredCostmap()->resizeMap(200u, 200u, 0.05, -5.0, -5.0);
    if (example.blocked) {
      unsigned int mx = 0u, my = 0u;
      ASSERT_TRUE(costmap->getCostmap()->worldToMap(example.x, 0., mx, my));
      costmap->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "goal_velocity_stop_regression", true));
    ScorePlannerAdapter planner;
    planner.configure(node, kPluginName, tf, costmap);
    auto generator = std::make_shared<VLimitedAccelTrajectoryGenerator>();
    generator->initialize(node, kPluginName);
    planner.set_test_components(generator, {});
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.orientation.w = 1.0;
    path.poses.push_back(goal);
    planner.setPlan(path);
    nav_2d_msgs::msg::Twist2D velocity;
    velocity.x = example.v;
    velocity.theta = example.w;
    auto command = velocity;
    if (std::isfinite(example.command_v)) {
      command.x = example.command_v;
      command.theta = example.command_w;
      auto snapshot = make_observable_zero_snapshot(node->now());
      snapshot.activation_state.velocity = velocity;
      snapshot.activation_state.native_command_velocity = command;
      snapshot.activation_state.native_command_velocity_valid = true;
      generator->set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
    }
    geometry_msgs::msg::Pose2D pose;
    pose.x = example.x;
    auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    dwb_msgs::msg::TrajectoryScore selected;
    try {
      selected = planner.run_terminal_core(pose, velocity, results);
    } catch (const dwb_core::NoLegalTrajectoriesException &) {
      EXPECT_FALSE(example.expected_stop);
      planner.cleanup();
      costmap->on_cleanup(rclcpp_lifecycle::State());
      continue;
    }
    const bool selected_goal_stop = std::any_of(selected.scores.begin(), selected.scores.end(),
        [](const auto & score) {return score.name == "TerminalGoalVelocityStop";});
    EXPECT_EQ(selected_goal_stop, example.expected_stop);
    if (example.expected_stop) {
      EXPECT_LE(std::abs(selected.traj.velocity.x), std::abs(command.x));
      EXPECT_LE(std::abs(selected.traj.velocity.theta), std::abs(command.theta));
      EXPECT_LE(std::abs(selected.traj.velocity.x - command.x), 1.2 * 0.03 + 1e-9);
      EXPECT_LE(std::abs(selected.traj.velocity.theta - command.theta), 1.57 * 0.03 + 1e-9);
      if (command.x > 0.) {
        EXPECT_LT(selected.traj.velocity.x, command.x);
      }
      ASSERT_FALSE(selected.traj.poses.empty());
      const auto & terminal = selected.traj.poses.back();
      EXPECT_LE(std::hypot(terminal.x, terminal.y), 0.3);
    }
    planner.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }
}
}  // namespace f_dwa_controller


namespace f_dwa_controller
{
TEST_F(NativeInputTrajectoryGeneratorTest, MovingGoalStopRequiresObservedAndPredictedCapture)
{
  struct Case {double x; double v; double w; bool expected_stop;};
  for (const auto & example : std::vector<Case>{
      {0.12, 0.074, 0.632, true}, {0.35, 0.074, 0.632, false},
      {0.27, 0.5, 0.0, false}})
  {
    SCOPED_TRACE(example.x);
    const auto node = make_node("goal_native_stop_regression");
    node->declare_parameter("publish_zero_velocity", true);
    node->declare_parameter("FollowPath.trajectory_generator_name",
      "dwb_plugins::StandardTrajectoryGenerator");
    node->declare_parameter("FollowPath.critics", std::vector<std::string>{"GoalDist"});
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_distance", 0.3);
    node->declare_parameter("FollowPath.terminal_stop_goal_capture_yaw_tolerance", M_PI);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides({
        {"global_frame", "odom"}, {"plugins", std::vector<std::string>{}},
        {"filters", std::vector<std::string>{}}, {"track_unknown_space", false}});
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap->getLayeredCostmap()->resizeMap(200u, 200u, 0.05, -5.0, -5.0);
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "goal_stop_regression", true));
    ScorePlannerAdapter planner;
    planner.configure(node, kPluginName, tf, costmap);
    auto generator = std::make_shared<AccelerationTrajectoryGenerator>();
    generator->initialize(node, kPluginName);
    planner.set_test_components(generator, {});
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.orientation.w = 1.0;
    path.poses.push_back(goal);
    planner.setPlan(path);
    auto snapshot = make_observable_zero_snapshot(node->now());
    snapshot.current_state.velocity.x = example.v;
    snapshot.current_state.velocity.theta = example.w;
    snapshot.activation_state = snapshot.current_state;
    generator->set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
    geometry_msgs::msg::Pose2D pose;
    pose.x = example.x;
    auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    dwb_msgs::msg::TrajectoryScore selected;
    try {
      selected = planner.run_terminal_core(pose, snapshot.current_state.velocity, results);
    } catch (const dwb_core::NoLegalTrajectoriesException &) {
      EXPECT_FALSE(example.expected_stop);
      planner.cleanup();
      costmap->on_cleanup(rclcpp_lifecycle::State());
      continue;
    }
    const bool selected_goal_stop = std::any_of(selected.scores.begin(), selected.scores.end(),
        [](const auto & score) {return score.name == "TerminalGoalNativeDirectStop";});
    EXPECT_EQ(selected_goal_stop, example.expected_stop);
    if (example.expected_stop) {
      EXPECT_LT(std::abs(selected.traj.velocity.x), example.v);
      EXPECT_LT(std::abs(selected.traj.velocity.theta), example.w);
      ASSERT_FALSE(selected.traj.poses.empty());
      const auto & terminal = selected.traj.poses.back();
      EXPECT_LE(std::hypot(terminal.x, terminal.y), 0.3);
    }
    planner.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }
}
}  // namespace f_dwa_controller

// Included after the normal native-generator tests in the private F7 build.
#include "f_dwa_controller/recovery_input_dynamics.hpp"
#include "f_dwa_controller/saturation_input_dynamics.hpp"

namespace f_dwa_controller
{

const std::vector<double> & saturation_test_coefficients()
{
  static const std::vector<double> coefficients{0.03350785471347518, 0.03880895409775884,
    0.050673824029058304, 0.06384786914847601, 0.07690688669506258, 0.08879254829473235,
    0.09838470216854145, 0.10476117964444213, 0.10718835907294214, 0.10526061702166864,
    0.0988754486043874, 0.08836452826447266, 0.0743921284996216, 0.057916281901727665,
    0.040067975848862604, 0.022120653081306618, 0.005308757788266448, -0.00927664974561404,
    -0.020804194540549305, -0.02872030625781599, -0.03282552684232682, -0.03325297309064127,
    -0.030450040432456568, -0.025097748543612516, -0.018060132851524392, -0.010256142160916025,
    -0.0025694893836071156, 0.004238749681463807, 0.009567441213462018, 0.013048185465411855,
    0.014559908920583887, 0.014219717639262417, 0.01232848704832304, 0.00933407371742618,
    0.005754862291016337, 0.0021132149070524696, -0.0011325292557404195, -0.00362954735748322,
    -0.005180194851508673, -0.005747253546242862, -0.005430583942718357, -0.004427296616422495,
    -0.003006002105302255, -0.001459087670380678, -4.8955061136223235e-05, 0.0010314444971943957};
  return coefficients;
}

TEST(InputRecovery, JerkUsesNewAccelerationAndCompletesWithinHorizon)
{
  const AxisLimits limits{0., 1., -1., 1., -.5, .5};
  const auto result = minimum_jerk_recovery({0., 0.}, limits, .5, .05, 50);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.states.front().acceleration, .025, 1e-12);
  EXPECT_EQ(result.recovery_steps, 49);
  EXPECT_NEAR(result.recovery_input, -.025 / (49 * .05), 1e-12);
  EXPECT_NEAR(result.states.back().velocity, .03125, 1e-12);
  EXPECT_NEAR(result.states.back().acceleration, 0., 1e-12);
  AxisState previous{0., 0.};
  for (const auto & state : result.states) {
    EXPECT_TRUE(recovery_state_valid(state, limits));
    EXPECT_LE(std::abs((state.acceleration - previous.acceleration) / .05), .5 + 1e-12);
    EXPECT_NEAR(state.velocity - previous.velocity, state.acceleration * .05, 1e-12);
    previous = state;
  }
}

TEST(InputRecovery, JerkRejectsClippedFirstInputAndImpossibleConstantRecovery)
{
  const AxisLimits limits{0., 1., -1., 1., -1., 1.};
  EXPECT_FALSE(minimum_jerk_recovery({0., 0.}, limits, 2., .05, 50).valid);
  // q=0 first yields v=.9985,a=.075; no constant integer recovery is feasible.
  EXPECT_FALSE(minimum_jerk_recovery({.99475, .075}, limits, 0., .05, 50).valid);
  EXPECT_FALSE(minimum_jerk_recovery({0., 0.}, limits, .5, .05, 1).valid);
}

TEST(InputRecovery, JerkHandlesNegativeAccelerationAndZeroWithoutDivision)
{
  const AxisLimits limits{-1., 1., -1., 1., -.5, .5};
  const auto positive = minimum_jerk_recovery({.2, .1}, limits, .3, .05, 50);
  const auto negative = minimum_jerk_recovery({-.2, -.1}, limits, -.3, .05, 50);
  ASSERT_TRUE(positive.valid); ASSERT_TRUE(negative.valid);
  EXPECT_NEAR(positive.recovery_input, -negative.recovery_input, 1e-12);
  EXPECT_TRUE(minimum_jerk_recovery({1., 0.}, limits, 0., .05, 50).valid);
}

TEST(InputRecovery, FirMinimumMatchesDirectNativeConvolutionAndPreservesInput)
{
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  const std::vector<double> coefficients(10, .1), initial_memory(9, 0.);
  const AxisState initial{.95, 0.};
  const auto response = prepare_recovery_fir_response(
    initial, coefficients, initial_memory, .05, 50, 2, 4);
  const auto result = minimum_fir_recovery(response, limits, 1.2);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.recovery_input, 0.);
  EXPECT_GT(result.recovery_input, -1.2);
  auto memory = initial_memory;
  double v = initial.velocity;
  for (std::size_t k = 0; k < response.free_states.size(); ++k) {
    double input = k < 2 ? 1.2 : (k < 6 ? result.recovery_input : 0.);
    const double a = fir_acceleration(coefficients, memory, input);
    v += .05 * a;
    EXPECT_TRUE(recovery_state_valid({v, a}, limits));
    if (k < result.states.size()) {
      EXPECT_NEAR(result.states[k].velocity, v, 1e-12);
      EXPECT_NEAR(result.states[k].acceleration, a, 1e-12);
    }
    push_fir_input(memory, input);
  }
  // Moving the optimum even slightly toward zero must violate a bound.
  bool violation = false;
  for (std::size_t k = 0; k < response.free_states.size(); ++k) {
    const auto & f = response.free_states[k]; const auto & q = response.candidate_states[k];
    const auto & r = response.recovery_states[k];
    violation |= !recovery_state_valid({f.velocity + 1.2 * q.velocity +
          (result.recovery_input + 1e-5) * r.velocity, f.acceleration + 1.2 * q.acceleration +
          (result.recovery_input + 1e-5) * r.acceleration}, limits);
  }
  EXPECT_TRUE(violation);
}

TEST(InputRecovery, FirChecksTailBeyondNominalHorizonAndZeroNegativeGains)
{
  double lower = -1., upper = 1.;
  EXPECT_FALSE(intersect_recovery_bound(2., 0., 0., 1., lower, upper));
  EXPECT_TRUE(intersect_recovery_bound(.5, -1., 0., 1., lower, upper));
  EXPECT_DOUBLE_EQ(lower, -.5); EXPECT_DOUBLE_EQ(upper, .5);
  // The response includes all taps after the last recovery input.
  const auto response = prepare_recovery_fir_response({.97, 0.}, {.1, .1, .8},
      {0., 0.}, .05, 2, 1, 1);
  ASSERT_TRUE(response.valid);
  EXPECT_EQ(response.free_states.size(), 4u);
  const AxisLimits limits{0., 1., -1., 1., 0., 1.};
  EXPECT_FALSE(minimum_fir_recovery(response, limits, 1.).valid);
}

TEST(EqualEffectFirSampling, GreedyAllocationPrioritizesTheLargestPredictedGap)
{
  EXPECT_EQ(
    allocate_equal_effect_refinements({1., 2., 8., 2., 1.}, 5),
    (std::vector<int>{0, 1, 3, 1, 0}));
}

TEST(EqualEffectFirSampling, UsesSixPlusFiveExactFullTailFeasibleEvaluations)
{
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  const std::vector<double> coefficients{.2, .3, .5};
  const std::vector<double> initial_history(coefficients.size() - 1u, 0.);
  const auto context = prepare_equal_effect_fir_context(
    {.5, 0.}, limits, coefficients, initial_history, .05, 50);
  ASSERT_TRUE(context.valid);
  const auto samples = sample_equal_effect_fir_axis(context, 11);
  ASSERT_TRUE(samples.converged) << samples.failure_reason;
  ASSERT_EQ(samples.pulses.size(), 11u);
  EXPECT_EQ(samples.coarse_evaluation_count, 6);
  EXPECT_EQ(samples.refinement_evaluation_count, 5);
  ASSERT_EQ(samples.interval_refinement_counts.size(), 5u);
  EXPECT_EQ(
    std::accumulate(
      samples.interval_refinement_counts.begin(),
      samples.interval_refinement_counts.end(), 0),
    5);

  std::vector<double> expected_inputs;
  const double coarse_step =
    (limits.native_input_max - limits.native_input_min) / 5.;
  for (int interval = 0; interval < 5; ++interval) {
    const double lower = limits.native_input_min + interval * coarse_step;
    const double upper = lower + coarse_step;
    expected_inputs.push_back(lower);
    const int additions =
      samples.interval_refinement_counts[static_cast<std::size_t>(interval)];
    for (int addition = 1; addition <= additions; ++addition) {
      expected_inputs.push_back(
        lower + static_cast<double>(addition) / (additions + 1) * (upper - lower));
    }
  }
  expected_inputs.push_back(limits.native_input_max);

  std::set<double> native_inputs;
  bool found_fractional_duration = false;
  for (std::size_t index = 0u; index < samples.pulses.size(); ++index) {
    const auto & pulse = samples.pulses[index];
    ASSERT_TRUE(pulse.valid);
    ASSERT_EQ(pulse.states.size(), 50u);
    EXPECT_NEAR(pulse.native_input, expected_inputs[index], 1.e-12);
    native_inputs.insert(pulse.native_input);
    found_fractional_duration |=
      pulse.fractional_last_input > 1.e-9 &&
      pulse.fractional_last_input < 1. - 1.e-9;

    auto history = initial_history;
    double velocity = .5;
    double effect = 0.;
    const int full_count = std::max(
      50, pulse.full_input_steps + static_cast<int>(coefficients.size()));
    for (int step = 0; step < full_count; ++step) {
      const double input = step < pulse.full_input_steps ? pulse.native_input :
        step == pulse.full_input_steps ?
        pulse.fractional_last_input * pulse.native_input : 0.;
      const double acceleration = fir_acceleration(coefficients, history, input);
      velocity += .05 * acceleration;
      EXPECT_TRUE(recovery_state_valid({velocity, acceleration}, limits));
      if (step < 50) {effect += .05 * velocity;}
      push_fir_input(history, input);
    }
    EXPECT_NEAR(effect, pulse.horizon_effect, 1.e-12);
  }
  EXPECT_EQ(native_inputs.size(), 11u);
  EXPECT_TRUE(found_fractional_duration);
}

TEST(EqualEffectFirSampling, FractionalOnlyPulseCommitsBetaTimesInput)
{
  const auto context = prepare_equal_effect_fir_context(
    {.999, 0.}, {0., 1., -1., 1., -1., 1.}, {1.}, {}, .05, 10);
  const auto pulse = maximum_feasible_equal_effect_fir_pulse(context, 1.);

  ASSERT_TRUE(pulse.valid);
  EXPECT_EQ(pulse.full_input_steps, 0);
  EXPECT_NEAR(pulse.fractional_last_input, .02, 1.e-10);
  EXPECT_NEAR(pulse.first_native_input, .02, 1.e-10);
  ASSERT_FALSE(pulse.states.empty());
  EXPECT_NEAR(pulse.states.front().velocity, 1., 1.e-10);
}

TEST(EqualEffectFirSampling, SavedFortySixTapDesignConvergesForBothAxes)
{
  const auto & coefficients = saturation_test_coefficients();
  const std::vector<double> zero_history(coefficients.size() - 1u, 0.);
  const auto linear_context = prepare_equal_effect_fir_context(
    {.5, 0.}, {0., 1., -1., 1., -1.2, 1.2},
    coefficients, zero_history, .05, 50);
  const auto angular_context = prepare_equal_effect_fir_context(
    {0., 0.}, {-1., 1., -1., 1., -1.57, 1.57},
    coefficients, zero_history, .05, 50);
  const auto started = std::chrono::steady_clock::now();
  const auto linear = sample_equal_effect_fir_axis(linear_context, 11);
  const auto angular = sample_equal_effect_fir_axis(angular_context, 15);
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - started).count();
  RecordProperty("two_axis_sampling_time_us", elapsed);
  ASSERT_TRUE(linear.converged) << linear.failure_reason;
  ASSERT_TRUE(angular.converged) << angular.failure_reason;
  EXPECT_EQ(linear.pulses.size(), 11u);
  EXPECT_EQ(angular.pulses.size(), 15u);
  EXPECT_EQ(linear.coarse_evaluation_count, 6);
  EXPECT_EQ(linear.refinement_evaluation_count, 5);
  EXPECT_EQ(angular.coarse_evaluation_count, 8);
  EXPECT_EQ(angular.refinement_evaluation_count, 7);
  EXPECT_EQ(
    linear.coarse_evaluation_count + linear.refinement_evaluation_count +
    angular.coarse_evaluation_count + angular.refinement_evaluation_count,
    26);
  EXPECT_DOUBLE_EQ(linear.pulses.front().native_input, -1.2);
  EXPECT_DOUBLE_EQ(linear.pulses.back().native_input, 1.2);
  EXPECT_DOUBLE_EQ(angular.pulses.front().native_input, -1.57);
  EXPECT_DOUBLE_EQ(angular.pulses.back().native_input, 1.57);
  EXPECT_TRUE(std::any_of(
      linear.pulses.begin(), linear.pulses.end(), [](const auto & pulse) {
        return pulse.equivalent_duration > 2.5;
      }));
  EXPECT_TRUE(std::any_of(
      angular.pulses.begin(), angular.pulses.end(), [](const auto & pulse) {
        return pulse.equivalent_duration > 2.5;
      }));
}

TEST(EqualEffectFirSampling, SavedFortySixTapNearLimitStateAlsoConverges)
{
  const auto & coefficients = saturation_test_coefficients();
  const std::vector<double> active_history(coefficients.size() - 1u, .05);
  const auto linear_context = prepare_equal_effect_fir_context(
    {.95, 0.}, {0., 1., -1., 1., -1.2, 1.2},
    coefficients, active_history, .05, 50);
  const auto angular_context = prepare_equal_effect_fir_context(
    {.30, 0.}, {-1., 1., -1., 1., -1.57, 1.57},
    coefficients, active_history, .05, 50);

  const auto linear = sample_equal_effect_fir_axis(linear_context, 11);
  const auto angular = sample_equal_effect_fir_axis(angular_context, 15);

  ASSERT_TRUE(linear.converged) << linear.failure_reason;
  ASSERT_TRUE(angular.converged) << angular.failure_reason;
  EXPECT_EQ(linear.pulses.size(), 11u);
  EXPECT_EQ(angular.pulses.size(), 15u);
  EXPECT_EQ(linear.coarse_evaluation_count + linear.refinement_evaluation_count, 11);
  EXPECT_EQ(angular.coarse_evaluation_count + angular.refinement_evaluation_count, 15);
  EXPECT_DOUBLE_EQ(linear.pulses.front().native_input, -1.2);
  EXPECT_DOUBLE_EQ(linear.pulses.back().native_input, 1.2);
  EXPECT_DOUBLE_EQ(angular.pulses.front().native_input, -1.57);
  EXPECT_DOUBLE_EQ(angular.pulses.back().native_input, 1.57);
}

TEST_F(
  NativeInputTrajectoryGeneratorTest,
  TrialResetHotSwitchesEqualEffectMaxDurationWithoutChangingLegacyDefault)
{
  const auto node = make_node(
    "runtime_equal_effect_sampling", true, false, false, .2,
    1., 1., .05);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  auto snapshot = make_observable_zero_snapshot(node->now());
  snapshot.activation_state.velocity.x = .5;
  snapshot.activation_state.native_command_velocity.x = .5;
  snapshot.activation_state.linear_fir_history.assign(2u, 0.);
  snapshot.activation_state.angular_fir_history.assign(2u, 0.);
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.activation_state.velocity);
  ASSERT_EQ(generator.candidate_count(), 165u);
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_FALSE(
    generator.active_candidate_diagnostics()->uses_equal_effect_max_duration);

  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
      "FollowPath.fir_equal_effect_max_duration_sampling", true)).successful);
  generator.reset_trial_state();
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.activation_state.velocity);
  ASSERT_EQ(generator.candidate_count(), 165u);
  std::set<double> linear_inputs;
  while (generator.hasMoreTwists()) {
    static_cast<void>(generator.nextTwist());
    const auto diagnostics = generator.active_candidate_diagnostics();
    ASSERT_TRUE(diagnostics.has_value());
    EXPECT_TRUE(diagnostics->uses_equal_effect_max_duration);
    linear_inputs.insert(diagnostics->linear_native_input);
  }
  EXPECT_EQ(linear_inputs.size(), 11u);

  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(
      "FollowPath.fir_equal_effect_max_duration_sampling", false)).successful);
  generator.reset_trial_state();
  generator.set_planning_snapshot(
    std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.activation_state.velocity);
  static_cast<void>(generator.nextTwist());
  ASSERT_TRUE(generator.active_candidate_diagnostics().has_value());
  EXPECT_FALSE(
    generator.active_candidate_diagnostics()->uses_equal_effect_max_duration);
  EXPECT_NEAR(
    generator.active_candidate_diagnostics()->linear_prediction_input_duration,
    .2, 1.e-12);
}

TEST_F(NativeInputTrajectoryGeneratorTest, AllSixInputConditionsHaveExactly150Candidates)
{
  for (bool fir : {false, true}) {
    for (int mode = 0; mode < 3; ++mode) {
      auto node = make_node("fixed_150_" + std::to_string(fir) + "_" + std::to_string(mode),
        true, false, false, mode == 2 ? .2 : 0., 1., 1., .05);
      node->declare_parameter("FollowPath.vx_samples", 10);
      node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 10));
      node->declare_parameter("FollowPath.vtheta_samples", 15);
      node->set_parameter(rclcpp::Parameter("FollowPath.vtheta_samples", 15));
      node->declare_parameter("FollowPath.native_input_recovery", mode == 1);
      node->declare_parameter("FollowPath.native_input_pulse_duration",
          !fir && mode == 2 ? .2 : 0.);
      NativeInputTrajectoryGenerator generator(fir ? NativeInputOrder::kFir :
        NativeInputOrder::kJerk);
      generator.initialize(node, kPluginName);
      for (double initial_velocity : {0., .4, .95}) {
        auto snapshot = make_observable_zero_snapshot(node->now());
        snapshot.activation_state.velocity.x = initial_velocity;
        snapshot.activation_state.native_command_velocity.x = initial_velocity;
        snapshot.activation_state.linear_fir_history.assign(2, 0.);
        snapshot.activation_state.angular_fir_history.assign(2, 0.);
        generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
        generator.startNewIteration({});
        ASSERT_EQ(generator.candidate_count(),
            150u) << fir << "/" << mode << "/" << initial_velocity;
        std::set<double> linear, angular;
        while (generator.hasMoreTwists()) {
          generator.nextTwist();
          const auto d = generator.active_candidate_diagnostics();
          ASSERT_TRUE(d.has_value());
          EXPECT_EQ(d->uses_recovery, mode == 1);
          if (mode == 1) {
            EXPECT_GE(d->linear_prediction_input_duration, .05 - 1e-12);
            EXPECT_LE(d->linear_prediction_input_duration, 2.4 + 1e-12);
            if (fir) {EXPECT_DOUBLE_EQ(d->linear_recovery_input, 0.);}
          } else {
            EXPECT_NEAR(d->linear_prediction_input_duration, mode == 0 ? 2.4 : .2, 1e-12);
          }
          linear.insert(d->linear_native_input); angular.insert(d->angular_native_input);
        }
        EXPECT_EQ(linear.size(), 10u); EXPECT_EQ(angular.size(), 15u);
      }
    }
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest,
    FirFixedPulseConstrainsResidualTailWithoutExtendingTrajectory)
{
  const auto & coefficients = saturation_test_coefficients();
  auto node = make_node("fir_fixed_pulse_residual_tail", true, false, false, .2, 1., 1., .05);
  node->declare_parameter("FollowPath.sim_time", .2);
  node->declare_parameter("FollowPath.time_granularity", .05);
  node->declare_parameter("FollowPath.acc_lim_x", .3);
  node->declare_parameter("FollowPath.decel_lim_x", -.3);
  node->declare_parameter("FollowPath.vx_samples", 10);
  node->declare_parameter("FollowPath.vtheta_samples", 15);
  node->declare_parameter("FollowPath.fir_coefficients", coefficients);
  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.sim_time", .2),
      rclcpp::Parameter("FollowPath.time_granularity", .05),
      rclcpp::Parameter("FollowPath.acc_lim_x", .3),
      rclcpp::Parameter("FollowPath.decel_lim_x", -.3),
      rclcpp::Parameter("FollowPath.vx_samples", 10),
      rclcpp::Parameter("FollowPath.vtheta_samples", 15),
      rclcpp::Parameter("FollowPath.fir_coefficients", coefficients)}).successful);
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);

  auto snapshot = make_observable_zero_snapshot(node->now());
  snapshot.activation_state.linear_fir_history.assign(coefficients.size() - 1u, 0.);
  snapshot.activation_state.angular_fir_history.assign(coefficients.size() - 1u, 0.);
  generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration(snapshot.activation_state.velocity);

  ASSERT_EQ(generator.candidate_count(), 150u);
  std::set<double> linear_inputs;
  while (generator.hasMoreTwists()) {
    static_cast<void>(generator.nextTwist());
    const auto diagnostics = generator.active_candidate_diagnostics();
    ASSERT_TRUE(diagnostics.has_value());
    EXPECT_NEAR(diagnostics->linear_prediction_input_duration, .2, 1.0e-12);
    linear_inputs.insert(diagnostics->linear_native_input);
  }
  ASSERT_EQ(linear_inputs.size(), 10u);
  EXPECT_GT(*linear_inputs.rbegin(), .70);
  EXPECT_LT(*linear_inputs.rbegin(), .73);

  const AxisLimits limits{0., 1., -.3, .3, -1.2, 1.2};
  for (const double input : linear_inputs) {
    auto history = std::vector<double>(coefficients.size() - 1u, 0.);
    double velocity = 0.;
    for (int step = 0; step < 4 + static_cast<int>(coefficients.size()) - 1; ++step) {
      const double acceleration = fir_acceleration(
        coefficients, history, step < 4 ? input : 0.);
      velocity += .05 * acceleration;
      EXPECT_TRUE(recovery_state_valid({velocity, acceleration}, limits));
      push_fir_input(history, step < 4 ? input : 0.);
    }
  }
}

TEST(InputRecovery, JerkRecoveryIntervalMatchesIndependentIntegerDurationSearch)
{
  const AxisLimits limits{0., 1., -1., 1., -.5, .5};
  for (const AxisState initial : std::vector<AxisState>{{0., 0.}, {.4, .2}, {.99475, .075},
      {.01, -.1}})
  {
    const auto interval = jerk_recovery_input_interval(initial, limits, .05, 50);
    for (int i = 0; i <= 1000; ++i) {
      const double q = -.5 + i * .001;
      bool feasible = false;
      for (int m = 1; m < 50 && !feasible; ++m) {
        AxisState state{initial.velocity, initial.acceleration};
        state.acceleration += q * .05; state.velocity += state.acceleration * .05;
        if (!recovery_state_valid(state, limits)) {continue;}
        const double r = -state.acceleration / (m * .05);
        if (r < -.5 - 1e-12 || r > .5 + 1e-12) {continue;}
        bool valid = true;
        for (int k = 1; k < 50; ++k) {
          state.acceleration += (k <= m ? r : 0.) * .05;
          state.velocity += state.acceleration * .05;
          valid &= recovery_state_valid(state, limits);
        }
        feasible = valid;
      }
      const bool projected = interval.feasible && q >= interval.lower - 1e-10 &&
        q <= interval.upper + 1e-10;
      EXPECT_EQ(projected, feasible) << initial.velocity << "/" << initial.acceleration << "/" << q;
      if (projected) {EXPECT_TRUE(minimum_jerk_recovery(initial, limits, q, .05, 50).valid);}
    }
  }
}

TEST(InputRecovery, FirProjectedIntervalSamplesRemainFeasibleIncludingEndpoints)
{
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  for (double velocity : {0., .4, .95, 1.}) {
    const auto response = prepare_recovery_fir_response({velocity, 0.},
        {.5, .3, .2}, {0., 0.}, .05, 50, 1, 4);
    const auto interval = fir_recovery_input_interval(response, limits);
    ASSERT_TRUE(interval.feasible);
    for (double q : uniform_samples(interval, 15)) {
      const auto recovered = minimum_fir_recovery(response, limits, q);
      ASSERT_TRUE(recovered.valid) << velocity << "/" << q;
      auto history = std::vector<double>{0., 0.};
      double v = velocity;
      for (int k = 0; k < 50; ++k) {
        const double input = k == 0 ? q : (k < 5 ? recovered.recovery_input : 0.);
        const double a = fir_acceleration({.5, .3, .2}, history, input);
        v += a * .05;
        EXPECT_TRUE(recovery_state_valid({v, a}, limits));
        EXPECT_NEAR(recovered.states[k].velocity, v, 1e-12);
        push_fir_input(history, input);
      }
    }
  }
}

TEST(InputRecovery, RecordedOneHertzFirRecoveryKeepsFullHistoryAndTailWithinBounds)
{
  // 20 Hz / 1 Hz production design, including its negative coefficient lobes.
  const auto & coefficients = saturation_test_coefficients();
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  for (double velocity : {.3, .7}) {
    for (double prior_input : {-.12, 0., .12}) {
      const std::vector<double> memory(coefficients.size() - 1, prior_input);
      const auto response = prepare_recovery_fir_response({velocity, prior_input},
        coefficients, memory, .05, 50, 1, 4);
      const auto interval = fir_recovery_input_interval(response, limits);
      ASSERT_TRUE(interval.feasible);
      for (double q : uniform_samples(interval, 15)) {
        const auto recovered = minimum_fir_recovery(response, limits, q);
        ASSERT_TRUE(recovered.valid) << velocity << "/" << prior_input << "/" << q;
        auto history = memory;
        double v = velocity;
        for (std::size_t k = 0; k < response.free_states.size(); ++k) {
          const double raw = k == 0 ? q : (k < 5 ? recovered.recovery_input : 0.);
          const double a = fir_acceleration(coefficients, history, raw);
          v += a * .05;
          EXPECT_TRUE(recovery_state_valid({v, a}, limits));
          if (k < recovered.states.size()) {
            EXPECT_NEAR(recovered.states[k].velocity, v, 1e-12);
            EXPECT_NEAR(recovered.states[k].acceleration, a, 1e-12);
          }
          push_fir_input(history, raw);
        }
      }
    }
  }
}

TEST(InputRecovery, JerkPulseEndsAtFourCyclesWithoutResettingAcceleration)
{
  const AxisLimits limits{0., 1., -1., 1., -.5, .5};
  const AxisState initial{.4, .05};
  const auto interval = pulsed_jerk_input_interval(initial, limits, .05, 50, 4);
  ASSERT_TRUE(interval.feasible);
  for (double q : uniform_samples(interval, 10)) {
    AxisState state = initial;
    for (int k = 0; k < 50; ++k) {
      state.acceleration += (k < 4 ? q : 0.) * .05;
      state.velocity += state.acceleration * .05;
      EXPECT_TRUE(recovery_state_valid(state, limits));
      if (k >= 3) {EXPECT_NEAR(state.acceleration, initial.acceleration + .2 * q, 1e-12);}
    }
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest, InputRecoveryFirCommitsOnlyExecutedInputToHistory)
{
  auto node = make_node("recovery_fir_memory", true, false, false, .06, 1.0);
  node->declare_parameter("FollowPath.native_input_recovery", true);
  node->declare_parameter("FollowPath.fir_coefficients", std::vector<double>{});
  node->set_parameter(rclcpp::Parameter("FollowPath.fir_coefficients",
      std::vector<double>(10, .1)));
  FirTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  auto snapshot = make_observable_zero_snapshot(node->now());
  snapshot.activation_state.velocity.x = .95;
  snapshot.activation_state.native_command_velocity.x = .95;
  snapshot.activation_state.linear_fir_history.assign(9, 0.);
  snapshot.activation_state.angular_fir_history.assign(9, 0.);
  generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
  generator.startNewIteration({});
  ASSERT_GT(generator.candidate_count(), 0u);
  geometry_msgs::msg::Pose2D pose;
  bool recovered = false;
  for (std::size_t k = 0; k < generator.candidate_count(); ++k) {
    generator.nextTwist();
    dwb_msgs::msg::Trajectory2D trajectory;
    NativeInputTrajectoryGenerator::NativeCommandState state;
    ASSERT_TRUE(generator.materialize_candidate(k, pose, trajectory, state));
    const auto diagnostics = generator.active_candidate_diagnostics();
    ASSERT_TRUE(diagnostics.has_value());
    recovered |= diagnostics->uses_recovery;
    ASSERT_EQ(state.linear_fir_history.size(), 9u);
    EXPECT_DOUBLE_EQ(state.linear_fir_history[0], diagnostics->linear_native_input);
    EXPECT_DOUBLE_EQ(state.linear_fir_history[1], 0.);
  }
  EXPECT_TRUE(recovered);
}

TEST_F(NativeInputTrajectoryGeneratorTest, InputRecoveryStopCertificateKeepsExactFirstCommand)
{
  for (const bool fir : {false, true}) {
    auto node = make_node(fir ? "recovery_fir_stop" : "recovery_j_stop",
      true, false, false, .06, 1.0);
    node->declare_parameter("FollowPath.native_input_recovery", true);
    node->declare_parameter("FollowPath.fir_coefficients", std::vector<double>{});
    node->set_parameter(rclcpp::Parameter("FollowPath.fir_coefficients",
        std::vector<double>(10, .1)));
    NativeInputTrajectoryGenerator generator(fir ? NativeInputOrder::kFir :
      NativeInputOrder::kJerk);
    generator.initialize(node, kPluginName);
    auto snapshot = make_observable_zero_snapshot(node->now());
    snapshot.activation_state.velocity.x = fir ? .95 : .55;
    snapshot.activation_state.linear_acceleration = fir ? 0.0 : .25;
    snapshot.activation_state.linear_fir_history.assign(9, 0.);
    snapshot.activation_state.angular_fir_history.assign(9, 0.);
    generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
    generator.startNewIteration(snapshot.activation_state.velocity);
    std::size_t recovered_certificates = 0;
    while (generator.hasMoreTwists()) {
      const auto command = generator.nextTwist();
      const auto diagnostic = generator.active_candidate_diagnostics();
      ASSERT_TRUE(diagnostic.has_value());
      if (!diagnostic->uses_recovery) {continue;}
      std::vector<geometry_msgs::msg::Pose2D> poses;
      std::vector<nav_2d_msgs::msg::Twist2D> velocities;
      std::vector<NativeInputTrajectoryGenerator::NativeCommandState> states;
      geometry_msgs::msg::Pose2D start;
      if (!generator.generate_stop_trajectory(start, 1000, .01, poses, velocities, &states)) {
        continue;
      }
      ++recovered_certificates;
      ASSERT_FALSE(states.empty());
      EXPECT_NEAR(states.front().command_velocity.x, command.x, 1e-12);
      EXPECT_NEAR(states.front().command_velocity.theta, command.theta, 1e-12);
      EXPECT_NEAR(states.front().linear_state.acceleration,
        diagnostic->first_command_state.linear_state.acceleration, 1e-12);
      if (fir) {
        EXPECT_NEAR(states.front().linear_fir_history.front(), diagnostic->linear_native_input,
            1e-12);
      }
    }
    EXPECT_GT(recovered_certificates, 0u);
  }
}

TEST(SaturationInput, AccelerationHoldsLongestAdmissibleInputThenCoasts)
{
  for (const double maximum : {0.5, 1.0, 1.5}) {
    const AxisLimits limits{-maximum, maximum, -1.5, 1.5, -1.5, 1.5};
    for (const double velocity : {-maximum, -0.3, 0.0, 0.3, maximum}) {
      const AxisState initial{velocity, 0.0};
      const auto interval = acceleration_input_interval(initial, limits, 0.05);
      for (const double input : uniform_samples(interval, 15)) {
        const auto result = longest_acceleration_input(initial, limits, input, 0.05, 50);
        ASSERT_TRUE(result.valid);
        ASSERT_EQ(result.states.size(), 50u);
        EXPECT_DOUBLE_EQ(result.recovery_input, 0.0);
        EXPECT_NEAR(result.states.front().velocity, velocity + input * 0.05, 1.0e-12);
        auto previous = initial;
        for (std::size_t k = 0u; k < result.states.size(); ++k) {
          const auto & state = result.states[k];
          const double acceleration = k <
            static_cast<std::size_t>(result.active_input_steps) ? input : 0.0;
          EXPECT_DOUBLE_EQ(state.acceleration, acceleration);
          EXPECT_NEAR(state.velocity, previous.velocity + acceleration * 0.05, 1.0e-12);
          EXPECT_TRUE(recovery_state_valid(state, limits));
          previous = state;
        }
        if (result.active_input_steps < 50) {
          // One more nonzero tick would violate the bound; no duration search
          // or Cartesian product is hidden in the fixed candidate bank.
          EXPECT_FALSE(recovery_state_valid(
              {previous.velocity + input * 0.05, input}, limits));
        }
      }
    }
    EXPECT_FALSE(longest_acceleration_input({0.0, 0.0}, limits, 2.0, 0.05, 50).valid);
    EXPECT_FALSE(longest_acceleration_input({0.0, 0.0}, limits, 0.1, 0.0, 50).valid);
    EXPECT_FALSE(longest_acceleration_input({0.0, 0.0}, limits, 0.1, 0.05, 0).valid);
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest,
    AccelerationSaturationHas150InputsAndExactFirstStopCommand)
{
  auto node = make_node("acceleration_saturation_150", true, false, false, 0.0, 1.5, 1.5, 0.05);
  node->declare_parameter("FollowPath.vx_samples", 10);
  node->declare_parameter("FollowPath.vtheta_samples", 15);
  node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 10));
  node->set_parameter(rclcpp::Parameter("FollowPath.vtheta_samples", 15));
  node->declare_parameter("FollowPath.native_input_recovery", true);
  AccelerationTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  for (const double initial : {0.0, 0.7, 1.49, 1.5}) {
    auto snapshot = make_observable_zero_snapshot(node->now());
    snapshot.activation_state.velocity.x = initial;
    snapshot.activation_state.native_command_velocity.x = initial;
    generator.set_planning_snapshot(std::make_shared<const PlanningSnapshot>(snapshot));
    generator.startNewIteration(snapshot.activation_state.velocity);
    ASSERT_EQ(generator.candidate_count(), 150u);
    std::set<double> linear, angular;
    std::size_t certificates = 0u;
    while (generator.hasMoreTwists()) {
      const auto command = generator.nextTwist();
      const auto diagnostic = generator.active_candidate_diagnostics();
      ASSERT_TRUE(diagnostic.has_value());
      EXPECT_TRUE(diagnostic->uses_recovery);
      EXPECT_DOUBLE_EQ(diagnostic->linear_recovery_input, 0.0);
      EXPECT_DOUBLE_EQ(diagnostic->angular_recovery_input, 0.0);
      EXPECT_NEAR(command.x, initial + diagnostic->linear_native_input * 0.05, 1.0e-12);
      linear.insert(diagnostic->linear_native_input);
      angular.insert(diagnostic->angular_native_input);
      std::vector<geometry_msgs::msg::Pose2D> poses;
      std::vector<nav_2d_msgs::msg::Twist2D> velocities;
      std::vector<NativeInputTrajectoryGenerator::NativeCommandState> states;
      if (generator.generate_stop_trajectory(geometry_msgs::msg::Pose2D(),
          1000, 0.01, poses, velocities, &states))
      {
        ++certificates;
        ASSERT_FALSE(states.empty());
        EXPECT_NEAR(states.front().command_velocity.x, command.x, 1.0e-12);
        EXPECT_NEAR(states.front().command_velocity.theta, command.theta, 1.0e-12);
      }
    }
    EXPECT_EQ(linear.size(), 10u);
    EXPECT_EQ(angular.size(), 15u);
    EXPECT_GT(certificates, 0u);
  }
}

TEST(SaturationInput, JerkSelectsLatestHoldAndLeastRecoveryWithinAllLimits)
{
  const AxisLimits limits{0., 1., -1., 1., -.5, .5};
  for (const auto initial : std::vector<AxisState>{{0., 0.}, {.4, .2}, {.95, .05}, {.1, -.2}}) {
    const int horizon = 20;
    const auto intervals = jerk_switch_input_intervals(initial, limits, .05, horizon);
    for (int sample = 0; sample <= 40; ++sample) {
      const double input = -.5 + sample * .025;
      int longest = 0, most_recovery_steps = 0;
      double least_recovery = 0.;
      for (int p = 1; p <= horizon; ++p) {
        for (int m = 0; m <= horizon - p; ++m) {
          AxisState state = initial;
          bool valid = true;
          for (int k = 0; k < p; ++k) {
            state.acceleration += input * .05;
            state.velocity += state.acceleration * .05;
            valid &= recovery_state_valid(state, limits);
          }
          if (m == 0 && std::abs(state.acceleration) > 1e-12) {continue;}
          const double recovery = m == 0 ? 0. : -state.acceleration / (m * .05);
          if (std::abs(recovery) > .5 + 1e-12) {continue;}
          for (int k = p; k < horizon; ++k) {
            state.acceleration += (k < p + m ? recovery : 0.) * .05;
            state.velocity += state.acceleration * .05;
            valid &= recovery_state_valid(state, limits);
          }
          if (valid && std::abs(state.acceleration) <= 1e-9) {
            longest = p;
            most_recovery_steps = m;
            least_recovery = recovery;
          }
        }
      }
      const auto result = longest_jerk_recovery(initial, limits, input, .05, horizon);
      EXPECT_EQ(result.valid,
          longest > 0) << initial.velocity << "/" << initial.acceleration << "/" << input;
      const bool in_domain = std::any_of(intervals.begin(), intervals.end(),
          [input](const auto & interval) {
            return input >= interval.lower - 1e-10 && input <= interval.upper + 1e-10;
      });
      EXPECT_EQ(in_domain, longest > 0);
      if (result.valid) {
        EXPECT_EQ(result.active_input_steps, longest);
        EXPECT_NEAR(result.recovery_input, least_recovery, 1e-10);
        if (std::abs(least_recovery) > 1e-10) {
          EXPECT_EQ(result.recovery_steps, most_recovery_steps);
        }
        EXPECT_NEAR(result.states.front().acceleration, initial.acceleration + input * .05, 1e-12);
      }
    }
    for (double input : sample_input_intervals(intervals, 15)) {
      EXPECT_TRUE(longest_jerk_recovery(initial, limits, input, .05, horizon).valid);
    }
  }
  const auto from_rest = longest_jerk_recovery({0., 0.}, limits, .5, .05, 50);
  ASSERT_TRUE(from_rest.valid);
  EXPECT_EQ(from_rest.active_input_steps, 25);
  EXPECT_EQ(from_rest.recovery_steps, 25);
  EXPECT_NEAR(from_rest.recovery_input, -.5, 1e-12);
  EXPECT_NEAR(from_rest.states.back().velocity, .78125, 1e-12);
}

TEST(SaturationInput, FirAnalyticVelocityAndAccelerationDomainMatchDirectConvolution)
{
  const auto & coefficients = saturation_test_coefficients();
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  for (double velocity : {0., .5, .95}) {
    for (double previous : {-.1, 0., .1}) {
      const std::vector<double> memory(coefficients.size() - 1, previous);
      const auto response = prepare_zero_fir_response({velocity, previous}, limits, coefficients,
          memory, .05, 50);
      ASSERT_TRUE(response.valid);
      ASSERT_TRUE(response.monotone_integral);
      EXPECT_EQ(response.free_states.size(), 95u);
      for (double input : {-1.2, -.6, 0., .6, 1.2}) {
        const auto steps = zero_fir_velocity_steps(response, limits, input);
        int longest = 0;
        for (int n = 1; n <= 50; ++n) {
          auto history = memory;
          double v = velocity;
          bool velocity_valid = true, acceleration_valid = true;
          for (int k = 0; k < 95; ++k) {
            const double raw = k < n ? input : 0.;
            const double a = fir_acceleration(coefficients, history, raw);
            v += a * .05;
            velocity_valid &= v >= -1e-10 && v <= 1. + 1e-10;
            acceleration_valid &= a >= -1. - 1e-10 && a <= 1. + 1e-10;
            push_fir_input(history, raw);
          }
          EXPECT_EQ(n >= steps.lower && n <= steps.upper, velocity_valid);
          const auto & domain = response.duration_intervals[n];
          const bool projected = domain.feasible && input >= domain.lower - 1e-10 &&
            input <= domain.upper + 1e-10;
          EXPECT_EQ(projected, velocity_valid && acceleration_valid);
          if (velocity_valid && acceleration_valid) {longest = n;}
        }
        const auto result = longest_zero_fir_input(response, limits, input);
        EXPECT_EQ(result.valid, longest > 0);
        if (!result.valid) {continue;}
        EXPECT_EQ(result.active_input_steps, longest);
        EXPECT_DOUBLE_EQ(result.recovery_input, 0.);
        auto history = memory;
        double v = velocity;
        for (int k = 0; k < 50; ++k) {
          const double raw = k < longest ? input : 0.;
          const double a = fir_acceleration(coefficients, history, raw);
          v += a * .05;
          EXPECT_NEAR(result.states[k].velocity, v, 1e-11);
          EXPECT_NEAR(result.states[k].acceleration, a, 1e-11);
          push_fir_input(history, raw);
        }
      }
      for (double input : sample_input_intervals(response.input_intervals, 15)) {
        EXPECT_TRUE(longest_zero_fir_input(response, limits, input).valid);
      }
    }
  }
}

TEST(SaturationInput, FirAccelerationCanLimitDurationBeforeVelocityAndNeverAddsReverseInput)
{
  const auto & coefficients = saturation_test_coefficients();
  const std::vector<double> memory(coefficients.size() - 1, 0.);
  const AxisLimits limits{0., 1., -1., 1., -1.2, 1.2};
  const auto from_rest = prepare_zero_fir_response({0., 0.}, limits, coefficients, memory, .05, 50);
  EXPECT_EQ(zero_fir_velocity_steps(from_rest, limits, 1.2).upper, 15);
  const auto limited = longest_zero_fir_input(from_rest, limits, 1.2);
  ASSERT_TRUE(limited.valid);
  EXPECT_EQ(limited.active_input_steps, 8);
  EXPECT_EQ(limited.recovery_steps, 0);
  EXPECT_DOUBLE_EQ(limited.recovery_input, 0.);
  const auto cruising = prepare_zero_fir_response({.5, 0.}, limits, coefficients, memory, .05, 50);
  EXPECT_EQ(longest_zero_fir_input(cruising, limits, 1.2).active_input_steps, 7);
  const auto near_limit = prepare_zero_fir_response({.95, 0.}, limits, coefficients, memory, .05,
      50);
  EXPECT_FALSE(longest_zero_fir_input(near_limit, limits, 1.2).valid);
}

TEST(SaturationInput, NonmonotoneFirUsesExactDomainsAndSeparatedInputIntervalsKeepTheirGaps)
{
  const AxisLimits limits{-1., 1., -1., 1., -1., 1.};
  const auto response = prepare_zero_fir_response({0., 0.}, limits, {-.1, 1.1}, {0.}, .05, 10);
  ASSERT_TRUE(response.valid);
  EXPECT_FALSE(response.monotone_integral);
  const auto result = longest_zero_fir_input(response, limits, .5);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.active_input_steps, 10);
  const auto intervals = merge_input_intervals({{-1., -.5, true}, {.5, 1., true}});
  const auto inputs = sample_input_intervals(intervals, 15);
  EXPECT_EQ(inputs.size(), 15u);
  EXPECT_DOUBLE_EQ(inputs.front(), -1.);
  EXPECT_DOUBLE_EQ(inputs.back(), 1.);
  for (double input : inputs) {
    EXPECT_TRUE(input <= -.5 || input >= .5);
  }
}

}  // namespace f_dwa_controller

namespace f_dwa_controller
{

TEST_F(NativeInputTrajectoryGeneratorTest, VdwaKeeps150UniqueCandidatesAcrossZero)
{
  const auto node = make_node("v_fixed_count_test", true, false, false, 0.0, 1.0, 1.0, 0.05);
  VLimitedAccelTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("FollowPath.vx_samples", 10),
      rclcpp::Parameter("FollowPath.vtheta_samples", 15),
      rclcpp::Parameter("FollowPath.acc_lim_x", 1.0),
      rclcpp::Parameter("FollowPath.decel_lim_x", -1.0),
      rclcpp::Parameter("FollowPath.acc_lim_theta", 1.0),
      rclcpp::Parameter("FollowPath.decel_lim_theta", -1.0)}).successful);
  generator.reset();
  for (const auto & state : std::vector<std::pair<double, double>>{
      {0.0, 0.0}, {0.3, 0.0}, {0.3, 0.012}, {0.3, -0.032},
      {0.96, 0.95}, {0.001, -1e-14}, {1.0, -1.0}})
  {
    nav_2d_msgs::msg::Twist2D velocity;
    velocity.x = state.first;
    velocity.theta = state.second;
    generator.startNewIteration(velocity);
    std::set<std::pair<double, double>> commands;
    std::set<double> linear, angular;
    std::size_t count = 0u;
    while (generator.hasMoreTwists()) {
      const auto command = generator.nextTwist();
      EXPECT_GE(command.x, std::max(0.0, velocity.x - 0.05) - 1e-12);
      EXPECT_LE(command.x, std::min(1.0, velocity.x + 0.05) + 1e-12);
      EXPECT_GE(command.theta, std::max(-1.0, velocity.theta - 0.05) - 1e-12);
      EXPECT_LE(command.theta, std::min(1.0, velocity.theta + 0.05) + 1e-12);
      EXPECT_DOUBLE_EQ(command.y, 0.0);
      commands.insert({command.x, command.theta});
      linear.insert(command.x);
      angular.insert(command.theta);
      ++count;
    }
    EXPECT_EQ(count, 150u);
    EXPECT_EQ(commands.size(), 150u);
    EXPECT_EQ(linear.size(), 10u);
    EXPECT_EQ(angular.size(), 15u);
    if (std::abs(velocity.theta) <= 0.05) {
      EXPECT_EQ(angular.count(0.0), 1u);
    }
    if (velocity.x <= 0.05 && std::abs(velocity.theta) <= 0.05) {
      EXPECT_EQ(commands.count({0.0, 0.0}), 1u);
    }
  }
}

TEST_F(NativeInputTrajectoryGeneratorTest, StandardVdwaAlsoKeepsZeroInsideTheBudget)
{
  const auto node = make_node("v_standard_count_test", true, false, false, 0.0, 1.0, 1.0, 0.05);
  node->declare_parameter("FollowPath.vx_samples", 10);
  node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples", 10));
  VStandardTrajectoryGenerator generator;
  generator.initialize(node, kPluginName);
  nav_2d_msgs::msg::Twist2D velocity;
  generator.startNewIteration(velocity);
  std::set<std::pair<double, double>> commands;
  std::size_t count = 0u;
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    commands.insert({command.x, command.theta});
    ++count;
  }
  EXPECT_EQ(count, 150u);
  EXPECT_EQ(commands.size(), 150u);
  EXPECT_EQ(commands.count({0.0, 0.0}), 1u);
}

}  // namespace f_dwa_controller
