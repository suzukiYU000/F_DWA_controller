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
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "f_dwa_controller/certified_dwb_local_planner.hpp"
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

  static bool terminal_goal_resume(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & goal_pose,
    const double capture_distance,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const double stop_velocity_threshold)
  {
    return terminal_goal_resume_is_applicable(
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
  TerminalGoalResumeRequiresClearedMotionOutsideTolerance)
{
  geometry_msgs::msg::Pose2D goal_pose;
  goal_pose.x = 5.0;
  geometry_msgs::msg::Pose2D stopped_pose = goal_pose;
  stopped_pose.x -= 0.250001;
  nav_2d_msgs::msg::Twist2D velocity;

  EXPECT_TRUE(ScorePlannerAdapter::terminal_goal_resume(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  stopped_pose.x = goal_pose.x - 0.25;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_resume(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  stopped_pose.x = goal_pose.x - 0.250001;
  velocity.x = 0.010001;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_resume(
      stopped_pose, goal_pose, 0.25, velocity, 0.01));
  velocity.x = 0.0;
  EXPECT_FALSE(ScorePlannerAdapter::terminal_goal_resume(
      stopped_pose, goal_pose, 0.0, velocity, 0.01));
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

}  // namespace f_dwa_controller
