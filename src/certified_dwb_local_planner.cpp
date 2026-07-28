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

#include "f_dwa_controller/certified_dwb_local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

namespace
{

geometry_msgs::msg::Pose2D integrate_pose(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double time_step)
{
  geometry_msgs::msg::Pose2D next = pose;
  next.x +=
    (velocity.x * std::cos(pose.theta) -
    velocity.y * std::sin(pose.theta)) * time_step;
  next.y +=
    (velocity.x * std::sin(pose.theta) +
    velocity.y * std::cos(pose.theta)) * time_step;
  next.theta += velocity.theta * time_step;
  return next;
}

bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool commands_match(
  const nav_2d_msgs::msg::Twist2D & first,
  const nav_2d_msgs::msg::Twist2D & second)
{
  constexpr double kCommandMatchTolerance = 1.0e-9;
  return std::abs(first.x - second.x) <= kCommandMatchTolerance &&
         std::abs(first.y - second.y) <= kCommandMatchTolerance &&
         std::abs(first.theta - second.theta) <= kCommandMatchTolerance;
}

void append_integrated_motion(
  geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double duration,
  const double maximum_time_step,
  std::vector<geometry_msgs::msg::Pose2D> & poses)
{
  if (duration <= 0.0) {
    return;
  }
  const int step_count = std::max(
    1, static_cast<int>(std::ceil(duration / maximum_time_step)));
  const double time_step = duration / static_cast<double>(step_count);
  for (int step_index = 0; step_index < step_count; ++step_index) {
    pose = integrate_pose(pose, velocity, time_step);
    poses.push_back(pose);
  }
}

double quantile(
  const std::vector<double> & sorted_values,
  const double probability)
{
  if (sorted_values.empty()) {
    return 0.0;
  }
  const double index =
    probability * static_cast<double>(sorted_values.size() - 1u);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double ratio = index - static_cast<double>(lower);
  return sorted_values[lower] +
         ratio * (sorted_values[upper] - sorted_values[lower]);
}

}  // namespace

void CertifiedDWBLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock controller server node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_certification",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_nominal_delay_preview",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".nominal_delay_preview_seconds",
    rclcpp::ParameterValue(0.07));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".command_dispatch_topic",
    rclcpp::ParameterValue("/controller/command_dispatch"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".require_command_dispatch_state",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".transport_valid_topic",
    rclcpp::ParameterValue("/dwa_experiment/transport_valid"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".certification_control_period",
    rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_maximum_time",
    rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_velocity_threshold",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_certified_margin",
    rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".maximum_swept_distance",
    rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".trial_reset_service_name",
    rclcpp::ParameterValue("~/" + name + "/reset_trial_state"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_metrics_enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_metrics_report_interval",
    rclcpp::ParameterValue(1000));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_deadline_seconds",
    rclcpp::ParameterValue(0.03));

  node->get_parameter(name + ".enable_certification", certification_enabled_);
  node->get_parameter(
    name + ".enable_nominal_delay_preview",
    nominal_delay_preview_enabled_);
  node->get_parameter(
    name + ".nominal_delay_preview_seconds",
    nominal_delay_preview_seconds_);
  node->get_parameter(
    name + ".require_command_dispatch_state",
    require_command_dispatch_state_);
  node->get_parameter(
    name + ".certification_control_period",
    certification_control_period_);
  node->get_parameter(
    name + ".terminal_stop_maximum_time",
    terminal_stop_maximum_time_);
  node->get_parameter(
    name + ".terminal_stop_velocity_threshold",
    terminal_stop_velocity_threshold_);
  node->get_parameter(
    name + ".minimum_certified_margin",
    minimum_certified_margin_);
  node->get_parameter(
    name + ".maximum_swept_distance",
    maximum_swept_distance_);
  node->get_parameter(
    name + ".planning_metrics_enabled", planning_metrics_enabled_);
  node->get_parameter(
    name + ".planning_metrics_report_interval",
    planning_metrics_report_interval_);
  node->get_parameter(
    name + ".planning_deadline_seconds", planning_deadline_seconds_);
  const std::string command_dispatch_topic =
    node->get_parameter(name + ".command_dispatch_topic").as_string();
  const std::string transport_valid_topic =
    node->get_parameter(name + ".transport_valid_topic").as_string();
  const std::string trial_reset_service_name =
    node->get_parameter(name + ".trial_reset_service_name").as_string();

  if (!std::isfinite(nominal_delay_preview_seconds_) ||
    nominal_delay_preview_seconds_ < 0.0 ||
    !is_positive_finite(certification_control_period_) ||
    !is_positive_finite(terminal_stop_maximum_time_) ||
    !is_positive_finite(terminal_stop_velocity_threshold_) ||
    !std::isfinite(minimum_certified_margin_) ||
    minimum_certified_margin_ < 0.0 ||
    !is_positive_finite(maximum_swept_distance_) ||
    planning_metrics_report_interval_ <= 0 ||
    !is_positive_finite(planning_deadline_seconds_))
  {
    throw nav2_core::ControllerException(
            "Invalid delay-preview or trajectory-certification parameter");
  }

  const std::string plugin_name = name;
  dwb_core::DWBLocalPlanner::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));

  node->get_parameter(
    plugin_name + ".min_vel_x", minimum_linear_velocity_);
  node->get_parameter(
    plugin_name + ".max_vel_x", maximum_linear_velocity_);
  node->get_parameter(
    plugin_name + ".max_vel_theta", maximum_angular_velocity_);
  node->get_parameter(
    plugin_name + ".acc_lim_x", maximum_linear_acceleration_);
  node->get_parameter(
    plugin_name + ".decel_lim_x", maximum_linear_deceleration_);
  node->get_parameter(
    plugin_name + ".acc_lim_theta", maximum_angular_acceleration_);
  node->get_parameter(
    plugin_name + ".decel_lim_theta", maximum_angular_deceleration_);

  clock_ = node->get_clock();
  command_dispatch_subscriber_ =
    node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    command_dispatch_topic,
    rclcpp::QoS(64).reliable().transient_local(),
    std::bind(
      &CertifiedDWBLocalPlanner::command_dispatch_callback,
      this, std::placeholders::_1));
  transport_valid_subscriber_ =
    node->create_subscription<std_msgs::msg::Bool>(
    transport_valid_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &CertifiedDWBLocalPlanner::transport_valid_callback,
      this, std::placeholders::_1));
  reset_trial_service_ = node->create_service<std_srvs::srv::Trigger>(
    trial_reset_service_name,
    std::bind(
      &CertifiedDWBLocalPlanner::reset_trial_callback,
      this, std::placeholders::_1, std::placeholders::_2));
}

void CertifiedDWBLocalPlanner::cleanup()
{
  command_dispatch_subscriber_.reset();
  transport_valid_subscriber_.reset();
  reset_trial_service_.reset();
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  current_candidate_stop_velocities_.clear();
  current_candidate_stop_states_.clear();
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  planning_snapshot_.reset();
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    pending_issued_commands_.clear();
    command_dispatch_observed_ = false;
    command_transport_valid_ = false;
    command_ledger_valid_ = false;
    expected_dispatch_sequence_ready_ = false;
    expected_dispatch_sequence_ = 0;
  }
  dwb_core::DWBLocalPlanner::cleanup();
}

geometry_msgs::msg::TwistStamped
CertifiedDWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto planning_started_at = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  try {
    planning_snapshot_ = build_planning_snapshot(pose);
    if (!planning_snapshot_->valid) {
      throw nav2_core::NoValidControl(
              "No valid robot-observable command-dispatch state");
    }

    auto native_generator =
      std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
      traj_generator_);
    if (native_generator) {
      PlanningSnapshot enriched_snapshot = *planning_snapshot_;
      native_generator->enrich_planning_snapshot(enriched_snapshot);
      planning_snapshot_ =
        std::make_shared<const PlanningSnapshot>(
        std::move(enriched_snapshot));
      native_generator->set_planning_snapshot(planning_snapshot_);
      if (!planning_snapshot_->activation_state.native_state_valid) {
        throw nav2_core::NoValidControl(
                "Native command state is not correlated with observed dispatches");
      }
    }

    if (certification_enabled_ &&
      !planning_snapshot_->delay_trajectory.empty())
    {
      CertificationFailure failure = CertificationFailure::kInvalidInput;
      if (!certify_stop_poses(
          planning_snapshot_->delay_trajectory, failure))
      {
        throw nav2_core::NoValidControl(
                std::string("Committed delay trajectory is unsafe: ") +
                certification_failure_name(failure));
      }
    }

    nav_2d_msgs::msg::Pose2DStamped activation_pose =
      nav_2d_utils::poseStampedToPose2D(pose);
    activation_pose.pose = planning_snapshot_->activation_state.pose;
    const geometry_msgs::msg::Twist activation_velocity =
      nav_2d_utils::twist2Dto3D(
      planning_snapshot_->activation_state.velocity);
    geometry_msgs::msg::TwistStamped command =
      dwb_core::DWBLocalPlanner::computeVelocityCommands(
      nav_2d_utils::pose2DToPoseStamped(activation_pose),
      activation_velocity, goal_checker);
    const rclcpp::Time issued_at = clock_->now();
    record_issued_command(command, issued_at);
    record_planning_duration(planning_started_at);
    return command;
  } catch (...) {
    record_planning_duration(planning_started_at);
    throw;
  }
}

void CertifiedDWBLocalPlanner::command_dispatch_callback(
  const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
{
  if (!message ||
    !std::isfinite(message->command.linear.x) ||
    !std::isfinite(message->command.angular.z))
  {
    return;
  }

  std::lock_guard<std::mutex> controller_lock(controller_state_mutex_);
  const nav_2d_msgs::msg::Twist2D dispatched =
    nav_2d_utils::twist3Dto2D(message->command);
  bool dispatch_is_valid = true;
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    if (!message->has_sequence) {
      pending_issued_commands_.clear();
      expected_dispatch_sequence_ = 0;
      expected_dispatch_sequence_ready_ = true;
      command_ledger_valid_ =
        commands_match(dispatched, nav_2d_msgs::msg::Twist2D());
      dispatch_is_valid = command_ledger_valid_;
    } else {
      dispatch_is_valid =
        expected_dispatch_sequence_ready_ &&
        message->sequence_id == expected_dispatch_sequence_ &&
        !pending_issued_commands_.empty() &&
        commands_match(
        pending_issued_commands_.front().command, dispatched);
      if (dispatch_is_valid) {
        pending_issued_commands_.pop_front();
        ++expected_dispatch_sequence_;
        command_ledger_valid_ = true;
      } else {
        pending_issued_commands_.clear();
        command_ledger_valid_ = false;
      }
    }
    if (dispatch_is_valid) {
      dispatched_command_ = dispatched;
      command_dispatch_observed_ = true;
    }
  }

  if (!dispatch_is_valid) {
    RCLCPP_ERROR(
      logger_,
      "Command dispatch ledger mismatch: has_sequence=%s sequence=%" PRIu64,
      message->has_sequence ? "true" : "false",
      message->sequence_id);
    return;
  }
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    native_generator->observe_command_dispatch(*message);
  }
}

void CertifiedDWBLocalPlanner::transport_valid_callback(
  const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> controller_lock(controller_state_mutex_);
  std::lock_guard<std::mutex> command_lock(command_state_mutex_);
  command_transport_valid_ = message->data;
  if (!message->data) {
    command_dispatch_observed_ = false;
    command_ledger_valid_ = false;
    expected_dispatch_sequence_ready_ = false;
    pending_issued_commands_.clear();
  }
}

std::shared_ptr<const PlanningSnapshot>
CertifiedDWBLocalPlanner::build_planning_snapshot(
  const geometry_msgs::msg::PoseStamped & pose)
{
  PlanningSnapshot snapshot;
  snapshot.measurement_time = clock_->now();
  snapshot.activation_time =
    snapshot.measurement_time +
    rclcpp::Duration::from_seconds(
    nominal_delay_preview_enabled_ ?
    nominal_delay_preview_seconds_ : 0.0);
  snapshot.current_state.pose =
    nav_2d_utils::poseStampedToPose2D(pose).pose;
  snapshot.current_state.activation_time = snapshot.measurement_time;

  std::deque<IssuedCommand> issued_commands;
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    snapshot.dispatch_state_observed = command_dispatch_observed_;
    snapshot.valid =
      command_transport_valid_ &&
      command_ledger_valid_ &&
      (command_dispatch_observed_ || !require_command_dispatch_state_);
    snapshot.current_state.velocity = dispatched_command_;
    issued_commands = pending_issued_commands_;
  }

  geometry_msgs::msg::Pose2D rollout_pose = snapshot.current_state.pose;
  nav_2d_msgs::msg::Twist2D rollout_velocity =
    snapshot.current_state.velocity;
  rclcpp::Time rollout_time = snapshot.measurement_time;
  snapshot.delay_trajectory.push_back(rollout_pose);

  rclcpp::Time previous_activation = snapshot.measurement_time;
  for (const IssuedCommand & issued : issued_commands) {
    rclcpp::Time activation_time =
      issued.issued_at +
      rclcpp::Duration::from_seconds(nominal_delay_preview_seconds_);
    if (activation_time < previous_activation) {
      activation_time = previous_activation;
    }
    previous_activation = activation_time;
    if (activation_time > snapshot.activation_time) {
      break;
    }
    if (activation_time > rollout_time) {
      append_integrated_motion(
        rollout_pose, rollout_velocity,
        (activation_time - rollout_time).seconds(),
        certification_control_period_, snapshot.delay_trajectory);
      rollout_time = activation_time;
    }
    rollout_velocity = issued.command;
    snapshot.committed_commands.push_back(
      ScheduledCommand{activation_time, issued.command});
  }
  if (snapshot.activation_time > rollout_time) {
    append_integrated_motion(
      rollout_pose, rollout_velocity,
      (snapshot.activation_time - rollout_time).seconds(),
      certification_control_period_, snapshot.delay_trajectory);
  }

  snapshot.activation_state = snapshot.current_state;
  snapshot.activation_state.pose = rollout_pose;
  snapshot.activation_state.velocity = rollout_velocity;
  snapshot.activation_state.activation_time = snapshot.activation_time;
  return std::make_shared<const PlanningSnapshot>(std::move(snapshot));
}

void CertifiedDWBLocalPlanner::record_issued_command(
  const geometry_msgs::msg::TwistStamped & command,
  const rclcpp::Time & issued_at)
{
  const nav_2d_msgs::msg::Twist2D command_2d =
    nav_2d_utils::twist3Dto2D(command.twist);
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    constexpr std::size_t kMaximumPendingCommandCount = 256;
    if (pending_issued_commands_.size() >= kMaximumPendingCommandCount) {
      command_transport_valid_ = false;
      pending_issued_commands_.clear();
      throw nav2_core::NoValidControl(
              "Local command ledger overflow");
    }
    pending_issued_commands_.push_back(
      IssuedCommand{issued_at, command_2d});
  }
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    native_generator->commit_selected_command(command_2d, issued_at);
  }
}

void CertifiedDWBLocalPlanner::record_planning_duration(
  const std::chrono::steady_clock::time_point started_at)
{
  if (!planning_metrics_enabled_) {
    return;
  }
  const double duration_seconds =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_at).count();
  planning_durations_seconds_.push_back(duration_seconds);
  ++planning_cycle_count_;
  maximum_planning_duration_seconds_ =
    std::max(maximum_planning_duration_seconds_, duration_seconds);
  if (duration_seconds > planning_deadline_seconds_) {
    ++planning_deadline_miss_count_;
  }
  if (planning_cycle_count_ %
    static_cast<uint64_t>(planning_metrics_report_interval_) != 0u)
  {
    return;
  }
  report_planning_metrics("periodic");
}

void CertifiedDWBLocalPlanner::report_planning_metrics(
  const char * scope)
{
  if (!planning_metrics_enabled_ ||
    planning_durations_seconds_.empty())
  {
    return;
  }
  std::vector<double> sorted_durations = planning_durations_seconds_;
  std::sort(
    sorted_durations.begin(), sorted_durations.end());
  RCLCPP_INFO(
    logger_,
    "planning_timing scope=%s cycles=%" PRIu64
    " p50=%.6f p95=%.6f p99=%.6f "
    "max=%.6f deadline=%.6f misses=%" PRIu64,
    scope,
    planning_cycle_count_,
    quantile(sorted_durations, 0.50),
    quantile(sorted_durations, 0.95),
    quantile(sorted_durations, 0.99),
    maximum_planning_duration_seconds_,
    planning_deadline_seconds_,
    planning_deadline_miss_count_);
}

void CertifiedDWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  current_candidate_stop_velocities_.clear();
  current_candidate_stop_states_.clear();
  dwb_core::DWBLocalPlanner::setPlan(path);
}

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (!certification_enabled_) {
    return dwb_core::DWBLocalPlanner::coreScoringAlgorithm(
      pose, velocity, results);
  }

  dwb_msgs::msg::TrajectoryScore best;
  dwb_msgs::msg::TrajectoryScore worst;
  best.total = -1.0;
  worst.total = -1.0;
  std::vector<nav_2d_msgs::msg::Twist2D> best_stop_velocities;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
  best_stop_states;
  dwb_core::IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D twist =
      traj_generator_->nextTwist();
    const dwb_msgs::msg::Trajectory2D trajectory =
      traj_generator_->generateTrajectory(pose, velocity, twist);
    try {
      const dwb_msgs::msg::TrajectoryScore score =
        scoreTrajectory(trajectory, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0.0 || score.total < best.total) {
        best = score;
        best_stop_velocities = current_candidate_stop_velocities_;
        best_stop_states = current_candidate_stop_states_;
        if (best_stop_states.empty()) {
          auto native_generator =
            std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
            traj_generator_);
          if (native_generator) {
            const auto active_state =
              native_generator->active_candidate_command_state();
            if (active_state) {
              best_stop_states.push_back(*active_state);
            }
          }
        }
        if (results) {
          results->best_index = results->twists.size() - 1u;
        }
      }
      if (worst.total < 0.0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1u;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & exception) {
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = trajectory;
        dwb_msgs::msg::CriticScore critic_score;
        critic_score.name = exception.getCriticName();
        critic_score.raw_score = -1.0;
        failed_score.scores.push_back(critic_score);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(exception);
    }
  }

  if (best.total >= 0.0) {
    retained_backup_commands_.clear();
    retained_backup_states_.clear();
    if (best_stop_velocities.size() > 1u) {
      retained_backup_commands_.assign(
        best_stop_velocities.begin() + 1,
        best_stop_velocities.end());
    }
    if (best_stop_states.size() > 1u) {
      retained_backup_states_.assign(
        best_stop_states.begin() + 1,
        best_stop_states.end());
    }
    auto native_generator =
      std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
      traj_generator_);
    if (native_generator) {
      if (best_stop_states.empty()) {
        native_generator->select_command_for_dispatch(std::nullopt);
      } else {
        native_generator->select_command_for_dispatch(
          best_stop_states.front());
      }
    }
    return best;
  }

  dwb_msgs::msg::TrajectoryScore backup_score;
  if (build_revalidated_backup(pose, backup_score)) {
    if (results) {
      results->twists.push_back(backup_score);
      results->best_index = results->twists.size() - 1u;
    }
    if (retained_backup_commands_.size() > 1u) {
      retained_backup_commands_.erase(
        retained_backup_commands_.begin());
    }
    if (retained_backup_states_.size() > 1u) {
      retained_backup_states_.erase(retained_backup_states_.begin());
    }
    return backup_score;
  }

  if (debug_trajectory_details_) {
    RCLCPP_ERROR(logger_, "%s", tracker.getMessage().c_str());
    for (const auto & percentage : tracker.getPercentages()) {
      RCLCPP_ERROR(
        logger_, "%.2f: %10s/%s", percentage.second,
        percentage.first.first.c_str(),
        percentage.first.second.c_str());
    }
  }
  throw dwb_core::NoLegalTrajectoriesException(tracker);
}

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double best_score)
{
  current_candidate_stop_velocities_.clear();
  current_candidate_stop_states_.clear();
  dwb_msgs::msg::TrajectoryScore score =
    dwb_core::DWBLocalPlanner::scoreTrajectory(
    trajectory, best_score);
  if (certification_enabled_) {
    if (best_score >= 0.0 && score.total >= best_score) {
      return score;
    }
    std::vector<geometry_msgs::msg::Pose2D> stop_poses;
    std::vector<nav_2d_msgs::msg::Twist2D> stop_velocities;
    std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
    stop_states;
    if (!build_stop_trajectory(
        trajectory, stop_poses, stop_velocities, &stop_states))
    {
      throw dwb_core::IllegalTrajectoryException(
              "SafetyCertificate",
              "No dynamically feasible terminal stop sequence");
    }

    CertificationFailure failure = CertificationFailure::kInvalidInput;
    if (!certify_stop_poses(stop_poses, failure)) {
      throw dwb_core::IllegalTrajectoryException(
              "SafetyCertificate",
              certification_failure_name(failure));
    }
    current_candidate_stop_velocities_ = std::move(stop_velocities);
    current_candidate_stop_states_ = std::move(stop_states);
    dwb_msgs::msg::CriticScore certificate_score;
    certificate_score.name = "SafetyCertificate";
    certificate_score.scale = 1.0;
    certificate_score.raw_score = 0.0;
    score.scores.push_back(certificate_score);
  }
  return score;
}

bool CertifiedDWBLocalPlanner::build_stop_trajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState> *
  command_states)
{
  poses.clear();
  velocities.clear();
  if (command_states) {
    command_states->clear();
  }
  if (trajectory.poses.empty()) {
    return false;
  }
  const int maximum_stop_steps =
    static_cast<int>(
    std::ceil(
      terminal_stop_maximum_time_ / certification_control_period_));
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    return native_generator->generate_stop_trajectory(
      trajectory.poses.front(), maximum_stop_steps,
      terminal_stop_velocity_threshold_, poses, velocities,
      command_states);
  }

  const StopSequence linear_stop =
    generate_acceleration_stop_sequence(
    AxisState{trajectory.velocity.x, 0.0}, linear_limits(),
    certification_control_period_, maximum_stop_steps,
    terminal_stop_velocity_threshold_);
  const StopSequence angular_stop =
    generate_acceleration_stop_sequence(
    AxisState{trajectory.velocity.theta, 0.0}, angular_limits(),
    certification_control_period_, maximum_stop_steps,
    terminal_stop_velocity_threshold_);
  if (!linear_stop.feasible || !angular_stop.feasible ||
    !linear_stop.terminal_state_cleared ||
    !angular_stop.terminal_state_cleared)
  {
    return false;
  }

  geometry_msgs::msg::Pose2D pose = trajectory.poses.front();
  poses.push_back(pose);
  const int delayed_candidate_steps = std::max(
    1, static_cast<int>(
      std::ceil(
        nominal_delay_preview_seconds_ /
        certification_control_period_)));
  for (int step_index = 0;
    step_index < delayed_candidate_steps; ++step_index)
  {
    pose = integrate_pose(
      pose, trajectory.velocity, certification_control_period_);
    velocities.push_back(trajectory.velocity);
    poses.push_back(pose);
  }
  const std::size_t stop_step_count =
    std::max(linear_stop.states.size(), angular_stop.states.size());
  for (std::size_t step_index = 0;
    step_index < stop_step_count; ++step_index)
  {
    nav_2d_msgs::msg::Twist2D stop_velocity;
    if (step_index < linear_stop.states.size()) {
      stop_velocity.x = linear_stop.states[step_index].velocity;
    }
    if (step_index < angular_stop.states.size()) {
      stop_velocity.theta = angular_stop.states[step_index].velocity;
    }
    pose = integrate_pose(
      pose, stop_velocity, certification_control_period_);
    velocities.push_back(stop_velocity);
    poses.push_back(pose);
  }
  nav_2d_msgs::msg::Twist2D zero_velocity;
  velocities.push_back(zero_velocity);
  poses.push_back(pose);
  return true;
}

bool CertifiedDWBLocalPlanner::build_revalidated_backup(
  const geometry_msgs::msg::Pose2D & start_pose,
  dwb_msgs::msg::TrajectoryScore & backup_score)
{
  if (retained_backup_commands_.empty()) {
    return false;
  }

  std::vector<geometry_msgs::msg::Pose2D> poses;
  poses.reserve(retained_backup_commands_.size() + 1u);
  geometry_msgs::msg::Pose2D pose = start_pose;
  poses.push_back(pose);
  for (const nav_2d_msgs::msg::Twist2D & command :
    retained_backup_commands_)
  {
    pose = integrate_pose(
      pose, command, certification_control_period_);
    poses.push_back(pose);
  }

  CertificationFailure failure = CertificationFailure::kInvalidInput;
  if (!certify_stop_poses(poses, failure)) {
    RCLCPP_WARN(
      logger_, "Retained stop backup rejected during revalidation: %s",
      certification_failure_name(failure));
    retained_backup_commands_.clear();
    return false;
  }

  backup_score = dwb_msgs::msg::TrajectoryScore();
  backup_score.total = 0.0;
  backup_score.traj.velocity = retained_backup_commands_.front();
  backup_score.traj.poses = std::move(poses);
  dwb_msgs::msg::CriticScore certificate_score;
  certificate_score.name = "RetainedSafetyBackup";
  certificate_score.scale = 1.0;
  certificate_score.raw_score = 0.0;
  backup_score.scores.push_back(certificate_score);
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    if (retained_backup_states_.empty()) {
      native_generator->select_command_for_dispatch(std::nullopt);
    } else {
      native_generator->select_command_for_dispatch(
        retained_backup_states_.front());
    }
  }
  return true;
}

bool CertifiedDWBLocalPlanner::certify_stop_poses(
  const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
  CertificationFailure & failure) const
{
  std::vector<geometry_msgs::msg::Point> certified_footprint =
    costmap_ros_->getRobotFootprint();
  nav2_costmap_2d::padFootprint(
    certified_footprint, minimum_certified_margin_);
  const CertificationResult result =
    certify_pose_sequence(
    *costmap_ros_->getCostmap(), certified_footprint, stop_poses,
    maximum_swept_distance_);
  failure = result.failure;
  return result.safe;
}

AxisLimits CertifiedDWBLocalPlanner::linear_limits() const
{
  return AxisLimits{
    minimum_linear_velocity_, maximum_linear_velocity_,
    maximum_linear_deceleration_, maximum_linear_acceleration_,
    maximum_linear_deceleration_, maximum_linear_acceleration_};
}

AxisLimits CertifiedDWBLocalPlanner::angular_limits() const
{
  return AxisLimits{
    -maximum_angular_velocity_, maximum_angular_velocity_,
    maximum_angular_deceleration_, maximum_angular_acceleration_,
    maximum_angular_deceleration_, maximum_angular_acceleration_};
}

void CertifiedDWBLocalPlanner::reset_trial_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  report_planning_metrics("trial_end");
  planning_durations_seconds_.clear();
  planning_cycle_count_ = 0;
  planning_deadline_miss_count_ = 0;
  maximum_planning_duration_seconds_ = 0.0;
  current_candidate_stop_velocities_.clear();
  current_candidate_stop_states_.clear();
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  global_plan_.poses.clear();
  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }

  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    native_generator->reset_trial_state();
  } else if (traj_generator_) {
    traj_generator_->reset();
  }
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    pending_issued_commands_.clear();
    dispatched_command_ = nav_2d_msgs::msg::Twist2D();
    command_dispatch_observed_ = true;
    command_transport_valid_ = true;
    command_ledger_valid_ = true;
    expected_dispatch_sequence_ready_ = true;
    expected_dispatch_sequence_ = 0;
  }

  response->success = true;
  response->message =
    "controller trial state reset to zero; configured FIR coefficients retained";
  RCLCPP_INFO(logger_, "%s", response->message.c_str());
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::CertifiedDWBLocalPlanner,
  nav2_core::Controller)
