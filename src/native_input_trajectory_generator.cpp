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

#include "f_dwa_controller/native_input_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{

namespace
{

bool command_states_match(
  const nav_2d_msgs::msg::Twist2D & first,
  const nav_2d_msgs::msg::Twist2D & second)
{
  constexpr double kCommandMatchTolerance = 1.0e-9;
  return std::abs(first.x - second.x) <= kCommandMatchTolerance &&
         std::abs(first.y - second.y) <= kCommandMatchTolerance &&
         std::abs(first.theta - second.theta) <= kCommandMatchTolerance;
}

bool is_captured_stop(
  const nav_2d_msgs::msg::Twist2D & command,
  const double threshold)
{
  return std::abs(command.x) <= threshold &&
         std::abs(command.theta) <= threshold;
}

}  // namespace

NativeInputTrajectoryGenerator::NativeInputTrajectoryGenerator(
  const NativeInputOrder input_order)
: input_order_(input_order)
{
}

AccelerationTrajectoryGenerator::AccelerationTrajectoryGenerator()
: NativeInputTrajectoryGenerator(NativeInputOrder::kAcceleration)
{
}

JerkTrajectoryGenerator::JerkTrajectoryGenerator()
: NativeInputTrajectoryGenerator(NativeInputOrder::kJerk)
{
}

FirTrajectoryGenerator::FirTrajectoryGenerator()
: NativeInputTrajectoryGenerator(NativeInputOrder::kFir)
{
}

void NativeInputTrajectoryGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name)
{
  dwb_plugins::StandardTrajectoryGenerator::initialize(node, plugin_name);
  plugin_name_ = plugin_name;

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".native_input_control_period",
    rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_linear_jerk",
    rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_angular_jerk",
    rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_linear_raw_input",
    rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".max_angular_raw_input",
    rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_coefficients",
    rclcpp::ParameterValue(std::vector<double>{1.0}));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_coefficients_generated",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stop_capture_velocity",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".terminal_stop_command_delay_seconds",
    rclcpp::ParameterValue(0.07));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".require_applied_command_state",
    rclcpp::ParameterValue(input_order_ != NativeInputOrder::kAcceleration));

  node->get_parameter(
    plugin_name + ".native_input_control_period", control_period_);
  node->get_parameter(
    plugin_name + ".max_linear_jerk", maximum_linear_jerk_);
  node->get_parameter(
    plugin_name + ".max_angular_jerk", maximum_angular_jerk_);
  node->get_parameter(
    plugin_name + ".max_linear_raw_input", maximum_linear_raw_input_);
  node->get_parameter(
    plugin_name + ".max_angular_raw_input", maximum_angular_raw_input_);
  node->get_parameter(
    plugin_name + ".fir_coefficients", fir_coefficients_);
  node->get_parameter(
    plugin_name + ".fir_coefficients_generated",
    fir_coefficients_generated_);
  node->get_parameter(
    plugin_name + ".stop_capture_velocity",
    stop_capture_velocity_);
  node->get_parameter(
    plugin_name + ".terminal_stop_command_delay_seconds",
    stop_command_delay_seconds_);
  node->get_parameter(plugin_name + ".vx_samples", linear_samples_);
  node->get_parameter(plugin_name + ".vtheta_samples", angular_samples_);
  node->get_parameter(
    plugin_name + ".require_applied_command_state",
    require_applied_command_state_);
  validate_parameters();
  if (input_order_ == NativeInputOrder::kFir) {
    applied_native_state_.linear_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
    applied_native_state_.angular_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
  }
}

void NativeInputTrajectoryGenerator::validate_parameters() const
{
  if (!std::isfinite(control_period_) || control_period_ <= 0.0) {
    throw std::invalid_argument(
            plugin_name_ + ".native_input_control_period must be positive");
  }
  if (!std::isfinite(stop_capture_velocity_) ||
    stop_capture_velocity_ <= 0.0 ||
    !std::isfinite(stop_command_delay_seconds_) ||
    stop_command_delay_seconds_ < 0.0)
  {
    throw std::invalid_argument(
            plugin_name_ +
            " stop-capture and stop-delay parameters are invalid");
  }
  if (!discretize_by_time_) {
    throw std::invalid_argument(
            plugin_name_ + ".discretize_by_time must be true");
  }
  const double effective_time_step =
    sim_time_ / std::ceil(sim_time_ / time_granularity_);
  if (!std::isfinite(effective_time_step) ||
    std::abs(effective_time_step - control_period_) > 1.0e-12)
  {
    throw std::invalid_argument(
            plugin_name_ +
            ".time_granularity must produce the native input control period");
  }
  if (linear_samples_ <= 0 || angular_samples_ <= 0) {
    throw std::invalid_argument(
            plugin_name_ + " sample counts must be positive");
  }
  if (!std::isfinite(maximum_linear_jerk_) ||
    maximum_linear_jerk_ <= 0.0 ||
    !std::isfinite(maximum_angular_jerk_) ||
    maximum_angular_jerk_ <= 0.0)
  {
    throw std::invalid_argument(
            plugin_name_ + " jerk limits must be finite and positive");
  }
  if (input_order_ == NativeInputOrder::kFir) {
    const double coefficient_sum =
      std::accumulate(
      fir_coefficients_.begin(), fir_coefficients_.end(), 0.0);
    if (!fir_coefficients_generated_ ||
      fir_coefficients_.empty() ||
      !std::all_of(
        fir_coefficients_.begin(), fir_coefficients_.end(),
        [](const double value) {return std::isfinite(value);}) ||
      std::abs(fir_coefficients_.front()) <= 1.0e-12 ||
      std::abs(coefficient_sum - 1.0) > 1.0e-6 ||
      !std::isfinite(maximum_linear_raw_input_) ||
      maximum_linear_raw_input_ <= 0.0 ||
      !std::isfinite(maximum_angular_raw_input_) ||
      maximum_angular_raw_input_ <= 0.0)
    {
      throw std::invalid_argument(
              plugin_name_ +
              " FIR coefficients must be generated by the Python design "
              "stage, and coefficients and raw-input limits must be valid");
    }
  }
}

void NativeInputTrajectoryGenerator::reset()
{
  candidates_.clear();
  candidate_index_ = 0;
  has_active_candidate_ = false;
}

void NativeInputTrajectoryGenerator::reset_trial_state()
{
  reset();
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  latest_applied_command_ = nav_2d_msgs::msg::Twist2D();
  applied_dispatch_time_ = rclcpp::Time();
  applied_native_state_ = NativeCommandState();
  if (input_order_ == NativeInputOrder::kFir) {
    applied_native_state_.linear_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
    applied_native_state_.angular_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
  }
  applied_native_state_.valid = true;
  pending_native_commands_.clear();
  selected_command_state_.reset();
  planning_snapshot_.reset();
  applied_command_state_ready_ = true;
}

void NativeInputTrajectoryGenerator::enrich_planning_snapshot(
  PlanningSnapshot & snapshot) const
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  snapshot.current_state.native_state_valid =
    applied_command_state_ready_ && applied_native_state_.valid;
  snapshot.activation_state.native_state_valid =
    snapshot.current_state.native_state_valid;
  if (!snapshot.current_state.native_state_valid) {
    snapshot.valid = false;
    return;
  }

  snapshot.current_state.linear_acceleration =
    applied_native_state_.linear_state.acceleration;
  snapshot.current_state.angular_acceleration =
    applied_native_state_.angular_state.acceleration;
  snapshot.current_state.linear_fir_history =
    applied_native_state_.linear_fir_history;
  snapshot.current_state.angular_fir_history =
    applied_native_state_.angular_fir_history;
  snapshot.activation_state.linear_acceleration =
    snapshot.current_state.linear_acceleration;
  snapshot.activation_state.angular_acceleration =
    snapshot.current_state.angular_acceleration;
  snapshot.activation_state.linear_fir_history =
    snapshot.current_state.linear_fir_history;
  snapshot.activation_state.angular_fir_history =
    snapshot.current_state.angular_fir_history;

  auto pending = pending_native_commands_.cbegin();
  for (const ScheduledCommand & scheduled : snapshot.committed_commands) {
    pending = std::find_if(
      pending, pending_native_commands_.cend(),
      [&scheduled](const PendingNativeCommand & command) {
        return command_states_match(
          command.state.command_velocity, scheduled.command);
      });
    if (pending == pending_native_commands_.cend() ||
      !pending->state.valid)
    {
      snapshot.activation_state.native_state_valid = false;
      snapshot.valid = false;
      return;
    }
    snapshot.activation_state.linear_acceleration =
      pending->state.linear_state.acceleration;
    snapshot.activation_state.angular_acceleration =
      pending->state.angular_state.acceleration;
    snapshot.activation_state.linear_fir_history =
      pending->state.linear_fir_history;
    snapshot.activation_state.angular_fir_history =
      pending->state.angular_fir_history;
    ++pending;
  }
}

void NativeInputTrajectoryGenerator::set_planning_snapshot(
  std::shared_ptr<const PlanningSnapshot> snapshot)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  planning_snapshot_ = std::move(snapshot);
}

void NativeInputTrajectoryGenerator::observe_command_dispatch(
  const f_dwa_controller::msg::CommandDispatch & dispatch)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  nav_2d_msgs::msg::Twist2D dispatched;
  dispatched.x = dispatch.command.linear.x;
  dispatched.y = dispatch.command.linear.y;
  dispatched.theta = dispatch.command.angular.z;
  const rclcpp::Time dispatch_time(dispatch.header.stamp);
  if (!dispatch.has_sequence) {
    pending_native_commands_.clear();
    selected_command_state_.reset();
    if (!is_captured_stop(dispatched, stop_capture_velocity_)) {
      applied_command_state_ready_ = false;
      return;
    }
    applied_native_state_ = NativeCommandState();
    applied_native_state_.command_velocity = dispatched;
    if (input_order_ == NativeInputOrder::kFir) {
      applied_native_state_.linear_fir_history.assign(
        fir_coefficients_.size() - 1u, 0.0);
      applied_native_state_.angular_fir_history.assign(
        fir_coefficients_.size() - 1u, 0.0);
    }
    applied_native_state_.valid = true;
    latest_applied_command_ = dispatched;
    applied_dispatch_time_ = dispatch_time;
    applied_command_state_ready_ = true;
    return;
  }

  if (!pending_native_commands_.empty() &&
    command_states_match(
      pending_native_commands_.front().state.command_velocity,
      dispatched))
  {
    applied_native_state_ = pending_native_commands_.front().state;
    pending_native_commands_.pop_front();
    applied_command_state_ready_ = applied_native_state_.valid;
  } else {
    if (input_order_ != NativeInputOrder::kFir &&
      applied_command_state_ready_ &&
      dispatch_time > applied_dispatch_time_)
    {
      const double time_step =
        (dispatch_time - applied_dispatch_time_).seconds();
      applied_native_state_.command_velocity = dispatched;
      applied_native_state_.linear_state.velocity = dispatched.x;
      applied_native_state_.linear_state.acceleration =
        (dispatched.x - latest_applied_command_.x) / time_step;
      applied_native_state_.angular_state.velocity = dispatched.theta;
      applied_native_state_.angular_state.acceleration =
        (dispatched.theta - latest_applied_command_.theta) / time_step;
      applied_native_state_.valid =
        std::isfinite(applied_native_state_.linear_state.acceleration) &&
        std::isfinite(applied_native_state_.angular_state.acceleration);
      applied_command_state_ready_ = applied_native_state_.valid;
      pending_native_commands_.clear();
    } else {
      // FIR state cannot be reconstructed reliably from differentiated velocity.
      // Do not clamp an inverse estimate and call it certified.
      applied_command_state_ready_ = false;
      pending_native_commands_.clear();
    }
  }
  latest_applied_command_ = dispatched;
  applied_dispatch_time_ = dispatch_time;
}

std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
NativeInputTrajectoryGenerator::active_candidate_command_state() const
{
  if (!has_active_candidate_ ||
    !active_candidate_.first_command_state.valid)
  {
    return std::nullopt;
  }
  return active_candidate_.first_command_state;
}

void NativeInputTrajectoryGenerator::select_command_for_dispatch(
  const std::optional<NativeCommandState> & command_state)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  selected_command_state_ = command_state;
}

void NativeInputTrajectoryGenerator::commit_selected_command(
  const nav_2d_msgs::msg::Twist2D & command,
  const rclcpp::Time & issued_at)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  constexpr std::size_t kMaximumPendingNativeCommands = 256;
  if (!selected_command_state_.has_value() ||
    !selected_command_state_->valid ||
    !command_states_match(
      selected_command_state_->command_velocity, command) ||
    pending_native_commands_.size() >= kMaximumPendingNativeCommands)
  {
    selected_command_state_.reset();
    applied_command_state_ready_ = false;
    pending_native_commands_.clear();
    return;
  }
  pending_native_commands_.push_back(
    PendingNativeCommand{issued_at, *selected_command_state_});
  selected_command_state_.reset();
}

void NativeInputTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  reset();

  nav_2d_msgs::msg::Twist2D initial_velocity = current_velocity;
  double initial_linear_acceleration = 0.0;
  double initial_angular_acceleration = 0.0;
  std::vector<double> initial_linear_fir_history;
  std::vector<double> initial_angular_fir_history;
  {
    std::lock_guard<std::mutex> lock(applied_command_mutex_);
    if (planning_snapshot_) {
      if (!planning_snapshot_->valid ||
        !planning_snapshot_->activation_state.native_state_valid)
      {
        return;
      }
      initial_velocity =
        planning_snapshot_->activation_state.velocity;
      initial_linear_acceleration =
        planning_snapshot_->activation_state.linear_acceleration;
      initial_angular_acceleration =
        planning_snapshot_->activation_state.angular_acceleration;
      initial_linear_fir_history =
        planning_snapshot_->activation_state.linear_fir_history;
      initial_angular_fir_history =
        planning_snapshot_->activation_state.angular_fir_history;
    } else {
      if (require_applied_command_state_ &&
        !applied_command_state_ready_)
      {
        return;
      }
      initial_velocity = latest_applied_command_;
      initial_linear_acceleration =
        applied_native_state_.linear_state.acceleration;
      initial_angular_acceleration =
        applied_native_state_.angular_state.acceleration;
      initial_linear_fir_history =
        applied_native_state_.linear_fir_history;
      initial_angular_fir_history =
        applied_native_state_.angular_fir_history;
    }
  }

  const AxisState linear_state{
    initial_velocity.x, initial_linear_acceleration};
  const AxisState angular_state{
    initial_velocity.theta, initial_angular_acceleration};
  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  const int rollout_step_count =
    static_cast<int>(std::ceil(sim_time_ / time_granularity_));
  const FeasibleInterval linear_interval =
    input_order_ == NativeInputOrder::kFir ?
    held_fir_input_interval(
    linear_state, linear_axis_limits, fir_coefficients_,
    initial_linear_fir_history, control_period_, rollout_step_count) :
    input_interval(linear_state, linear_axis_limits, control_period_);
  const FeasibleInterval angular_interval =
    input_order_ == NativeInputOrder::kFir ?
    held_fir_input_interval(
    angular_state, angular_axis_limits, fir_coefficients_,
    initial_angular_fir_history, control_period_, rollout_step_count) :
    input_interval(angular_state, angular_axis_limits, control_period_);
  const std::vector<double> linear_inputs =
    uniform_samples(linear_interval, linear_samples_);
  const std::vector<double> angular_inputs =
    uniform_samples(angular_interval, angular_samples_);
  const auto build_axis_rollouts =
    [this, rollout_step_count](
    const std::vector<double> & inputs,
    const AxisState & initial_state,
    const AxisLimits & limits,
    const std::vector<double> & initial_fir_history)
    {
      std::vector<std::shared_ptr<const AxisRollout>> rollouts;
      rollouts.reserve(inputs.size());
      for (const double input : inputs) {
        auto rollout = std::make_shared<AxisRollout>();
        rollout->native_input = input;
        rollout->states.reserve(
          static_cast<std::size_t>(rollout_step_count));
        if (input_order_ == NativeInputOrder::kFir) {
          rollout->fir_histories.reserve(
            static_cast<std::size_t>(rollout_step_count));
        }
        AxisState state = initial_state;
        std::vector<double> fir_history = initial_fir_history;
        for (int step_index = 0;
          step_index < rollout_step_count; ++step_index)
        {
          ProjectedAxisStep step;
          if (input_order_ == NativeInputOrder::kFir) {
            const ProjectedFirStep fir_step =
              apply_projected_fir_step(
              state, limits, fir_coefficients_, fir_history,
              input, control_period_);
            step.state = fir_step.state;
            step.feasible = fir_step.feasible;
            fir_history = fir_step.history;
          } else {
            step = project_axis(
              state, limits, input, control_period_,
              rollout_step_count - step_index);
          }
          if (!step.feasible) {
            break;
          }
          state = step.state;
          rollout->states.push_back(state);
          if (input_order_ == NativeInputOrder::kFir) {
            rollout->fir_histories.push_back(fir_history);
          }
        }
        rollout->valid =
          rollout->states.size() ==
          static_cast<std::size_t>(rollout_step_count);
        if (rollout->valid) {
          rollouts.push_back(std::move(rollout));
        }
      }
      return rollouts;
    };
  const auto linear_rollouts =
    build_axis_rollouts(
    linear_inputs, linear_state, linear_axis_limits,
    initial_linear_fir_history);
  const auto angular_rollouts =
    build_axis_rollouts(
    angular_inputs, angular_state, angular_axis_limits,
    initial_angular_fir_history);

  candidates_.reserve(linear_rollouts.size() * angular_rollouts.size());
  for (const auto & linear_rollout : linear_rollouts) {
    for (const auto & angular_rollout : angular_rollouts) {
      Candidate candidate;
      candidate.command_velocity.x =
        linear_rollout->states.front().velocity;
      candidate.command_velocity.y = 0.0;
      candidate.command_velocity.theta =
        angular_rollout->states.front().velocity;
      candidate.linear_native_input = linear_rollout->native_input;
      candidate.angular_native_input = angular_rollout->native_input;
      candidate.initial_linear_velocity = initial_velocity.x;
      candidate.initial_angular_velocity = initial_velocity.theta;
      candidate.initial_linear_acceleration = initial_linear_acceleration;
      candidate.initial_angular_acceleration = initial_angular_acceleration;
      candidate.initial_linear_fir_history =
        initial_linear_fir_history;
      candidate.initial_angular_fir_history =
        initial_angular_fir_history;
      candidate.first_command_state.command_velocity =
        candidate.command_velocity;
      candidate.first_command_state.linear_state =
        linear_rollout->states.front();
      candidate.first_command_state.angular_state =
        angular_rollout->states.front();
      if (input_order_ == NativeInputOrder::kFir) {
        candidate.first_command_state.linear_fir_history =
          linear_rollout->fir_histories.front();
        candidate.first_command_state.angular_fir_history =
          angular_rollout->fir_histories.front();
      }
      candidate.first_command_state.valid = true;
      candidate.linear_rollout = linear_rollout;
      candidate.angular_rollout = angular_rollout;
      candidates_.push_back(std::move(candidate));
    }
  }
}

bool NativeInputTrajectoryGenerator::hasMoreTwists()
{
  return candidate_index_ < candidates_.size();
}

nav_2d_msgs::msg::Twist2D NativeInputTrajectoryGenerator::nextTwist()
{
  if (!hasMoreTwists()) {
    throw std::out_of_range("No native-input trajectory candidates remain");
  }
  active_candidate_ = candidates_[candidate_index_];
  has_active_candidate_ = true;
  ++candidate_index_;
  return active_candidate_.command_velocity;
}

dwb_msgs::msg::Trajectory2D
NativeInputTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & /*start_velocity*/,
  const nav_2d_msgs::msg::Twist2D & command_velocity)
{
  if (!has_active_candidate_) {
    throw dwb_core::IllegalTrajectoryException(
            "NativeInputDynamics",
            "generateTrajectory called without an active native-input candidate");
  }

  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity = command_velocity;
  trajectory.poses.push_back(start_pose);

  geometry_msgs::msg::Pose2D pose = start_pose;
  double running_time = 0.0;
  const std::vector<double> time_steps = getTimeSteps(command_velocity);
  if (!active_candidate_.linear_rollout ||
    !active_candidate_.angular_rollout ||
    active_candidate_.linear_rollout->states.size() != time_steps.size() ||
    active_candidate_.angular_rollout->states.size() != time_steps.size())
  {
    throw dwb_core::IllegalTrajectoryException(
            "NativeInputDynamics",
            "precomputed axis rollout does not match the trajectory horizon");
  }
  for (std::size_t step_index = 0;
    step_index < time_steps.size(); ++step_index)
  {
    const double time_step = time_steps[step_index];

    nav_2d_msgs::msg::Twist2D velocity;
    velocity.x =
      active_candidate_.linear_rollout->states[step_index].velocity;
    velocity.y = 0.0;
    velocity.theta =
      active_candidate_.angular_rollout->states[step_index].velocity;
    pose = computeNewPosition(pose, velocity, time_step);
    trajectory.poses.push_back(pose);
    trajectory.time_offsets.push_back(
      rclcpp::Duration::from_seconds(running_time));
    running_time += time_step;
  }
  if (include_last_point_) {
    trajectory.poses.push_back(pose);
    trajectory.time_offsets.push_back(
      rclcpp::Duration::from_seconds(running_time));
  }
  return trajectory;
}

bool NativeInputTrajectoryGenerator::generate_stop_trajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
  std::vector<NativeCommandState> * command_states)
{
  poses.clear();
  velocities.clear();
  if (command_states) {
    command_states->clear();
  }
  if (!has_active_candidate_ || maximum_stop_steps <= 0 ||
    !std::isfinite(stop_velocity_threshold) ||
    stop_velocity_threshold <= 0.0)
  {
    return false;
  }

  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  const AxisState initial_linear_state{
    active_candidate_.initial_linear_velocity,
    active_candidate_.initial_linear_acceleration};
  const AxisState initial_angular_state{
    active_candidate_.initial_angular_velocity,
    active_candidate_.initial_angular_acceleration};
  const int rollout_step_count =
    static_cast<int>(std::ceil(sim_time_ / time_granularity_));
  const int command_delay_steps = std::max(
    1, static_cast<int>(
      std::ceil(stop_command_delay_seconds_ / control_period_)));
  AxisState delayed_linear_state = initial_linear_state;
  AxisState delayed_angular_state = initial_angular_state;
  std::vector<double> delayed_linear_fir_history =
    active_candidate_.initial_linear_fir_history;
  std::vector<double> delayed_angular_fir_history =
    active_candidate_.initial_angular_fir_history;
  std::vector<NativeCommandState> delayed_command_states;
  delayed_command_states.reserve(
    static_cast<std::size_t>(command_delay_steps));
  for (int step_index = 0;
    step_index < command_delay_steps; ++step_index)
  {
    ProjectedAxisStep linear_step;
    ProjectedAxisStep angular_step;
    if (input_order_ == NativeInputOrder::kFir) {
      const ProjectedFirStep linear_fir_step =
        apply_projected_fir_step(
        delayed_linear_state, linear_axis_limits, fir_coefficients_,
        delayed_linear_fir_history,
        active_candidate_.linear_native_input, control_period_);
      const ProjectedFirStep angular_fir_step =
        apply_projected_fir_step(
        delayed_angular_state, angular_axis_limits, fir_coefficients_,
        delayed_angular_fir_history,
        active_candidate_.angular_native_input, control_period_);
      linear_step.state = linear_fir_step.state;
      linear_step.feasible = linear_fir_step.feasible;
      angular_step.state = angular_fir_step.state;
      angular_step.feasible = angular_fir_step.feasible;
      delayed_linear_fir_history = linear_fir_step.history;
      delayed_angular_fir_history = angular_fir_step.history;
    } else {
      const int remaining_steps =
        std::max(1, rollout_step_count - step_index);
      linear_step = project_axis(
        delayed_linear_state, linear_axis_limits,
        active_candidate_.linear_native_input, control_period_,
        remaining_steps);
      angular_step = project_axis(
        delayed_angular_state, angular_axis_limits,
        active_candidate_.angular_native_input, control_period_,
        remaining_steps);
    }
    if (!linear_step.feasible || !angular_step.feasible) {
      return false;
    }
    delayed_linear_state = linear_step.state;
    delayed_angular_state = angular_step.state;
    NativeCommandState delayed_state;
    delayed_state.command_velocity.x = delayed_linear_state.velocity;
    delayed_state.command_velocity.theta =
      delayed_angular_state.velocity;
    delayed_state.linear_state = delayed_linear_state;
    delayed_state.angular_state = delayed_angular_state;
    delayed_state.linear_fir_history = delayed_linear_fir_history;
    delayed_state.angular_fir_history = delayed_angular_fir_history;
    delayed_state.valid = true;
    delayed_command_states.push_back(std::move(delayed_state));
  }

  StopSequence linear_stop;
  StopSequence angular_stop;
  if (input_order_ == NativeInputOrder::kAcceleration) {
    linear_stop = generate_acceleration_stop_sequence(
      delayed_linear_state, linear_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_acceleration_stop_sequence(
      delayed_angular_state, angular_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else if (input_order_ == NativeInputOrder::kJerk) {
    linear_stop = generate_jerk_stop_sequence(
      delayed_linear_state, linear_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_jerk_stop_sequence(
      delayed_angular_state, angular_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else {
    linear_stop = generate_fir_stop_sequence(
      delayed_linear_state, linear_axis_limits, fir_coefficients_,
      delayed_linear_fir_history, control_period_, maximum_stop_steps,
      stop_velocity_threshold);
    angular_stop = generate_fir_stop_sequence(
      delayed_angular_state, angular_axis_limits, fir_coefficients_,
      delayed_angular_fir_history, control_period_, maximum_stop_steps,
      stop_velocity_threshold);
  }
  if (!linear_stop.feasible || !angular_stop.feasible ||
    !linear_stop.terminal_state_cleared ||
    !angular_stop.terminal_state_cleared)
  {
    return false;
  }

  geometry_msgs::msg::Pose2D pose = start_pose;
  poses.push_back(pose);
  for (const NativeCommandState & delayed_state :
    delayed_command_states)
  {
    pose = computeNewPosition(
      pose, delayed_state.command_velocity, control_period_);
    velocities.push_back(delayed_state.command_velocity);
    poses.push_back(pose);
    if (command_states) {
      command_states->push_back(delayed_state);
    }
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
    pose = computeNewPosition(pose, stop_velocity, control_period_);
    velocities.push_back(stop_velocity);
    poses.push_back(pose);
    if (command_states) {
      NativeCommandState stop_state;
      stop_state.command_velocity = stop_velocity;
      if (step_index < linear_stop.states.size()) {
        stop_state.linear_state = linear_stop.states[step_index];
      }
      if (step_index < angular_stop.states.size()) {
        stop_state.angular_state = angular_stop.states[step_index];
      }
      if (input_order_ == NativeInputOrder::kFir) {
        if (step_index < linear_stop.fir_histories.size()) {
          stop_state.linear_fir_history =
            linear_stop.fir_histories[step_index];
        } else {
          stop_state.linear_fir_history.assign(
            fir_coefficients_.size() - 1u, 0.0);
        }
        if (step_index < angular_stop.fir_histories.size()) {
          stop_state.angular_fir_history =
            angular_stop.fir_histories[step_index];
        } else {
          stop_state.angular_fir_history.assign(
            fir_coefficients_.size() - 1u, 0.0);
        }
      }
      stop_state.valid = true;
      command_states->push_back(std::move(stop_state));
    }
  }
  nav_2d_msgs::msg::Twist2D zero_velocity;
  velocities.push_back(zero_velocity);
  poses.push_back(pose);
  if (command_states) {
    NativeCommandState zero_state;
    zero_state.command_velocity = zero_velocity;
    if (input_order_ == NativeInputOrder::kFir) {
      zero_state.linear_fir_history.assign(
        fir_coefficients_.size() - 1u, 0.0);
      zero_state.angular_fir_history.assign(
        fir_coefficients_.size() - 1u, 0.0);
    }
    zero_state.valid = true;
    command_states->push_back(std::move(zero_state));
  }
  return true;
}

ProjectedAxisStep NativeInputTrajectoryGenerator::project_axis(
  const AxisState & state,
  const AxisLimits & limits,
  const double native_input_reference,
  const double time_step,
  const int remaining_steps) const
{
  if (input_order_ == NativeInputOrder::kAcceleration) {
    return project_held_acceleration_step(
      state, limits, native_input_reference, time_step, remaining_steps);
  }
  if (input_order_ == NativeInputOrder::kJerk) {
    return project_held_jerk_step(
      state, limits, native_input_reference, time_step, remaining_steps);
  }
  return {};
}

FeasibleInterval NativeInputTrajectoryGenerator::input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step) const
{
  if (input_order_ == NativeInputOrder::kAcceleration) {
    return acceleration_input_interval(state, limits, time_step);
  }
  if (input_order_ == NativeInputOrder::kJerk) {
    return jerk_input_interval(state, limits, time_step);
  }
  return {};
}

AxisLimits NativeInputTrajectoryGenerator::linear_limits() const
{
  dwb_plugins::KinematicParameters kinematics =
    kinematics_handler_->getKinematics();
  AxisLimits limits;
  limits.velocity_min = kinematics.getMinX();
  limits.velocity_max =
    std::min(kinematics.getMaxX(), kinematics.getMaxSpeedXY());
  limits.acceleration_min = kinematics.getDecelX();
  limits.acceleration_max = kinematics.getAccX();
  if (input_order_ == NativeInputOrder::kAcceleration) {
    limits.native_input_min = limits.acceleration_min;
    limits.native_input_max = limits.acceleration_max;
  } else if (input_order_ == NativeInputOrder::kJerk) {
    limits.native_input_min = -maximum_linear_jerk_;
    limits.native_input_max = maximum_linear_jerk_;
  } else {
    limits.native_input_min = -maximum_linear_raw_input_;
    limits.native_input_max = maximum_linear_raw_input_;
  }
  return limits;
}

AxisLimits NativeInputTrajectoryGenerator::angular_limits() const
{
  dwb_plugins::KinematicParameters kinematics =
    kinematics_handler_->getKinematics();
  AxisLimits limits;
  limits.velocity_min = kinematics.getMinTheta();
  limits.velocity_max = kinematics.getMaxTheta();
  limits.acceleration_min = kinematics.getDecelTheta();
  limits.acceleration_max = kinematics.getAccTheta();
  if (input_order_ == NativeInputOrder::kAcceleration) {
    limits.native_input_min = limits.acceleration_min;
    limits.native_input_max = limits.acceleration_max;
  } else if (input_order_ == NativeInputOrder::kJerk) {
    limits.native_input_min = -maximum_angular_jerk_;
    limits.native_input_max = maximum_angular_jerk_;
  } else {
    limits.native_input_min = -maximum_angular_raw_input_;
    limits.native_input_max = maximum_angular_raw_input_;
  }
  return limits;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::AccelerationTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::JerkTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::FirTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
