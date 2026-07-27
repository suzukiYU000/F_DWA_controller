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
#include <functional>
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
    node, plugin_name + ".native_state_reset_velocity_threshold",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".applied_command_topic",
    rclcpp::ParameterValue("/controller/applied_cmd_vel"));
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
    plugin_name + ".native_state_reset_velocity_threshold",
    native_state_reset_velocity_threshold_);
  node->get_parameter(plugin_name + ".vx_samples", linear_samples_);
  node->get_parameter(plugin_name + ".vtheta_samples", angular_samples_);
  node->get_parameter(
    plugin_name + ".require_applied_command_state",
    require_applied_command_state_);
  const std::string applied_command_topic =
    node->get_parameter(plugin_name + ".applied_command_topic").as_string();

  validate_parameters();
  if (input_order_ == NativeInputOrder::kFir) {
    applied_linear_fir_history_.assign(
      fir_coefficients_.size() - 1u, 0.0);
    applied_angular_fir_history_.assign(
      fir_coefficients_.size() - 1u, 0.0);
  }
  applied_command_subscriber_ =
    node->create_subscription<geometry_msgs::msg::Twist>(
    applied_command_topic,
    rclcpp::QoS(2).reliable(),
    std::bind(
      &NativeInputTrajectoryGenerator::applied_command_callback,
      this, std::placeholders::_1));
}

void NativeInputTrajectoryGenerator::validate_parameters() const
{
  if (!std::isfinite(control_period_) || control_period_ <= 0.0) {
    throw std::invalid_argument(
            plugin_name_ + ".native_input_control_period must be positive");
  }
  if (!std::isfinite(native_state_reset_velocity_threshold_) ||
    native_state_reset_velocity_threshold_ < 0.0)
  {
    throw std::invalid_argument(
            plugin_name_ +
            ".native_state_reset_velocity_threshold must be non-negative");
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
    if (fir_coefficients_.empty() ||
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
              " FIR coefficients and raw-input limits are invalid");
    }
  }
}

void NativeInputTrajectoryGenerator::reset()
{
  candidates_.clear();
  candidate_index_ = 0;
  has_active_candidate_ = false;
}

void NativeInputTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  reset();

  double initial_linear_acceleration = 0.0;
  double initial_angular_acceleration = 0.0;
  std::vector<double> initial_linear_fir_history;
  std::vector<double> initial_angular_fir_history;
  {
    std::lock_guard<std::mutex> lock(applied_command_mutex_);
    if (require_applied_command_state_ && !applied_command_state_ready_) {
      return;
    }
    initial_linear_acceleration = applied_linear_acceleration_;
    initial_angular_acceleration = applied_angular_acceleration_;
    initial_linear_fir_history = applied_linear_fir_history_;
    initial_angular_fir_history = applied_angular_fir_history_;
  }

  const AxisState linear_state{
    current_velocity.x, initial_linear_acceleration};
  const AxisState angular_state{
    current_velocity.theta, initial_angular_acceleration};
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
  candidates_.reserve(linear_inputs.size() * angular_inputs.size());
  for (const double linear_input : linear_inputs) {
    ProjectedAxisStep linear_step;
    ProjectedFirStep linear_fir_step;
    if (input_order_ == NativeInputOrder::kFir) {
      linear_fir_step = project_held_fir_step(
        linear_state, linear_axis_limits, fir_coefficients_,
        initial_linear_fir_history, linear_input, control_period_,
        rollout_step_count);
      linear_step.state = linear_fir_step.state;
      linear_step.feasible = linear_fir_step.feasible;
    } else {
      linear_step = project_axis(
        linear_state, linear_axis_limits, linear_input, control_period_,
        rollout_step_count);
    }
    if (!linear_step.feasible) {
      continue;
    }
    for (const double angular_input : angular_inputs) {
      ProjectedAxisStep angular_step;
      ProjectedFirStep angular_fir_step;
      if (input_order_ == NativeInputOrder::kFir) {
        angular_fir_step = project_held_fir_step(
          angular_state, angular_axis_limits, fir_coefficients_,
          initial_angular_fir_history, angular_input, control_period_,
          rollout_step_count);
        angular_step.state = angular_fir_step.state;
        angular_step.feasible = angular_fir_step.feasible;
      } else {
        angular_step = project_axis(
          angular_state, angular_axis_limits, angular_input,
          control_period_, rollout_step_count);
      }
      if (!angular_step.feasible) {
        continue;
      }

      Candidate candidate;
      candidate.command_velocity.x = linear_step.state.velocity;
      candidate.command_velocity.y = 0.0;
      candidate.command_velocity.theta = angular_step.state.velocity;
      candidate.linear_native_input = linear_input;
      candidate.angular_native_input = angular_input;
      candidate.initial_linear_velocity = current_velocity.x;
      candidate.initial_angular_velocity = current_velocity.theta;
      candidate.initial_linear_acceleration = initial_linear_acceleration;
      candidate.initial_angular_acceleration = initial_angular_acceleration;
      candidate.initial_linear_fir_history =
        initial_linear_fir_history;
      candidate.initial_angular_fir_history =
        initial_angular_fir_history;
      candidates_.push_back(candidate);
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
  const nav_2d_msgs::msg::Twist2D & start_velocity,
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
  AxisState linear_state{
    start_velocity.x, active_candidate_.initial_linear_acceleration};
  AxisState angular_state{
    start_velocity.theta, active_candidate_.initial_angular_acceleration};
  std::vector<double> linear_fir_history =
    active_candidate_.initial_linear_fir_history;
  std::vector<double> angular_fir_history =
    active_candidate_.initial_angular_fir_history;
  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  double running_time = 0.0;
  const std::vector<double> time_steps = getTimeSteps(command_velocity);
  for (std::size_t step_index = 0;
    step_index < time_steps.size(); ++step_index)
  {
    const double time_step = time_steps[step_index];
    const int remaining_steps =
      static_cast<int>(time_steps.size() - step_index);
    ProjectedAxisStep linear_step;
    ProjectedAxisStep angular_step;
    if (input_order_ == NativeInputOrder::kFir) {
      const ProjectedFirStep linear_fir_step =
        project_held_fir_step(
        linear_state, linear_axis_limits, fir_coefficients_,
        linear_fir_history, active_candidate_.linear_native_input,
        time_step, remaining_steps);
      const ProjectedFirStep angular_fir_step =
        project_held_fir_step(
        angular_state, angular_axis_limits, fir_coefficients_,
        angular_fir_history, active_candidate_.angular_native_input,
        time_step, remaining_steps);
      linear_step.state = linear_fir_step.state;
      linear_step.feasible = linear_fir_step.feasible;
      angular_step.state = angular_fir_step.state;
      angular_step.feasible = angular_fir_step.feasible;
      linear_fir_history = linear_fir_step.history;
      angular_fir_history = angular_fir_step.history;
    } else {
      linear_step = project_axis(
        linear_state, linear_axis_limits,
        active_candidate_.linear_native_input, time_step, remaining_steps);
      angular_step = project_axis(
        angular_state, angular_axis_limits,
        active_candidate_.angular_native_input, time_step, remaining_steps);
    }
    if (!linear_step.feasible || !angular_step.feasible) {
      throw dwb_core::IllegalTrajectoryException(
              "NativeInputDynamics",
              "native-input projection became infeasible during rollout");
    }
    linear_state = linear_step.state;
    angular_state = angular_step.state;

    nav_2d_msgs::msg::Twist2D velocity;
    velocity.x = linear_state.velocity;
    velocity.y = 0.0;
    velocity.theta = angular_state.velocity;
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
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities)
{
  poses.clear();
  velocities.clear();
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
  ProjectedAxisStep first_linear_step;
  ProjectedAxisStep first_angular_step;
  std::vector<double> first_linear_fir_history;
  std::vector<double> first_angular_fir_history;
  if (input_order_ == NativeInputOrder::kFir) {
    const ProjectedFirStep linear_fir_step =
      project_held_fir_step(
      initial_linear_state, linear_axis_limits, fir_coefficients_,
      active_candidate_.initial_linear_fir_history,
      active_candidate_.linear_native_input, control_period_,
      rollout_step_count);
    const ProjectedFirStep angular_fir_step =
      project_held_fir_step(
      initial_angular_state, angular_axis_limits, fir_coefficients_,
      active_candidate_.initial_angular_fir_history,
      active_candidate_.angular_native_input, control_period_,
      rollout_step_count);
    first_linear_step.state = linear_fir_step.state;
    first_linear_step.feasible = linear_fir_step.feasible;
    first_angular_step.state = angular_fir_step.state;
    first_angular_step.feasible = angular_fir_step.feasible;
    first_linear_fir_history = linear_fir_step.history;
    first_angular_fir_history = angular_fir_step.history;
  } else {
    first_linear_step = project_axis(
      initial_linear_state, linear_axis_limits,
      active_candidate_.linear_native_input, control_period_,
      rollout_step_count);
    first_angular_step = project_axis(
      initial_angular_state, angular_axis_limits,
      active_candidate_.angular_native_input, control_period_,
      rollout_step_count);
  }
  if (!first_linear_step.feasible || !first_angular_step.feasible) {
    return false;
  }

  StopSequence linear_stop;
  StopSequence angular_stop;
  if (input_order_ == NativeInputOrder::kAcceleration) {
    linear_stop = generate_acceleration_stop_sequence(
      first_linear_step.state, linear_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_acceleration_stop_sequence(
      first_angular_step.state, angular_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else if (input_order_ == NativeInputOrder::kJerk) {
    linear_stop = generate_jerk_stop_sequence(
      first_linear_step.state, linear_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_jerk_stop_sequence(
      first_angular_step.state, angular_axis_limits, control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else {
    linear_stop = generate_fir_stop_sequence(
      first_linear_step.state, linear_axis_limits, fir_coefficients_,
      first_linear_fir_history, control_period_, maximum_stop_steps,
      stop_velocity_threshold);
    angular_stop = generate_fir_stop_sequence(
      first_angular_step.state, angular_axis_limits, fir_coefficients_,
      first_angular_fir_history, control_period_, maximum_stop_steps,
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
  nav_2d_msgs::msg::Twist2D first_velocity;
  first_velocity.x = first_linear_step.state.velocity;
  first_velocity.theta = first_angular_step.state.velocity;
  pose = computeNewPosition(pose, first_velocity, control_period_);
  velocities.push_back(first_velocity);
  poses.push_back(pose);

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
  }
  nav_2d_msgs::msg::Twist2D zero_velocity;
  velocities.push_back(zero_velocity);
  poses.push_back(pose);
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

void NativeInputTrajectoryGenerator::applied_command_callback(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message ||
    !std::isfinite(message->linear.x) ||
    !std::isfinite(message->angular.z))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  if (applied_command_state_ready_) {
    applied_linear_acceleration_ =
      (message->linear.x - latest_applied_command_.linear.x) /
      control_period_;
    applied_angular_acceleration_ =
      (message->angular.z - latest_applied_command_.angular.z) /
      control_period_;
    if (input_order_ == NativeInputOrder::kFir) {
      const auto update_fir_state =
        [this](
        const double applied_velocity,
        double & applied_acceleration,
        std::vector<double> & history)
        {
          if (std::abs(applied_velocity) <=
            native_state_reset_velocity_threshold_)
          {
            applied_acceleration = 0.0;
            std::fill(history.begin(), history.end(), 0.0);
            return true;
          }
          const double free_acceleration =
            fir_acceleration(fir_coefficients_, history, 0.0);
          const double inferred_input =
            (applied_acceleration - free_acceleration) /
            fir_coefficients_.front();
          if (!std::isfinite(inferred_input)) {
            return false;
          }
          // A delayed command may be held for an extra transport tick. Treat
          // the resulting inverse-FIR input as measured disturbance history;
          // only future candidate inputs are constrained by the raw limit.
          push_fir_input(history, inferred_input);
          return true;
        };
      if (!update_fir_state(
          message->linear.x, applied_linear_acceleration_,
          applied_linear_fir_history_) ||
        !update_fir_state(
          message->angular.z, applied_angular_acceleration_,
          applied_angular_fir_history_))
      {
        applied_command_state_ready_ = false;
        latest_applied_command_ = *message;
        return;
      }
    }
  } else {
    applied_linear_acceleration_ = 0.0;
    applied_angular_acceleration_ = 0.0;
    if (input_order_ == NativeInputOrder::kFir) {
      std::fill(
        applied_linear_fir_history_.begin(),
        applied_linear_fir_history_.end(), 0.0);
      std::fill(
        applied_angular_fir_history_.begin(),
        applied_angular_fir_history_.end(), 0.0);
    }
  }
  latest_applied_command_ = *message;
  applied_command_state_ready_ = true;
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
