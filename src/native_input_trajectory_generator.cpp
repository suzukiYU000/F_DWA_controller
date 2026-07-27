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
#include <stdexcept>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
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
    node, plugin_name + ".applied_command_topic",
    rclcpp::ParameterValue("/controller/applied_cmd_vel"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".require_applied_command_state",
    rclcpp::ParameterValue(input_order_ == NativeInputOrder::kJerk));

  node->get_parameter(
    plugin_name + ".native_input_control_period", control_period_);
  node->get_parameter(
    plugin_name + ".max_linear_jerk", maximum_linear_jerk_);
  node->get_parameter(
    plugin_name + ".max_angular_jerk", maximum_angular_jerk_);
  node->get_parameter(plugin_name + ".vx_samples", linear_samples_);
  node->get_parameter(plugin_name + ".vtheta_samples", angular_samples_);
  node->get_parameter(
    plugin_name + ".require_applied_command_state",
    require_applied_command_state_);
  const std::string applied_command_topic =
    node->get_parameter(plugin_name + ".applied_command_topic").as_string();

  validate_parameters();
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
  {
    std::lock_guard<std::mutex> lock(applied_command_mutex_);
    if (require_applied_command_state_ && !applied_command_state_ready_) {
      return;
    }
    initial_linear_acceleration = applied_linear_acceleration_;
    initial_angular_acceleration = applied_angular_acceleration_;
  }

  const AxisState linear_state{
    current_velocity.x, initial_linear_acceleration};
  const AxisState angular_state{
    current_velocity.theta, initial_angular_acceleration};
  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  const FeasibleInterval linear_interval =
    input_interval(linear_state, linear_axis_limits, control_period_);
  const FeasibleInterval angular_interval =
    input_interval(angular_state, angular_axis_limits, control_period_);
  const std::vector<double> linear_inputs =
    uniform_samples(linear_interval, linear_samples_);
  const std::vector<double> angular_inputs =
    uniform_samples(angular_interval, angular_samples_);
  const int rollout_step_count =
    static_cast<int>(std::ceil(sim_time_ / time_granularity_));

  candidates_.reserve(linear_inputs.size() * angular_inputs.size());
  for (const double linear_input : linear_inputs) {
    const ProjectedAxisStep linear_step =
      project_axis(
      linear_state, linear_axis_limits, linear_input, control_period_,
      rollout_step_count);
    if (!linear_step.feasible) {
      continue;
    }
    for (const double angular_input : angular_inputs) {
      const ProjectedAxisStep angular_step =
        project_axis(
        angular_state, angular_axis_limits, angular_input, control_period_,
        rollout_step_count);
      if (!angular_step.feasible) {
        continue;
      }

      Candidate candidate;
      candidate.command_velocity.x = linear_step.state.velocity;
      candidate.command_velocity.y = 0.0;
      candidate.command_velocity.theta = angular_step.state.velocity;
      candidate.linear_native_input = linear_input;
      candidate.angular_native_input = angular_input;
      candidate.initial_linear_acceleration = initial_linear_acceleration;
      candidate.initial_angular_acceleration = initial_angular_acceleration;
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
    const ProjectedAxisStep linear_step =
      project_axis(
      linear_state, linear_axis_limits,
      active_candidate_.linear_native_input, time_step, remaining_steps);
    const ProjectedAxisStep angular_step =
      project_axis(
      angular_state, angular_axis_limits,
      active_candidate_.angular_native_input, time_step, remaining_steps);
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
  return project_held_jerk_step(
    state, limits, native_input_reference, time_step, remaining_steps);
}

FeasibleInterval NativeInputTrajectoryGenerator::input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step) const
{
  if (input_order_ == NativeInputOrder::kAcceleration) {
    return acceleration_input_interval(state, limits, time_step);
  }
  return jerk_input_interval(state, limits, time_step);
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
  } else {
    limits.native_input_min = -maximum_linear_jerk_;
    limits.native_input_max = maximum_linear_jerk_;
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
  } else {
    limits.native_input_min = -maximum_angular_jerk_;
    limits.native_input_max = maximum_angular_jerk_;
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
  } else {
    applied_linear_acceleration_ = 0.0;
    applied_angular_acceleration_ = 0.0;
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
