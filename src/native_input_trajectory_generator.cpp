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
#include <cinttypes>
#include <cmath>
#include <iterator>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/equal_effect_fir_sampling.hpp"
#include "f_dwa_controller/saturation_input_dynamics.hpp"
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
  node_ = node;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".native_input_recovery", rclcpp::ParameterValue(false));
  node->get_parameter(plugin_name + ".native_input_recovery", native_input_recovery_);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".native_input_pulse_duration", rclcpp::ParameterValue(0.0));
  node->get_parameter(plugin_name + ".native_input_pulse_duration", native_input_pulse_duration_);
  if (!std::isfinite(native_input_pulse_duration_) || native_input_pulse_duration_ < 0.0) {
    throw std::invalid_argument("native_input_pulse_duration must be finite and nonnegative");
  }
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
    node, plugin_name + ".fir_cutoff_frequency_hz",
    rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_filter_specification",
    rclcpp::ParameterValue(std::string{}));
  // Zero preserves coefficient-only callers. Generated GUI profiles provide
  // the explicit effective count, which must agree with the executed filter.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_effective_taps", rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_prediction_pulse_duration",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_prediction_pulse_durations",
    rclcpp::ParameterValue(std::vector<double>{}));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_independent_pulse_durations",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fir_equal_effect_max_duration_sampling",
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
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".prefer_previous_selected_candidate",
    rclcpp::ParameterValue(false));

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
    plugin_name + ".fir_cutoff_frequency_hz",
    fir_cutoff_frequency_hz_);
  node->get_parameter(
    plugin_name + ".fir_filter_specification",
    fir_filter_specification_);
  node->get_parameter(
    plugin_name + ".fir_prediction_pulse_duration",
    fir_prediction_pulse_duration_);
  node->get_parameter(
    plugin_name + ".fir_prediction_pulse_durations",
    fir_prediction_pulse_durations_);
  node->get_parameter(
    plugin_name + ".fir_independent_pulse_durations",
    fir_independent_pulse_durations_);
  node->get_parameter(
    plugin_name + ".fir_equal_effect_max_duration_sampling",
    fir_equal_effect_max_duration_sampling_);
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
  node->get_parameter(
    plugin_name + ".prefer_previous_selected_candidate",
    prefer_previous_selected_candidate_);
  validate_parameters();
  fixed_time_steps_ = getTimeSteps(nav_2d_msgs::msg::Twist2D());
  if (input_order_ == NativeInputOrder::kFir) {
    fir_stop_coefficient_response_ =
      prepare_fir_stop_coefficient_response(
      fir_coefficients_, control_period_);
    if (!fir_stop_coefficient_response_.valid) {
      throw std::invalid_argument(
              plugin_name_ +
              " FIR terminal-stop coefficient response is invalid");
    }
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
    const auto node = node_.lock();
    if (!node) {throw std::runtime_error("FIR Controller node is unavailable");}
    const auto effective_taps = node->get_parameter(plugin_name_ + ".fir_effective_taps").as_int();
    if (effective_taps < 0 || (effective_taps > 0 &&
      static_cast<std::size_t>(effective_taps) != fir_coefficients_.size()))
    {
      throw std::invalid_argument(plugin_name_ + ".fir_effective_taps must match fir_coefficients");
    }
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
      maximum_angular_raw_input_ <= 0.0 ||
      !std::isfinite(fir_cutoff_frequency_hz_) ||
      fir_cutoff_frequency_hz_ <= 0.0 ||
      fir_cutoff_frequency_hz_ >= 10.0 ||
      !std::isfinite(fir_prediction_pulse_duration_) ||
      fir_prediction_pulse_duration_<0.0 ||
      fir_prediction_pulse_duration_> sim_time_ + 1.0e-12)
    {
      throw std::invalid_argument(
              plugin_name_ +
              " FIR coefficients must be generated by the Python design "
              "stage, and coefficients and raw-input limits must be valid");
    }
    for (const double duration : fir_prediction_pulse_durations_) {
      if (!std::isfinite(duration) || duration < 0.0 ||
        duration > sim_time_ + 1.0e-12)
      {
        throw std::invalid_argument(
                plugin_name_ + ".fir_prediction_pulse_durations entries "
                "must be finite and within [0, sim_time] seconds");
      }
    }
    if (fir_equal_effect_max_duration_sampling_ &&
      (native_input_recovery_ || fir_independent_pulse_durations_ ||
      !fir_prediction_pulse_durations_.empty() ||
      linear_samples_ % 2 == 0 || angular_samples_ % 2 == 0))
    {
      throw std::invalid_argument(
              plugin_name_ +
              ".fir_equal_effect_max_duration_sampling requires recovery "
              "off, no duration bank, synchronized axes, and odd sample counts");
    }
  }
}

std::vector<int> NativeInputTrajectoryGenerator::prediction_input_step_counts() const
{
  const int horizon_steps = static_cast<int>(fixed_time_steps_.size());
  if (input_order_ != NativeInputOrder::kFir) {
    if (input_order_ == NativeInputOrder::kJerk && native_input_pulse_duration_ > 0.0) {
      return {std::min(horizon_steps, std::max(1,
        static_cast<int>(std::ceil(native_input_pulse_duration_ / control_period_ - 1.0e-12))))};
    }
    return {horizon_steps};
  }
  std::vector<int> step_counts;
  const auto append_duration = [&step_counts, horizon_steps, this](const double duration) {
      const int steps = duration > 0.0 ?
        std::min(horizon_steps, std::max(
          1, static_cast<int>(std::ceil(duration / control_period_ - 1.0e-12)))) :
        horizon_steps;
      if (std::find(step_counts.begin(), step_counts.end(), steps) == step_counts.end()) {
        step_counts.push_back(steps);
      }
    };
  // Keep the original action set first, including its canonical tie order.
  // Durations that quantize to the same controller tick are evaluated once.
  append_duration(fir_prediction_pulse_duration_);
  for (const double duration : fir_prediction_pulse_durations_) {
    append_duration(duration);
  }
  return step_counts;
}

void NativeInputTrajectoryGenerator::reset()
{
  candidates_.clear();
  candidate_order_.clear();
  candidate_index_ = 0;
  active_candidate_ = nullptr;
}

void NativeInputTrajectoryGenerator::reset_trial_state()
{
  // The controller reset service holds CertifiedDWBLocalPlanner's state mutex,
  // so this is the safe boundary at which GUI parameter changes become active.
  // No candidate can be evaluated with a partially updated J/F-DWA model.
  reload_runtime_parameters();
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
  previous_selected_velocity_.reset();
  planning_snapshot_.reset();
  applied_command_state_ready_ = true;
}

void NativeInputTrajectoryGenerator::reload_runtime_parameters()
{
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(plugin_name_ + " lifecycle node is unavailable");
  }

  double updated_linear_jerk = maximum_linear_jerk_;
  double updated_angular_jerk = maximum_angular_jerk_;
  int updated_linear_samples = linear_samples_;
  int updated_angular_samples = angular_samples_;
  double updated_sim_time = sim_time_;
  bool updated_native_input_recovery = native_input_recovery_;
  double updated_native_input_pulse_duration = native_input_pulse_duration_;
  double updated_pulse_duration = fir_prediction_pulse_duration_;
  std::vector<double> updated_pulse_durations = fir_prediction_pulse_durations_;
  bool updated_independent_durations = fir_independent_pulse_durations_;
  bool updated_equal_effect_sampling =
    fir_equal_effect_max_duration_sampling_;
  double updated_cutoff_frequency = fir_cutoff_frequency_hz_;
  std::string updated_filter_specification = fir_filter_specification_;
  bool updated_coefficients_generated = fir_coefficients_generated_;
  std::vector<double> updated_coefficients = fir_coefficients_;
  node->get_parameter(
    plugin_name_ + ".max_linear_jerk", updated_linear_jerk);
  node->get_parameter(
    plugin_name_ + ".max_angular_jerk", updated_angular_jerk);
  node->get_parameter(plugin_name_ + ".sim_time", updated_sim_time);
  node->get_parameter(plugin_name_ + ".vx_samples", updated_linear_samples);
  node->get_parameter(plugin_name_ + ".vtheta_samples", updated_angular_samples);
  node->get_parameter(
    plugin_name_ + ".native_input_recovery",
    updated_native_input_recovery);
  node->get_parameter(
    plugin_name_ + ".native_input_pulse_duration",
    updated_native_input_pulse_duration);
  if (input_order_ == NativeInputOrder::kFir) {
    node->get_parameter(
      plugin_name_ + ".fir_coefficients", updated_coefficients);
    node->get_parameter(
      plugin_name_ + ".fir_coefficients_generated",
      updated_coefficients_generated);
    node->get_parameter(
      plugin_name_ + ".fir_cutoff_frequency_hz",
      updated_cutoff_frequency);
    node->get_parameter(
      plugin_name_ + ".fir_filter_specification",
      updated_filter_specification);
    node->get_parameter(
      plugin_name_ + ".fir_prediction_pulse_duration",
      updated_pulse_duration);
    node->get_parameter(
      plugin_name_ + ".fir_prediction_pulse_durations",
      updated_pulse_durations);
    node->get_parameter(
      plugin_name_ + ".fir_independent_pulse_durations",
      updated_independent_durations);
    node->get_parameter(
      plugin_name_ + ".fir_equal_effect_max_duration_sampling",
      updated_equal_effect_sampling);
  }

  const double previous_linear_jerk = maximum_linear_jerk_;
  const double previous_angular_jerk = maximum_angular_jerk_;
  const int previous_linear_samples = linear_samples_;
  const int previous_angular_samples = angular_samples_;
  const double previous_sim_time = sim_time_;
  const bool previous_native_input_recovery = native_input_recovery_;
  const double previous_native_input_pulse_duration =
    native_input_pulse_duration_;
  const double previous_pulse_duration = fir_prediction_pulse_duration_;
  const std::vector<double> previous_pulse_durations = fir_prediction_pulse_durations_;
  const bool previous_independent_durations = fir_independent_pulse_durations_;
  const bool previous_equal_effect_sampling =
    fir_equal_effect_max_duration_sampling_;
  const double previous_cutoff_frequency = fir_cutoff_frequency_hz_;
  const std::string previous_filter_specification =
    fir_filter_specification_;
  const bool previous_coefficients_generated = fir_coefficients_generated_;
  const std::vector<double> previous_coefficients = fir_coefficients_;
  const FirStopCoefficientResponse previous_stop_response =
    fir_stop_coefficient_response_;
  const std::vector<double> previous_time_steps = fixed_time_steps_;

  maximum_linear_jerk_ = updated_linear_jerk;
  maximum_angular_jerk_ = updated_angular_jerk;
  linear_samples_ = updated_linear_samples;
  angular_samples_ = updated_angular_samples;
  sim_time_ = updated_sim_time;
  native_input_recovery_ = updated_native_input_recovery;
  native_input_pulse_duration_ = updated_native_input_pulse_duration;
  fir_prediction_pulse_duration_ = updated_pulse_duration;
  fir_prediction_pulse_durations_ = std::move(updated_pulse_durations);
  fir_independent_pulse_durations_ = updated_independent_durations;
  fir_equal_effect_max_duration_sampling_ = updated_equal_effect_sampling;
  fir_cutoff_frequency_hz_ = updated_cutoff_frequency;
  fir_filter_specification_ = std::move(updated_filter_specification);
  fir_coefficients_generated_ = updated_coefficients_generated;
  fir_coefficients_ = std::move(updated_coefficients);
  try {
    if (!std::isfinite(native_input_pulse_duration_) ||
      native_input_pulse_duration_ < 0.0)
    {
      throw std::invalid_argument(
              "native_input_pulse_duration must be finite and nonnegative");
    }
    validate_parameters();
    fixed_time_steps_ = getTimeSteps(nav_2d_msgs::msg::Twist2D());
    if (input_order_ == NativeInputOrder::kFir) {
      fir_stop_coefficient_response_ =
        prepare_fir_stop_coefficient_response(
        fir_coefficients_, control_period_);
      if (!fir_stop_coefficient_response_.valid) {
        throw std::invalid_argument(
                plugin_name_ +
                " FIR terminal-stop coefficient response is invalid");
      }
    }
  } catch (...) {
    maximum_linear_jerk_ = previous_linear_jerk;
    maximum_angular_jerk_ = previous_angular_jerk;
    linear_samples_ = previous_linear_samples;
    angular_samples_ = previous_angular_samples;
    sim_time_ = previous_sim_time;
    native_input_recovery_ = previous_native_input_recovery;
    native_input_pulse_duration_ = previous_native_input_pulse_duration;
    fir_prediction_pulse_duration_ = previous_pulse_duration;
    fir_prediction_pulse_durations_ = previous_pulse_durations;
    fir_independent_pulse_durations_ = previous_independent_durations;
    fir_equal_effect_max_duration_sampling_ = previous_equal_effect_sampling;
    fir_cutoff_frequency_hz_ = previous_cutoff_frequency;
    fir_filter_specification_ = previous_filter_specification;
    fir_coefficients_generated_ = previous_coefficients_generated;
    fir_coefficients_ = previous_coefficients;
    fir_stop_coefficient_response_ = previous_stop_response;
    fixed_time_steps_ = previous_time_steps;
    throw;
  }
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
  snapshot.current_state.native_command_velocity =
    applied_native_state_.command_velocity;
  snapshot.current_state.native_command_velocity_valid = true;
  snapshot.current_state.linear_fir_history =
    applied_native_state_.linear_fir_history;
  snapshot.current_state.angular_fir_history =
    applied_native_state_.angular_fir_history;
  snapshot.activation_state.linear_acceleration =
    snapshot.current_state.linear_acceleration;
  snapshot.activation_state.angular_acceleration =
    snapshot.current_state.angular_acceleration;
  snapshot.activation_state.native_command_velocity =
    snapshot.current_state.native_command_velocity;
  snapshot.activation_state.native_command_velocity_valid = true;
  snapshot.activation_state.linear_fir_history =
    snapshot.current_state.linear_fir_history;
  snapshot.activation_state.angular_fir_history =
    snapshot.current_state.angular_fir_history;

  auto pending = pending_native_commands_.cbegin();
  for (const ScheduledCommand & scheduled : snapshot.committed_commands) {
    if (pending == pending_native_commands_.cend() ||
      !command_states_match(
        pending->state.command_velocity, scheduled.command) ||
      pending->is_controller_failure_stop !=
      scheduled.is_controller_failure_stop ||
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
    snapshot.activation_state.native_command_velocity =
      pending->state.command_velocity;
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
  const f_dwa_controller::msg::CommandDispatch & dispatch,
  const bool safety_reduction,
  const std::size_t skipped_unpublished_commands)
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

  if (skipped_unpublished_commands > pending_native_commands_.size()) {
    applied_command_state_ready_ = false;
    pending_native_commands_.clear();
    latest_applied_command_ = dispatched;
    applied_dispatch_time_ = dispatch_time;
    return;
  }
  pending_native_commands_.erase(
    pending_native_commands_.begin(),
    std::next(
      pending_native_commands_.begin(),
      static_cast<std::ptrdiff_t>(skipped_unpublished_commands)));

  if (!pending_native_commands_.empty() &&
    (command_states_match(
      pending_native_commands_.front().state.command_velocity,
      dispatched) || safety_reduction))
  {
    // The robot-facing delay queue is FIFO. A value elsewhere in this queue
    // is not proof that its native metadata produced the observed dispatch,
    // especially when repeated zero commands are pending.
    const bool is_controller_failure_stop =
      pending_native_commands_.front().is_controller_failure_stop;
    applied_native_state_ = pending_native_commands_.front().state;
    pending_native_commands_.pop_front();
    if (safety_reduction || is_controller_failure_stop) {
      // The raw native-input history remains the Controller's internal state,
      // while the robot starts the next cycle from the command that actually
      // passed the actuator-side boundary. Reconstruct observable acceleration
      // from the applied commands and retain FIR history rather than pretending
      // a Controller Server failure zero was generated by a zero-filled FIR.
      if (dispatch_time <= applied_dispatch_time_) {
        applied_native_state_.valid = false;
      } else {
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
      }
    }
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
  if (active_candidate_ == nullptr ||
    !active_candidate_->first_command_state.valid)
  {
    return std::nullopt;
  }
  return candidate_command_state(*active_candidate_);
}

std::optional<std::size_t>
NativeInputTrajectoryGenerator::active_candidate_canonical_index() const
{
  if (active_candidate_ == nullptr) {
    return std::nullopt;
  }
  return active_candidate_->canonical_index;
}

std::optional<NativeInputTrajectoryGenerator::ActiveCandidateDiagnostics>
NativeInputTrajectoryGenerator::active_candidate_diagnostics() const
{
  if (active_candidate_ == nullptr) {
    return std::nullopt;
  }
  ActiveCandidateDiagnostics diagnostics;
  diagnostics.canonical_index = active_candidate_->canonical_index;
  diagnostics.uses_recovery = active_candidate_->linear_rollout->uses_recovery ||
    active_candidate_->angular_rollout->uses_recovery;
  diagnostics.uses_equal_effect_max_duration =
    active_candidate_->linear_rollout->uses_equal_effect_max_duration ||
    active_candidate_->angular_rollout->uses_equal_effect_max_duration;
  diagnostics.linear_recovery_input = active_candidate_->linear_rollout->recovery_input;
  diagnostics.angular_recovery_input = active_candidate_->angular_rollout->recovery_input;
  diagnostics.linear_prediction_input_duration =
    active_candidate_->linear_rollout->prediction_input_duration;
  diagnostics.angular_prediction_input_duration =
    active_candidate_->angular_rollout->prediction_input_duration;
  diagnostics.linear_fractional_last_input =
    active_candidate_->linear_rollout->fractional_last_input;
  diagnostics.angular_fractional_last_input =
    active_candidate_->angular_rollout->fractional_last_input;
  diagnostics.linear_zero_input_duration_unbounded =
    active_candidate_->linear_rollout->zero_input_duration_unbounded;
  diagnostics.angular_zero_input_duration_unbounded =
    active_candidate_->angular_rollout->zero_input_duration_unbounded;
  diagnostics.linear_native_input = active_candidate_->linear_native_input;
  diagnostics.angular_native_input = active_candidate_->angular_native_input;
  diagnostics.initial_linear_velocity = iteration_initial_linear_velocity_;
  diagnostics.initial_angular_velocity = iteration_initial_angular_velocity_;
  diagnostics.initial_linear_acceleration =
    iteration_initial_linear_acceleration_;
  diagnostics.initial_angular_acceleration =
    iteration_initial_angular_acceleration_;
  diagnostics.first_command_state = active_candidate_->first_command_state;
  return diagnostics;
}

std::size_t NativeInputTrajectoryGenerator::candidate_count() const noexcept
{
  return candidates_.size();
}

bool NativeInputTrajectoryGenerator::materialize_candidate(
  const std::size_t canonical_index,
  const geometry_msgs::msg::Pose2D & start_pose,
  dwb_msgs::msg::Trajectory2D & trajectory,
  NativeCommandState & first_command_state)
{
  if (canonical_index >= candidates_.size() ||
    candidates_[canonical_index].canonical_index != canonical_index)
  {
    trajectory = dwb_msgs::msg::Trajectory2D();
    first_command_state = NativeCommandState();
    return false;
  }
  const Candidate & candidate = candidates_[canonical_index];
  generate_candidate_trajectory_into(candidate, start_pose, trajectory);
  first_command_state = candidate_command_state(candidate);
  return first_command_state.valid;
}

void NativeInputTrajectoryGenerator::select_command_for_dispatch(
  const std::optional<NativeCommandState> & command_state)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  selected_command_state_ = command_state;
  if (command_state && command_state->valid) {
    previous_selected_velocity_ = command_state->command_velocity;
  }
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
    PendingNativeCommand{issued_at, *selected_command_state_, false});
  selected_command_state_.reset();
}

bool NativeInputTrajectoryGenerator::commit_expected_controller_stop(
  const rclcpp::Time & issued_at,
  const std::size_t skipped_unpublished_commands)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  constexpr std::size_t kMaximumPendingNativeCommands = 256;
  selected_command_state_.reset();
  if (skipped_unpublished_commands > pending_native_commands_.size() ||
    pending_native_commands_.size() >= kMaximumPendingNativeCommands)
  {
    applied_command_state_ready_ = false;
    pending_native_commands_.clear();
    return false;
  }
  pending_native_commands_.erase(
    pending_native_commands_.begin(),
    std::next(
      pending_native_commands_.begin(),
      static_cast<std::ptrdiff_t>(skipped_unpublished_commands)));

  NativeCommandState stopped_state;
  if (!pending_native_commands_.empty()) {
    stopped_state = pending_native_commands_.back().state;
  } else if (applied_command_state_ready_ && applied_native_state_.valid) {
    stopped_state = applied_native_state_;
  } else {
    applied_command_state_ready_ = false;
    pending_native_commands_.clear();
    return false;
  }
  const nav_2d_msgs::msg::Twist2D previous_command =
    stopped_state.command_velocity;
  stopped_state.command_velocity = nav_2d_msgs::msg::Twist2D();
  stopped_state.linear_state.velocity = 0.0;
  stopped_state.linear_state.acceleration =
    -previous_command.x / control_period_;
  stopped_state.angular_state.velocity = 0.0;
  stopped_state.angular_state.acceleration =
    -previous_command.theta / control_period_;
  // A Controller Server failure zero is outside the native-input model. Keep
  // the last observable FIR memory until its actual dispatch is observed;
  // zeroing it here creates a fictitious filter reset and repeated step response.
  stopped_state.valid = true;
  pending_native_commands_.push_back(
    PendingNativeCommand{issued_at, std::move(stopped_state), true});
  return true;
}

bool NativeInputTrajectoryGenerator::commit_observed_controller_stop_before_pending(
  const rclcpp::Time & issued_at)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  constexpr std::size_t kMaximumPendingNativeCommands = 256;
  selected_command_state_.reset();
  if (!applied_command_state_ready_ || !applied_native_state_.valid ||
    pending_native_commands_.size() >= kMaximumPendingNativeCommands)
  {
    applied_command_state_ready_ = false;
    pending_native_commands_.clear();
    return false;
  }

  // The dispatch callback can observe a Controller Server zero after the next
  // plugin result has already been computed and recorded. The transport FIFO
  // still proves that the zero precedes every command currently pending here,
  // so derive it from the last applied state and insert it at the front. Never
  // erase those later commands: they will receive the following sequence IDs.
  NativeCommandState stopped_state = applied_native_state_;
  const nav_2d_msgs::msg::Twist2D previous_command = latest_applied_command_;
  stopped_state.command_velocity = nav_2d_msgs::msg::Twist2D();
  stopped_state.linear_state.velocity = 0.0;
  stopped_state.linear_state.acceleration =
    -previous_command.x / control_period_;
  stopped_state.angular_state.velocity = 0.0;
  stopped_state.angular_state.acceleration =
    -previous_command.theta / control_period_;
  // Preserve the FIR raw-input memory. observe_command_dispatch() replaces
  // velocity and acceleration with the values measured at the actual dispatch
  // timestamp without inventing a filter reset.
  stopped_state.valid = true;
  pending_native_commands_.push_front(
    PendingNativeCommand{issued_at, std::move(stopped_state), true});
  return true;
}

bool NativeInputTrajectoryGenerator::observe_terminal_controller_stop(
  const f_dwa_controller::msg::CommandDispatch & dispatch)
{
  std::lock_guard<std::mutex> lock(applied_command_mutex_);
  nav_2d_msgs::msg::Twist2D stopped_command;
  stopped_command.x = dispatch.command.linear.x;
  stopped_command.y = dispatch.command.linear.y;
  stopped_command.theta = dispatch.command.angular.z;
  const rclcpp::Time dispatch_time(dispatch.header.stamp);
  const bool stopped_command_is_captured =
    is_captured_stop(stopped_command, stop_capture_velocity_);
  const bool latest_command_is_captured =
    is_captured_stop(latest_applied_command_, stop_capture_velocity_);
  const bool dispatch_time_is_newer =
    applied_dispatch_time_.nanoseconds() == 0 ||
    dispatch_time > applied_dispatch_time_;
  if (!dispatch.has_sequence || !stopped_command_is_captured ||
    !latest_command_is_captured || !dispatch_time_is_newer)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(plugin_name_),
      "Terminal Controller stop did not close native state: "
      "has_sequence=%s stopped_captured=%s latest_captured=%s "
      "dispatch_time_ns=%" PRId64 " applied_time_ns=%" PRId64 " "
      "stopped=(%.17g,%.17g,%.17g) latest=(%.17g,%.17g,%.17g) "
      "capture=%.17g",
      dispatch.has_sequence ? "true" : "false",
      stopped_command_is_captured ? "true" : "false",
      latest_command_is_captured ? "true" : "false",
      dispatch_time.nanoseconds(), applied_dispatch_time_.nanoseconds(),
      stopped_command.x, stopped_command.y, stopped_command.theta,
      latest_applied_command_.x, latest_applied_command_.y,
      latest_applied_command_.theta, stop_capture_velocity_);
    return false;
  }

  // A committed terminal stop is an action boundary: Controller Server owns
  // the final exact zero after StoppedGoalChecker accepts the already-captured
  // native output. No queued result can be published after that boundary, and
  // a subsequent action must start from the externally observed zero state.
  pending_native_commands_.clear();
  selected_command_state_.reset();
  previous_selected_velocity_.reset();
  applied_native_state_ = NativeCommandState();
  applied_native_state_.command_velocity = stopped_command;
  if (input_order_ == NativeInputOrder::kFir) {
    applied_native_state_.linear_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
    applied_native_state_.angular_fir_history.assign(
      fir_coefficients_.size() - 1u, 0.0);
  }
  applied_native_state_.valid = true;
  latest_applied_command_ = stopped_command;
  applied_dispatch_time_ = dispatch_time;
  applied_command_state_ready_ = true;
  return true;
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
  std::optional<nav_2d_msgs::msg::Twist2D> preferred_velocity;
  {
    std::lock_guard<std::mutex> lock(applied_command_mutex_);
    if (planning_snapshot_) {
      if (!planning_snapshot_->valid ||
        !planning_snapshot_->activation_state.native_state_valid)
      {
        return;
      }
      initial_velocity =
        planning_snapshot_->activation_state.native_command_velocity_valid ?
        planning_snapshot_->activation_state.native_command_velocity :
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
    if (prefer_previous_selected_candidate_) {
      preferred_velocity = previous_selected_velocity_;
    }
  }

  const AxisState linear_state{
    initial_velocity.x, initial_linear_acceleration};
  const AxisState angular_state{
    initial_velocity.theta, initial_angular_acceleration};
  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  iteration_initial_linear_velocity_ = initial_velocity.x;
  iteration_initial_angular_velocity_ = initial_velocity.theta;
  iteration_initial_linear_acceleration_ = initial_linear_acceleration;
  iteration_initial_angular_acceleration_ = initial_angular_acceleration;
  iteration_initial_linear_fir_history_ = initial_linear_fir_history;
  iteration_initial_angular_fir_history_ = initial_angular_fir_history;
  const int rollout_step_count =
    static_cast<int>(fixed_time_steps_.size());
  // A complete-stop response uses the initial state and first native input,
  // not the nominal pulse length. Reuse only exact input matches, separately
  // per axis and within this immutable planning snapshot.
  std::map<double, std::shared_ptr<AxisStopData>> linear_stop_data;
  std::map<double, std::shared_ptr<AxisStopData>> angular_stop_data;
  std::vector<std::vector<std::shared_ptr<const AxisRollout>>> linear_banks;
  std::vector<std::vector<std::shared_ptr<const AxisRollout>>> angular_banks;
  bool equal_effect_sampling_active = false;
  if (input_order_ == NativeInputOrder::kFir &&
    fir_equal_effect_max_duration_sampling_)
  {
    const EqualEffectFirContext linear_context =
      prepare_equal_effect_fir_context(
      linear_state, linear_axis_limits, fir_coefficients_,
      initial_linear_fir_history, control_period_, rollout_step_count);
    const EqualEffectFirContext angular_context =
      prepare_equal_effect_fir_context(
      angular_state, angular_axis_limits, fir_coefficients_,
      initial_angular_fir_history, control_period_, rollout_step_count);
    const EqualEffectFirSamples linear_samples =
      sample_equal_effect_fir_axis(linear_context, linear_samples_);
    const EqualEffectFirSamples angular_samples =
      sample_equal_effect_fir_axis(angular_context, angular_samples_);
    if (linear_samples.converged && angular_samples.converged) {
      const auto build_equal_effect_rollouts =
        [this, rollout_step_count](
        const std::vector<EqualEffectFirPulse> & pulses,
        const std::vector<double> & initial_fir_history,
        std::map<double, std::shared_ptr<AxisStopData>> & stop_data)
        {
          std::vector<std::shared_ptr<const AxisRollout>> rollouts;
          rollouts.reserve(pulses.size());
          for (const auto & pulse : pulses) {
            if (!pulse.valid) {continue;}
            auto rollout = std::make_shared<AxisRollout>();
            rollout->uses_equal_effect_max_duration = true;
            rollout->prediction_input_steps = pulse.full_input_steps;
            rollout->prediction_input_duration = pulse.equivalent_duration;
            rollout->fractional_last_input = pulse.fractional_last_input;
            rollout->zero_input_duration_unbounded =
              pulse.zero_input_duration_unbounded;
            rollout->states = pulse.states;
            rollout->native_input = pulse.native_input;
            rollout->first_native_input = pulse.first_native_input;
            rollout->first_fir_history = initial_fir_history;
            push_fir_input(
              rollout->first_fir_history, rollout->first_native_input);
            auto & shared_stop = stop_data[rollout->first_native_input];
            if (!shared_stop) {
              shared_stop = std::make_shared<AxisStopData>();
            }
            rollout->stop_data = shared_stop;
            rollout->valid = rollout->states.size() ==
              static_cast<std::size_t>(rollout_step_count);
            if (rollout->valid) {rollouts.push_back(std::move(rollout));}
          }
          return rollouts;
        };
      auto linear_rollouts = build_equal_effect_rollouts(
        linear_samples.pulses, initial_linear_fir_history, linear_stop_data);
      auto angular_rollouts = build_equal_effect_rollouts(
        angular_samples.pulses, initial_angular_fir_history, angular_stop_data);
      equal_effect_sampling_active =
        linear_rollouts.size() == static_cast<std::size_t>(linear_samples_) &&
        angular_rollouts.size() == static_cast<std::size_t>(angular_samples_);
      if (equal_effect_sampling_active) {
        linear_banks.push_back(std::move(linear_rollouts));
        angular_banks.push_back(std::move(angular_rollouts));
      }
    }
    if (!equal_effect_sampling_active) {
      const auto node = node_.lock();
      if (node) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 5000,
          "%s two-stage effect/max-duration sampling failed "
          "(linear: %s; angular: %s); using the configured fixed-duration "
          "F-DWA sampling for this iteration",
          plugin_name_.c_str(), linear_samples.failure_reason.c_str(),
          angular_samples.failure_reason.c_str());
      }
    }
  }
  const bool recovery = native_input_recovery_;
  const auto periods = equal_effect_sampling_active ? std::vector<int>{} :
  recovery ? std::vector<int>{1} : prediction_input_step_counts();
  for (const int active_input_steps : periods) {
    ZeroFirResponse linear_recovery_response, angular_recovery_response;
    std::vector<FeasibleInterval> linear_recovery_intervals, angular_recovery_intervals;
    HeldFirAffineResponse linear_fir_response;
    HeldFirAffineResponse angular_fir_response;
    FeasibleInterval linear_interval;
    FeasibleInterval angular_interval;
    if (recovery) {
      if (input_order_ == NativeInputOrder::kAcceleration) {
        linear_recovery_intervals = {acceleration_input_interval(
          linear_state, linear_axis_limits, control_period_)};
        angular_recovery_intervals = {acceleration_input_interval(
          angular_state, angular_axis_limits, control_period_)};
      } else if (input_order_ == NativeInputOrder::kJerk) {
        linear_recovery_intervals = jerk_switch_input_intervals(
          linear_state, linear_axis_limits, control_period_, rollout_step_count);
        angular_recovery_intervals = jerk_switch_input_intervals(
          angular_state, angular_axis_limits, control_period_, rollout_step_count);
      } else {
        linear_recovery_response = prepare_zero_fir_response(linear_state, linear_axis_limits,
          fir_coefficients_, initial_linear_fir_history, control_period_, rollout_step_count);
        angular_recovery_response = prepare_zero_fir_response(angular_state, angular_axis_limits,
          fir_coefficients_, initial_angular_fir_history, control_period_, rollout_step_count);
        linear_recovery_intervals = linear_recovery_response.input_intervals;
        angular_recovery_intervals = angular_recovery_response.input_intervals;
      }
    } else if (input_order_ == NativeInputOrder::kFir) {
      // A finite pulse explicitly specifies zero raw input after its last
      // tick. Constrain the complete FIR tail, while retaining only the
      // nominal horizon for poses and critic scoring. A zero duration means
      // the legacy held-input action, whose post-horizon input is unspecified.
      const auto is_configured_fir_pulse = [this, active_input_steps, rollout_step_count](
        const double duration)
        {
          if (duration <= 0.0) {return false;}
          const int steps = std::min(rollout_step_count, std::max(
            1, static_cast<int>(std::ceil(duration / control_period_ - 1.0e-12))));
          return steps == active_input_steps;
        };
      const bool check_fir_tail = is_configured_fir_pulse(fir_prediction_pulse_duration_) ||
        std::any_of(fir_prediction_pulse_durations_.begin(),
        fir_prediction_pulse_durations_.end(), is_configured_fir_pulse);
      const int constraint_step_count = check_fir_tail ? std::max(
        rollout_step_count, active_input_steps + static_cast<int>(fir_coefficients_.size()) - 1) :
        rollout_step_count;
      linear_fir_response = prepare_pulsed_fir_affine_response(
        linear_state, linear_axis_limits, fir_coefficients_,
        initial_linear_fir_history, control_period_, constraint_step_count,
        active_input_steps);
      angular_fir_response = prepare_pulsed_fir_affine_response(
        angular_state, angular_axis_limits, fir_coefficients_,
        initial_angular_fir_history, control_period_, constraint_step_count,
        active_input_steps);
      linear_interval = linear_fir_response.input_interval;
      angular_interval = angular_fir_response.input_interval;
    } else if (input_order_ == NativeInputOrder::kJerk && active_input_steps < rollout_step_count) {
      linear_interval = pulsed_jerk_input_interval(linear_state, linear_axis_limits,
        control_period_, rollout_step_count, active_input_steps);
      angular_interval = pulsed_jerk_input_interval(angular_state, angular_axis_limits,
        control_period_, rollout_step_count, active_input_steps);
    } else {
      // A/J apply one constant native input over the complete nominal horizon.
      // Sample that exact feasible interval directly. Sampling the wider
      // one-step interval and clamping each rollout to the horizon interval made
      // many of the configured 11 x 15 samples collapse to identical rollouts.
      linear_interval =
        held_input_interval(
        linear_state, linear_axis_limits, control_period_,
        rollout_step_count);
      angular_interval =
        held_input_interval(
        angular_state, angular_axis_limits, control_period_,
        rollout_step_count);
    }
    const std::vector<double> linear_inputs =
      recovery ? sample_input_intervals(linear_recovery_intervals, linear_samples_) :
      uniform_samples(linear_interval, linear_samples_);
    const std::vector<double> angular_inputs =
      recovery ? sample_input_intervals(angular_recovery_intervals, angular_samples_) :
      uniform_samples(angular_interval, angular_samples_);
    const auto build_axis_rollouts =
      [this, rollout_step_count, active_input_steps, recovery](
      const std::vector<double> & inputs,
      const AxisState & initial_state,
      const AxisLimits & limits,
      const std::vector<double> & initial_fir_history,
      const HeldFirAffineResponse * fir_response,
      const ZeroFirResponse & recovery_response,
      std::map<double, std::shared_ptr<AxisStopData>> & stop_data)
      {
        std::vector<std::shared_ptr<const AxisRollout>> rollouts;
        rollouts.reserve(inputs.size());
        for (const double input : inputs) {
          auto rollout = std::make_shared<AxisRollout>();
          rollout->prediction_input_steps = active_input_steps;
          rollout->prediction_input_duration =
            active_input_steps * control_period_;
          rollout->native_input = input;
          rollout->first_native_input = input;
          const bool finite_jerk_pulse =
            input_order_ == NativeInputOrder::kJerk &&
            active_input_steps < rollout_step_count;
          if (recovery) {
            auto recovered = input_order_ == NativeInputOrder::kAcceleration ?
              longest_acceleration_input(initial_state, limits, input, control_period_,
              rollout_step_count) :
              input_order_ == NativeInputOrder::kJerk ?
              longest_jerk_recovery(initial_state, limits, input, control_period_,
              rollout_step_count) :
              longest_zero_fir_input(recovery_response, limits, input);
            if (!recovered.valid) {continue;}
            rollout->prediction_input_steps = recovered.active_input_steps;
            rollout->prediction_input_duration =
              recovered.active_input_steps * control_period_;
            rollout->uses_recovery = true;
            rollout->recovery_input = recovered.recovery_input;
            rollout->states = std::move(recovered.states);
            if (input_order_ == NativeInputOrder::kFir) {
              rollout->first_fir_history = initial_fir_history;
              push_fir_input(rollout->first_fir_history, input);
            }
          } else if (input_order_ == NativeInputOrder::kFir) {
            if (fir_response == nullptr ||
              !sample_held_fir_affine_response(
                *fir_response, limits, input, rollout->states))
            {
              continue;
            }
            // The affine response may include the FIR tail solely for the
            // motion-constraint interval. Do not extend critic geometry or
            // the published candidate beyond the nominal prediction horizon.
            rollout->states.resize(static_cast<std::size_t>(rollout_step_count));
            rollout->first_fir_history = initial_fir_history;
            push_fir_input(rollout->first_fir_history, input);
          } else if (finite_jerk_pulse) {
            AxisState state = initial_state;
            for (int k = 0; k < rollout_step_count; ++k) {
              state.acceleration += (k < active_input_steps ? input : 0.0) * control_period_;
              state.velocity += state.acceleration * control_period_;
              if (!recovery_state_valid(state, limits)) {break;}
              rollout->states.push_back(state);
            }
          } else {
            rollout->states.reserve(
              static_cast<std::size_t>(rollout_step_count));
            AxisState state = initial_state;
            for (int step_index = 0;
              step_index < rollout_step_count; ++step_index)
            {
              const ProjectedAxisStep step =
                project_axis(
                state, limits, input, control_period_,
                rollout_step_count - step_index);
              if (!step.feasible) {
                break;
              }
              state = step.state;
              rollout->states.push_back(state);
            }
          }
          rollout->valid =
            rollout->states.size() ==
            static_cast<std::size_t>(rollout_step_count);
          if (rollout->valid) {
            auto & shared_stop = stop_data[input];
            if (!shared_stop) {
              shared_stop = std::make_shared<AxisStopData>();
            }
            rollout->stop_data = shared_stop;
            rollouts.push_back(std::move(rollout));
          }
        }
        return rollouts;
      };
    const HeldFirAffineResponse * linear_fir_response_pointer =
      input_order_ == NativeInputOrder::kFir ?
      &linear_fir_response : nullptr;
    const HeldFirAffineResponse * angular_fir_response_pointer =
      input_order_ == NativeInputOrder::kFir ?
      &angular_fir_response : nullptr;
    const auto linear_rollouts =
      build_axis_rollouts(
      linear_inputs, linear_state, linear_axis_limits,
      initial_linear_fir_history, linear_fir_response_pointer, linear_recovery_response,
        linear_stop_data);
    const auto angular_rollouts =
      build_axis_rollouts(
      angular_inputs, angular_state, angular_axis_limits,
      initial_angular_fir_history, angular_fir_response_pointer, angular_recovery_response,
        angular_stop_data);

    linear_banks.push_back(linear_rollouts);
    angular_banks.push_back(angular_rollouts);
  }
  const auto append_candidates = [this](const auto & linear_rollouts,
    const auto & angular_rollouts) {
      candidates_.reserve(candidates_.size() + linear_rollouts.size() * angular_rollouts.size());
      for (const auto & linear_rollout : linear_rollouts) {
        for (const auto & angular_rollout : angular_rollouts) {
          Candidate candidate;
          candidate.canonical_index = candidates_.size();
          candidate.command_velocity.x =
            linear_rollout->states.front().velocity;
          candidate.command_velocity.y = 0.0;
          candidate.command_velocity.theta =
            angular_rollout->states.front().velocity;
          // For m=0 the first executable input is beta*q, not q. Candidate
          // metadata and delayed-stop history must follow the command that is
          // actually committed this cycle.
          candidate.linear_native_input = linear_rollout->first_native_input;
          candidate.angular_native_input = angular_rollout->first_native_input;
          candidate.first_command_state.command_velocity =
            candidate.command_velocity;
          candidate.first_command_state.linear_state =
            linear_rollout->states.front();
          candidate.first_command_state.angular_state =
            angular_rollout->states.front();
          candidate.first_command_state.valid = true;
          candidate.linear_rollout = linear_rollout;
          candidate.angular_rollout = angular_rollout;
          candidates_.push_back(std::move(candidate));
        }
      }
    };
  // Synchronized pulses stay first, so enabling independent pairing only
  // appends new actions and preserves the former canonical tie order.
  for (std::size_t index = 0u; index < linear_banks.size(); ++index) {
    append_candidates(linear_banks[index], angular_banks[index]);
  }
  if (!equal_effect_sampling_active && !recovery &&
    input_order_ == NativeInputOrder::kFir &&
    fir_independent_pulse_durations_)
  {
    for (std::size_t linear = 0u; linear < linear_banks.size(); ++linear) {
      for (std::size_t angular = 0u; angular < angular_banks.size(); ++angular) {
        if (linear != angular) {
          append_candidates(linear_banks[linear], angular_banks[angular]);
        }
      }
    }
  }
  candidate_order_.resize(candidates_.size());
  std::iota(candidate_order_.begin(), candidate_order_.end(), 0u);
  if (preferred_velocity && !candidates_.empty()) {
    const auto preferred = std::min_element(
      candidate_order_.begin(), candidate_order_.end(),
      [this, &preferred_velocity](
        const std::size_t first, const std::size_t second)
      {
        const auto squared_distance =
        [this, &preferred_velocity](const std::size_t index)
        {
          const auto & velocity = candidates_[index].command_velocity;
          return std::pow(velocity.x - preferred_velocity->x, 2) +
                 std::pow(
              velocity.theta - preferred_velocity->theta, 2);
        };
        return squared_distance(first) < squared_distance(second);
      });
    if (preferred != candidate_order_.end()) {
      std::rotate(
        candidate_order_.begin(), preferred, std::next(preferred));
    }
  }
}

bool NativeInputTrajectoryGenerator::hasMoreTwists()
{
  return candidate_index_ < candidate_order_.size();
}

nav_2d_msgs::msg::Twist2D NativeInputTrajectoryGenerator::nextTwist()
{
  if (!hasMoreTwists()) {
    throw std::out_of_range("No native-input trajectory candidates remain");
  }
  active_candidate_ = &candidates_[candidate_order_[candidate_index_]];
  ++candidate_index_;
  return active_candidate_->command_velocity;
}

dwb_msgs::msg::Trajectory2D
NativeInputTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & /*start_velocity*/,
  const nav_2d_msgs::msg::Twist2D & command_velocity)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  generate_trajectory_into(start_pose, command_velocity, trajectory);
  return trajectory;
}

void NativeInputTrajectoryGenerator::generate_trajectory_into(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & /*command_velocity*/,
  dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (active_candidate_ == nullptr) {
    throw dwb_core::IllegalTrajectoryException(
            "NativeInputDynamics",
            "generateTrajectory called without an active native-input candidate");
  }

  generate_candidate_trajectory_into(
    *active_candidate_, start_pose, trajectory);
}

void NativeInputTrajectoryGenerator::generate_candidate_trajectory_into(
  const Candidate & candidate,
  const geometry_msgs::msg::Pose2D & start_pose,
  dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!candidate.linear_rollout || !candidate.angular_rollout) {
    throw dwb_core::IllegalTrajectoryException(
            "NativeInputDynamics", "candidate rollout is unavailable");
  }

  trajectory.velocity = candidate.command_velocity;
  const std::vector<double> & time_steps = fixed_time_steps_;
  trajectory.poses.clear();
  trajectory.time_offsets.clear();
  trajectory.poses.reserve(
    time_steps.size() + (include_last_point_ ? 2u : 1u));
  trajectory.time_offsets.reserve(
    time_steps.size() + (include_last_point_ ? 1u : 0u));
  trajectory.poses.push_back(start_pose);

  geometry_msgs::msg::Pose2D pose = start_pose;
  double running_time = 0.0;
  if (candidate.linear_rollout->states.size() != time_steps.size() ||
    candidate.angular_rollout->states.size() != time_steps.size())
  {
    throw dwb_core::IllegalTrajectoryException(
            "NativeInputDynamics",
            "precomputed axis rollout does not match the trajectory horizon");
  }
  const auto & angular_pose_steps =
    angular_pose_integration_steps(
    *candidate.angular_rollout, start_pose.theta, time_steps);
  for (std::size_t step_index = 0;
    step_index < time_steps.size(); ++step_index)
  {
    const double time_step = time_steps[step_index];
    const auto & angular_pose_step = angular_pose_steps[step_index];

    const double linear_velocity =
      candidate.linear_rollout->states[step_index].velocity;
    pose.x = pose.x +
      linear_velocity * angular_pose_step.heading_cosine *
      time_step;
    pose.y = pose.y +
      linear_velocity * angular_pose_step.heading_sine *
      time_step;
    pose.theta = angular_pose_step.theta_after_step;
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
}

NativeInputTrajectoryGenerator::NativeCommandState
NativeInputTrajectoryGenerator::candidate_command_state(
  const Candidate & candidate) const
{
  NativeCommandState command_state = candidate.first_command_state;
  if (input_order_ == NativeInputOrder::kFir) {
    command_state.linear_fir_history =
      candidate.linear_rollout->first_fir_history;
    command_state.angular_fir_history =
      candidate.angular_rollout->first_fir_history;
  }
  return command_state;
}

const std::vector<
  NativeInputTrajectoryGenerator::AngularPoseIntegrationStep> &
NativeInputTrajectoryGenerator::angular_pose_integration_steps(
  const AxisRollout & angular_rollout,
  const double start_theta,
  const std::vector<double> & time_steps) const
{
  auto & cache = angular_rollout.pose_integration_cache;
  // Time discretization is required by validate_parameters(), so every
  // candidate in an iteration has the same time-step sequence.
  if (cache.valid &&
    cache.start_theta == start_theta &&
    std::signbit(cache.start_theta) == std::signbit(start_theta) &&
    cache.steps.size() == time_steps.size())
  {
    return cache.steps;
  }

  cache.valid = false;
  cache.steps.clear();
  cache.steps.reserve(time_steps.size());
  double theta = start_theta;
  for (std::size_t step_index = 0;
    step_index < time_steps.size(); ++step_index)
  {
    AngularPoseIntegrationStep step;
    step.heading_cosine = std::cos(theta);
    step.heading_sine = std::sin(theta);
    theta = theta +
      angular_rollout.states[step_index].velocity *
      time_steps[step_index];
    step.theta_after_step = theta;
    cache.steps.push_back(step);
  }
  cache.start_theta = start_theta;
  cache.valid = true;
  return cache.steps;
}

const std::vector<
  NativeInputTrajectoryGenerator::AngularPoseIntegrationStep> &
NativeInputTrajectoryGenerator::angular_stop_pose_integration_steps(
  const AxisRollout & angular_rollout,
  const AxisStopCache & angular_stop_cache,
  const double start_theta,
  const std::size_t stop_step_count) const
{
  auto & cache = angular_rollout.stop_data->pose_integration;
  if (!cache.valid ||
    cache.start_theta != start_theta ||
    std::signbit(cache.start_theta) != std::signbit(start_theta))
  {
    cache.valid = false;
    cache.steps.clear();
    cache.start_theta = start_theta;
  }

  const std::size_t delayed_step_count =
    angular_stop_cache.delayed_states.size();
  const std::size_t required_step_count =
    delayed_step_count + stop_step_count;
  double theta =
    cache.steps.empty() ?
    start_theta : cache.steps.back().theta_after_step;
  cache.steps.reserve(required_step_count);
  for (std::size_t step_index = cache.steps.size();
    step_index < required_step_count; ++step_index)
  {
    double angular_velocity = 0.0;
    if (step_index < delayed_step_count) {
      angular_velocity =
        angular_stop_cache.delayed_states[step_index].velocity;
    } else {
      const std::size_t stop_index =
        step_index - delayed_step_count;
      if (stop_index <
        angular_stop_cache.stop_sequence.states.size())
      {
        angular_velocity =
          angular_stop_cache.stop_sequence.states[stop_index].velocity;
      }
    }
    AngularPoseIntegrationStep step;
    step.heading_cosine = std::cos(theta);
    step.heading_sine = std::sin(theta);
    theta += angular_velocity * control_period_;
    step.theta_after_step = theta;
    cache.steps.push_back(step);
  }
  cache.valid = true;
  return cache.steps;
}

bool NativeInputTrajectoryGenerator::generate_stop_trajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
  std::vector<NativeCommandState> * command_states)
{
  if (active_candidate_ == nullptr) {
    poses.clear();
    velocities.clear();
    if (command_states) {
      command_states->clear();
    }
    return false;
  }
  return generate_candidate_stop_trajectory(
    *active_candidate_, start_pose, maximum_stop_steps,
    stop_velocity_threshold, poses, &velocities, command_states);
}

bool NativeInputTrajectoryGenerator::generate_direct_stop_trajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
  std::vector<NativeCommandState> & command_states) const
{
  poses.clear();
  velocities.clear();
  command_states.clear();
  if (maximum_stop_steps <= 0 ||
    !std::isfinite(stop_velocity_threshold) ||
    stop_velocity_threshold <= 0.0)
  {
    return false;
  }

  const AxisState initial_linear_state{
    iteration_initial_linear_velocity_, iteration_initial_linear_acceleration_};
  const AxisState initial_angular_state{
    iteration_initial_angular_velocity_, iteration_initial_angular_acceleration_};
  StopSequence linear_stop;
  StopSequence angular_stop;
  if (input_order_ == NativeInputOrder::kAcceleration) {
    linear_stop = generate_acceleration_stop_sequence(
      initial_linear_state, linear_limits(), control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_acceleration_stop_sequence(
      initial_angular_state, angular_limits(), control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else if (input_order_ == NativeInputOrder::kJerk) {
    linear_stop = generate_jerk_stop_sequence(
      initial_linear_state, linear_limits(), control_period_,
      maximum_stop_steps, stop_velocity_threshold);
    angular_stop = generate_jerk_stop_sequence(
      initial_angular_state, angular_limits(), control_period_,
      maximum_stop_steps, stop_velocity_threshold);
  } else {
    linear_stop = generate_fir_stop_sequence(
      initial_linear_state, linear_limits(), fir_coefficients_,
      iteration_initial_linear_fir_history_, control_period_,
      maximum_stop_steps, stop_velocity_threshold, true,
      &fir_stop_coefficient_response_, false);
    angular_stop = generate_fir_stop_sequence(
      initial_angular_state, angular_limits(), fir_coefficients_,
      iteration_initial_angular_fir_history_, control_period_,
      maximum_stop_steps, stop_velocity_threshold, true,
      &fir_stop_coefficient_response_, false);
    if ((!angular_stop.feasible ||
      !angular_stop.terminal_state_cleared))
    {
      angular_stop = generate_fir_stop_sequence(
        initial_angular_state, angular_limits(), fir_coefficients_,
        iteration_initial_angular_fir_history_, control_period_,
        maximum_stop_steps, stop_velocity_threshold, true,
        &fir_stop_coefficient_response_, true);
    }
  }
  if (!linear_stop.feasible || !linear_stop.terminal_state_cleared ||
    !angular_stop.feasible || !angular_stop.terminal_state_cleared)
  {
    return false;
  }

  geometry_msgs::msg::Pose2D pose = start_pose;
  poses.push_back(pose);
  const std::size_t stop_step_count =
    std::max(linear_stop.states.size(), angular_stop.states.size());
  poses.reserve(stop_step_count + 2u);
  velocities.reserve(stop_step_count + 1u);
  command_states.reserve(stop_step_count + 1u);
  for (std::size_t step_index = 0u;
    step_index < stop_step_count; ++step_index)
  {
    NativeCommandState state;
    if (step_index < linear_stop.states.size()) {
      state.linear_state = linear_stop.states[step_index];
      state.command_velocity.x = state.linear_state.velocity;
      if (step_index < linear_stop.fir_histories.size()) {
        state.linear_fir_history = linear_stop.fir_histories[step_index];
      }
    }
    if (step_index < angular_stop.states.size()) {
      state.angular_state = angular_stop.states[step_index];
      state.command_velocity.theta = state.angular_state.velocity;
      if (step_index < angular_stop.fir_histories.size()) {
        state.angular_fir_history = angular_stop.fir_histories[step_index];
      }
    }
    if (input_order_ == NativeInputOrder::kFir) {
      if (state.linear_fir_history.empty()) {
        state.linear_fir_history.assign(fir_coefficients_.size() - 1u, 0.0);
      }
      if (state.angular_fir_history.empty()) {
        state.angular_fir_history.assign(fir_coefficients_.size() - 1u, 0.0);
      }
    }
    state.valid = true;
    pose.x += state.command_velocity.x * std::cos(pose.theta) * control_period_;
    pose.y += state.command_velocity.x * std::sin(pose.theta) * control_period_;
    pose.theta += state.command_velocity.theta * control_period_;
    velocities.push_back(state.command_velocity);
    command_states.push_back(std::move(state));
    poses.push_back(pose);
  }

  NativeCommandState zero_state;
  if (input_order_ == NativeInputOrder::kFir) {
    zero_state.linear_fir_history.assign(fir_coefficients_.size() - 1u, 0.0);
    zero_state.angular_fir_history.assign(fir_coefficients_.size() - 1u, 0.0);
  }
  zero_state.valid = true;
  velocities.push_back(zero_state.command_velocity);
  command_states.push_back(std::move(zero_state));
  poses.push_back(pose);
  return true;
}

bool NativeInputTrajectoryGenerator::generate_stop_trajectory_for_candidate(
  const std::size_t canonical_index,
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
  std::vector<NativeCommandState> * command_states)
{
  if (canonical_index >= candidates_.size() ||
    candidates_[canonical_index].canonical_index != canonical_index)
  {
    poses.clear();
    velocities.clear();
    if (command_states) {
      command_states->clear();
    }
    return false;
  }
  return generate_candidate_stop_trajectory(
    candidates_[canonical_index], start_pose, maximum_stop_steps,
    stop_velocity_threshold, poses, &velocities, command_states);
}

bool NativeInputTrajectoryGenerator::generate_stop_poses(
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses)
{
  if (active_candidate_ == nullptr) {
    poses.clear();
    return false;
  }
  return generate_candidate_stop_trajectory(
    *active_candidate_, start_pose, maximum_stop_steps,
    stop_velocity_threshold, poses, nullptr, nullptr);
}

bool NativeInputTrajectoryGenerator::generate_stop_poses_for_candidate(
  const std::size_t canonical_index,
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses)
{
  if (canonical_index >= candidates_.size() ||
    candidates_[canonical_index].canonical_index != canonical_index)
  {
    poses.clear();
    return false;
  }
  return generate_candidate_stop_trajectory(
    candidates_[canonical_index], start_pose, maximum_stop_steps,
    stop_velocity_threshold, poses, nullptr, nullptr);
}

const NativeInputTrajectoryGenerator::AxisStopCache &
NativeInputTrajectoryGenerator::get_axis_stop_cache(
  const AxisRollout & rollout,
  const AxisState & initial_state,
  const AxisLimits & limits,
  const std::vector<double> & initial_fir_history,
  const int rollout_step_count,
  const int command_delay_steps,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  const bool allow_velocity_direction_reversal)
{
  AxisStopCache & cache = rollout.stop_data->sequence;
  if (cache.computed &&
    cache.maximum_stop_steps == maximum_stop_steps &&
    cache.stop_velocity_threshold == stop_velocity_threshold &&
    cache.allow_velocity_direction_reversal ==
    allow_velocity_direction_reversal)
  {
    return cache;
  }

  rollout.stop_data->pose_integration.valid = false;
  rollout.stop_data->pose_integration.steps.clear();
  cache = AxisStopCache();
  cache.computed = true;
  cache.maximum_stop_steps = maximum_stop_steps;
  cache.stop_velocity_threshold = stop_velocity_threshold;
  cache.allow_velocity_direction_reversal =
    allow_velocity_direction_reversal;
  cache.delayed_states.reserve(
    static_cast<std::size_t>(command_delay_steps));

  AxisState state = initial_state;
  std::vector<double> fir_history = initial_fir_history;
  const bool uses_jerk_pulse =
    (rollout.uses_recovery || native_input_pulse_duration_ > 0.0) &&
    input_order_ == NativeInputOrder::kJerk;
  const bool uses_fir_pulse =
    (rollout.uses_recovery || rollout.uses_equal_effect_max_duration) &&
    input_order_ == NativeInputOrder::kFir;
  for (int step_index = 0;
    step_index < command_delay_steps; ++step_index)
  {
    bool feasible = false;
    if (rollout.uses_recovery && input_order_ == NativeInputOrder::kAcceleration) {
      const auto step = project_acceleration_step(state, limits, rollout.native_input,
          control_period_);
      feasible = step.feasible &&
        std::abs(step.applied_native_input - rollout.native_input) <= 1.0e-9;
      state = step.state;
    } else if (uses_jerk_pulse) {
      const auto step = project_jerk_step(state, limits, rollout.native_input, control_period_);
      feasible = step.feasible &&
        std::abs(step.applied_native_input - rollout.native_input) <= 1.0e-9;
      state = step.state;
    } else if (uses_fir_pulse) {
      const double acceleration = fir_acceleration(
        fir_coefficients_, fir_history, rollout.first_native_input);
      state = {state.velocity + acceleration * control_period_, acceleration};
      feasible = recovery_state_valid(state, limits);
      push_fir_input(fir_history, rollout.first_native_input);
    } else if (input_order_ == NativeInputOrder::kFir) {
      feasible = apply_projected_fir_step_in_place(
        state, limits, fir_coefficients_, fir_history,
        rollout.native_input, control_period_);
    } else {
      const ProjectedAxisStep step =
        project_axis(
        state, limits, rollout.native_input, control_period_,
        std::max(1, rollout_step_count - step_index));
      feasible = step.feasible;
      state = step.state;
    }
    if (!feasible) {
      return cache;
    }
    cache.delayed_states.push_back(state);
  }

  if (input_order_ == NativeInputOrder::kAcceleration) {
    cache.stop_sequence = generate_acceleration_stop_sequence(
      state, limits, control_period_, maximum_stop_steps,
      stop_velocity_threshold);
  } else if (input_order_ == NativeInputOrder::kJerk) {
    cache.stop_sequence = generate_jerk_stop_sequence(
      state, limits, control_period_, maximum_stop_steps,
      stop_velocity_threshold);
  } else {
    cache.stop_sequence = generate_fir_stop_sequence(
      state, limits, fir_coefficients_, fir_history, control_period_,
      maximum_stop_steps, stop_velocity_threshold, false,
      &fir_stop_coefficient_response_, false);
    if (allow_velocity_direction_reversal &&
      (!cache.stop_sequence.feasible ||
      !cache.stop_sequence.terminal_state_cleared))
    {
      // Preserve the shortest no-reversal stop when one exists. Angular FIR
      // inertia may make that class empty after a steering sign change; only
      // then allow a bounded zero crossing, whose complete swept footprint is
      // still checked by StopAdmissibility.
      cache.stop_sequence = generate_fir_stop_sequence(
        state, limits, fir_coefficients_, fir_history, control_period_,
        maximum_stop_steps, stop_velocity_threshold, false,
        &fir_stop_coefficient_response_, true);
    }
  }
  cache.valid =
    cache.stop_sequence.feasible &&
    cache.stop_sequence.terminal_state_cleared;
  return cache;
}

bool NativeInputTrajectoryGenerator::generate_candidate_stop_trajectory(
  const Candidate & candidate,
  const geometry_msgs::msg::Pose2D & start_pose,
  const int maximum_stop_steps,
  const double stop_velocity_threshold,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> * velocities,
  std::vector<NativeCommandState> * command_states)
{
  poses.clear();
  if (velocities) {
    velocities->clear();
  }
  if (command_states) {
    command_states->clear();
  }
  if (!candidate.linear_rollout || !candidate.angular_rollout ||
    maximum_stop_steps <= 0 ||
    !std::isfinite(stop_velocity_threshold) ||
    stop_velocity_threshold <= 0.0)
  {
    return false;
  }

  const AxisLimits linear_axis_limits = linear_limits();
  const AxisLimits angular_axis_limits = angular_limits();
  const AxisState initial_linear_state{
    iteration_initial_linear_velocity_,
    iteration_initial_linear_acceleration_};
  const AxisState initial_angular_state{
    iteration_initial_angular_velocity_,
    iteration_initial_angular_acceleration_};
  const int rollout_step_count =
    static_cast<int>(std::ceil(sim_time_ / time_granularity_));
  const int command_delay_steps = std::max(
    1, static_cast<int>(
      std::ceil(stop_command_delay_seconds_ / control_period_)));
  const AxisStopCache & linear_cache =
    get_axis_stop_cache(
    *candidate.linear_rollout, initial_linear_state, linear_axis_limits,
    iteration_initial_linear_fir_history_, rollout_step_count,
    command_delay_steps, maximum_stop_steps, stop_velocity_threshold, false);
  const AxisStopCache & angular_cache =
    get_axis_stop_cache(
    *candidate.angular_rollout, initial_angular_state, angular_axis_limits,
    iteration_initial_angular_fir_history_, rollout_step_count,
    command_delay_steps, maximum_stop_steps, stop_velocity_threshold, true);
  if (!linear_cache.valid || !angular_cache.valid ||
    linear_cache.delayed_states.size() !=
    static_cast<std::size_t>(command_delay_steps) ||
    angular_cache.delayed_states.size() !=
    static_cast<std::size_t>(command_delay_steps))
  {
    return false;
  }
  const StopSequence & linear_stop = linear_cache.stop_sequence;
  const StopSequence & angular_stop = angular_cache.stop_sequence;

  std::vector<double> linear_fir_history;
  std::vector<double> angular_fir_history;
  if (command_states && input_order_ == NativeInputOrder::kFir) {
    linear_fir_history = iteration_initial_linear_fir_history_;
    angular_fir_history = iteration_initial_angular_fir_history_;
  }

  geometry_msgs::msg::Pose2D pose = start_pose;
  const std::size_t stop_step_count =
    std::max(linear_stop.states.size(), angular_stop.states.size());
  const std::size_t trajectory_step_count =
    linear_cache.delayed_states.size() + stop_step_count + 1u;
  const auto & angular_pose_steps =
    angular_stop_pose_integration_steps(
    *candidate.angular_rollout, angular_cache, start_pose.theta,
    stop_step_count);
  if (angular_pose_steps.size() <
    linear_cache.delayed_states.size() + stop_step_count)
  {
    return false;
  }
  poses.reserve(trajectory_step_count + 1u);
  if (velocities) {
    velocities->reserve(trajectory_step_count);
  }
  if (command_states) {
    command_states->reserve(trajectory_step_count);
  }
  poses.push_back(pose);
  std::size_t pose_step_index = 0u;
  for (std::size_t step_index = 0;
    step_index < linear_cache.delayed_states.size(); ++step_index)
  {
    NativeCommandState delayed_state;
    delayed_state.linear_state = linear_cache.delayed_states[step_index];
    delayed_state.angular_state = angular_cache.delayed_states[step_index];
    delayed_state.command_velocity.x =
      delayed_state.linear_state.velocity;
    delayed_state.command_velocity.theta =
      delayed_state.angular_state.velocity;
    if (command_states && input_order_ == NativeInputOrder::kFir) {
      push_fir_input(
        linear_fir_history, candidate.linear_native_input);
      push_fir_input(
        angular_fir_history, candidate.angular_native_input);
      delayed_state.linear_fir_history = linear_fir_history;
      delayed_state.angular_fir_history = angular_fir_history;
    }
    delayed_state.valid = true;
    const AngularPoseIntegrationStep & angular_pose_step =
      angular_pose_steps[pose_step_index];
    pose.x +=
      delayed_state.command_velocity.x *
      angular_pose_step.heading_cosine * control_period_;
    pose.y +=
      delayed_state.command_velocity.x *
      angular_pose_step.heading_sine * control_period_;
    pose.theta = angular_pose_step.theta_after_step;
    ++pose_step_index;
    if (velocities) {
      velocities->push_back(delayed_state.command_velocity);
    }
    poses.push_back(pose);
    if (command_states) {
      command_states->push_back(delayed_state);
    }
  }

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
    const AngularPoseIntegrationStep & angular_pose_step =
      angular_pose_steps[pose_step_index];
    pose.x +=
      stop_velocity.x *
      angular_pose_step.heading_cosine * control_period_;
    pose.y +=
      stop_velocity.x *
      angular_pose_step.heading_sine * control_period_;
    pose.theta = angular_pose_step.theta_after_step;
    ++pose_step_index;
    if (velocities) {
      velocities->push_back(stop_velocity);
    }
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
        if (step_index < linear_stop.native_inputs.size()) {
          push_fir_input(
            linear_fir_history, linear_stop.native_inputs[step_index]);
          stop_state.linear_fir_history = linear_fir_history;
        } else {
          stop_state.linear_fir_history.assign(
            fir_coefficients_.size() - 1u, 0.0);
        }
        if (step_index < angular_stop.native_inputs.size()) {
          push_fir_input(
            angular_fir_history, angular_stop.native_inputs[step_index]);
          stop_state.angular_fir_history = angular_fir_history;
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
  if (velocities) {
    velocities->push_back(zero_velocity);
  }
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

FeasibleInterval NativeInputTrajectoryGenerator::held_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step,
  const int remaining_steps) const
{
  if (input_order_ == NativeInputOrder::kAcceleration) {
    return held_acceleration_input_interval(
      state, limits, time_step, remaining_steps);
  }
  if (input_order_ == NativeInputOrder::kJerk) {
    return held_jerk_input_interval(
      state, limits, time_step, remaining_steps);
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
