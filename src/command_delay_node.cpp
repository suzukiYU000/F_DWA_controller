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

#include "f_dwa_controller/command_delay_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr double kMaximumPublishFrequencyHz = 33.333333333333333;

uint64_t steady_time_nanoseconds(
  const std::chrono::steady_clock::time_point time_point)
{
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
    time_point.time_since_epoch()).count();
  return count > 0 ? static_cast<uint64_t>(count) : 0u;
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

std::string command_to_string(const geometry_msgs::msg::Twist & command)
{
  std::ostringstream stream;
  stream << "linear=[" << command.linear.x << "," << command.linear.y << "," <<
    command.linear.z << "],angular=[" << command.angular.x << "," <<
    command.angular.y << "," << command.angular.z << "]";
  return stream.str();
}

bool command_is_finite(const geometry_msgs::msg::Twist & command)
{
  return
    std::isfinite(command.linear.x) &&
    std::isfinite(command.linear.y) &&
    std::isfinite(command.linear.z) &&
    std::isfinite(command.angular.x) &&
    std::isfinite(command.angular.y) &&
    std::isfinite(command.angular.z);
}

}  // namespace

CommandDelayNode::CommandDelayNode(const rclcpp::NodeOptions & options)
: Node("command_delay_transport", options)
{
  const std::string input_topic =
    declare_parameter<std::string>("input_topic", "cmd_vel_nav");
  const std::string output_topic =
    declare_parameter<std::string>("output_topic", "/whill/controller/cmd_vel");
  const std::string applied_topic =
    declare_parameter<std::string>("applied_topic", "/controller/applied_cmd_vel");
  const std::string dispatch_topic =
    declare_parameter<std::string>(
    "dispatch_topic", "/controller/command_dispatch");
  const std::string transport_valid_topic =
    declare_parameter<std::string>(
    "transport_valid_topic", "/dwa_experiment/transport_valid");
  const std::string transport_stopped_topic =
    declare_parameter<std::string>(
    "transport_stopped_topic", "/dwa_experiment/transport_stopped");
  const std::string diagnostics_topic =
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  const std::string reset_trial_service_name =
    declare_parameter<std::string>("reset_trial_service_name", "~/reset_trial_state");
  const std::string invalidate_trial_service_name =
    declare_parameter<std::string>(
    "invalidate_trial_service_name", "~/invalidate_trial");
  const std::string emergency_stop_service_name =
    declare_parameter<std::string>(
    "emergency_stop_service_name", "~/emergency_stop");
  const double publish_frequency_hz =
    declare_parameter<double>("publish_frequency_hz", kMaximumPublishFrequencyHz);
  stopped_velocity_threshold_ =
    declare_parameter<double>("stopped_velocity_threshold", 0.01);
  const double minimum_input_interval_ms =
    declare_parameter<double>("minimum_input_interval_ms", 0.0);
  velocity_response_model_enabled_ =
    declare_parameter<bool>("enable_velocity_response_model", false);
  linear_velocity_response_model_.dead_time_seconds =
    declare_parameter<double>(
    "linear_velocity_response_dead_time_seconds", 0.035);
  linear_velocity_response_model_.time_constant_seconds =
    declare_parameter<double>(
    "linear_velocity_response_time_constant_seconds", 0.02);
  linear_velocity_response_model_.steady_state_gain =
    declare_parameter<double>("linear_velocity_response_gain", 1.0);
  angular_velocity_response_model_.dead_time_seconds =
    declare_parameter<double>(
    "angular_velocity_response_dead_time_seconds", 0.015);
  angular_velocity_response_model_.time_constant_seconds =
    declare_parameter<double>(
    "angular_velocity_response_time_constant_seconds", 0.085);
  angular_velocity_response_model_.steady_state_gain =
    declare_parameter<double>("angular_velocity_response_gain", 0.95);

  CommandDelayParameters delay_parameters;
  delay_parameters.min_delay_ms = declare_parameter<double>("min_delay_ms", 5.0);
  delay_parameters.max_delay_ms = declare_parameter<double>("max_delay_ms", 35.0);
  delay_parameters.mean_delay_ms = declare_parameter<double>("mean_delay_ms", 20.0);
  delay_parameters.delay_stddev_ms =
    declare_parameter<double>("delay_stddev_ms", 5.0);
  const double command_zero_threshold =
    declare_parameter<double>("command_zero_threshold", 0.0);
  const double legacy_zero_threshold =
    declare_parameter<double>("zero_threshold", -1.0);
  delay_parameters.zero_threshold =
    legacy_zero_threshold >= 0.0 ?
    legacy_zero_threshold : command_zero_threshold;
  if (legacy_zero_threshold >= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "zero_threshold is deprecated; use command_zero_threshold");
  }
  const int64_t max_queue_depth = declare_parameter<int64_t>("max_queue_depth", 24);
  const int64_t random_seed = declare_parameter<int64_t>("random_seed", 0);

  if (!std::isfinite(publish_frequency_hz) || publish_frequency_hz <= 0.0 ||
    publish_frequency_hz > kMaximumPublishFrequencyHz)
  {
    throw std::invalid_argument(
            "publish_frequency_hz must be in (0, 33.333333333333333]");
  }
  if (max_queue_depth <= 0) {
    throw std::invalid_argument("max_queue_depth must be positive");
  }
  if (!std::isfinite(stopped_velocity_threshold_) ||
    stopped_velocity_threshold_ < 0.0)
  {
    throw std::invalid_argument(
            "stopped_velocity_threshold must be finite and non-negative");
  }
  if (!std::isfinite(minimum_input_interval_ms) ||
    minimum_input_interval_ms < 0.0)
  {
    throw std::invalid_argument(
            "minimum_input_interval_ms must be finite and non-negative");
  }
  if (!valid_velocity_response_model(linear_velocity_response_model_) ||
    !valid_velocity_response_model(angular_velocity_response_model_))
  {
    throw std::invalid_argument(
            "velocity response model parameters are invalid");
  }
  minimum_input_interval_seconds_ = minimum_input_interval_ms * 1.0e-3;
  if (random_seed < 0) {
    throw std::invalid_argument("random_seed must be non-negative");
  }
  delay_parameters.max_queue_depth = static_cast<std::size_t>(max_queue_depth);
  delay_parameters.random_seed = static_cast<uint64_t>(random_seed);
  delay_queue_ = std::make_unique<CommandDelayQueue>(delay_parameters);

  const auto command_qos = rclcpp::QoS(
    rclcpp::KeepLast(static_cast<std::size_t>(max_queue_depth + 1))).reliable();
  command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic,
    command_qos,
    std::bind(&CommandDelayNode::command_callback, this, std::placeholders::_1));
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 1);
  applied_command_publisher_ =
    create_publisher<geometry_msgs::msg::Twist>(applied_topic, 1);
  command_dispatch_publisher_ =
    create_publisher<f_dwa_controller::msg::CommandDispatch>(
    dispatch_topic,
    // Late-joining planners need the latest observable dispatch state, not
    // stale events from before the current rosbag trial reset.
    rclcpp::QoS(1).reliable().transient_local());
  transport_valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
    transport_valid_topic,
    rclcpp::QoS(1).reliable().transient_local());
  transport_stopped_publisher_ = create_publisher<std_msgs::msg::Bool>(
    transport_stopped_topic,
    rclcpp::QoS(1).reliable().transient_local());
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

  const auto publish_period = std::chrono::nanoseconds(
    static_cast<int64_t>(std::llround(1.0e9 / publish_frequency_hz)));
  publish_period_nanoseconds_ = publish_period.count();
  publish_timer_ = create_wall_timer(
    publish_period,
    std::bind(&CommandDelayNode::timer_callback, this));
  reset_trial_service_ = create_service<std_srvs::srv::Trigger>(
    reset_trial_service_name,
    std::bind(
      &CommandDelayNode::reset_trial_callback,
      this, std::placeholders::_1, std::placeholders::_2));
  invalidate_trial_service_ = create_service<std_srvs::srv::Trigger>(
    invalidate_trial_service_name,
    std::bind(
      &CommandDelayNode::invalidate_trial_callback,
      this, std::placeholders::_1, std::placeholders::_2));
  emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
    emergency_stop_service_name,
    std::bind(
      &CommandDelayNode::emergency_stop_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  publish_transport_valid(true);
  geometry_msgs::msg::Twist zero_command;
  const uint64_t initial_steady_time_ns =
    steady_time_nanoseconds(std::chrono::steady_clock::now());
  const rclcpp::Time initial_time = now();
  last_observed_time_ = initial_time;
  has_observed_time_ = true;
  reset_velocity_response_locked(initial_time);
  command_publisher_->publish(zero_command);
  applied_command_publisher_->publish(zero_command);
  f_dwa_controller::msg::CommandDispatch initial_dispatch;
  initial_dispatch.header.stamp = initial_time;
  initial_dispatch.received_at = initial_time;
  initial_dispatch.received_steady_time_ns = initial_steady_time_ns;
  initial_dispatch.command = zero_command;
  initial_dispatch.has_sequence = false;
  command_dispatch_publisher_->publish(initial_dispatch);
  publish_transport_stopped(true, true);
  RCLCPP_INFO(
    get_logger(),
    "Command delay transport: %.6f Hz, delay=[%.3f, %.3f] ms, mean=%.3f ms, "
    "stddev=%.3f ms, input_interval>=%.3f ms, queue_depth=%" PRId64
    ", seed=%" PRId64 ", velocity_response=%s, "
    "linear_response=(L=%.3f s,T=%.3f s,K=%.3f), "
    "angular_response=(L=%.3f s,T=%.3f s,K=%.3f)",
    publish_frequency_hz,
    delay_parameters.min_delay_ms,
    delay_parameters.max_delay_ms,
    delay_parameters.mean_delay_ms,
    delay_parameters.delay_stddev_ms,
    minimum_input_interval_ms,
    max_queue_depth,
    random_seed,
    velocity_response_model_enabled_ ? "enabled" : "disabled",
    linear_velocity_response_model_.dead_time_seconds,
    linear_velocity_response_model_.time_constant_seconds,
    linear_velocity_response_model_.steady_state_gain,
    angular_velocity_response_model_.dead_time_seconds,
    angular_velocity_response_model_.time_constant_seconds,
    angular_velocity_response_model_.steady_state_gain);
}

void CommandDelayNode::command_callback(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  const uint64_t received_steady_time_ns =
    steady_time_nanoseconds(std::chrono::steady_clock::now());
  const rclcpp::Time received_at = now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reset_publication_pending_ || !transport_valid_) {
      return;
    }
    if (!observe_time_locked(received_at)) {
      invalidate_transport(received_at, "ROS time moved backwards");
      return;
    }
    if (!command_is_finite(*message)) {
      invalidate_transport(received_at, "received a non-finite command");
      return;
    }
    if (has_command_received_steady_time_) {
      const double input_interval_seconds =
        static_cast<double>(
        received_steady_time_ns - last_command_received_steady_time_ns_) * 1.0e-9;
      if (input_interval_seconds < minimum_input_interval_seconds_) {
        std::ostringstream reason;
        reason << "command input interval " <<
          input_interval_seconds * 1.0e3 <<
          " ms is below the configured minimum " <<
          minimum_input_interval_seconds_ * 1.0e3 << " ms";
        invalidate_transport(received_at, reason.str());
        return;
      }
    }
    last_command_received_time_ = received_at;
    has_command_received_time_ = true;
    last_command_received_steady_time_ns_ = received_steady_time_ns;
    has_command_received_steady_time_ = true;

    if (!delay_queue_->enqueue(
        *message, received_at, received_steady_time_ns))
    {
      invalidate_transport(received_at, "command queue overflow");
      return;
    }
    // Serialize the non-empty state with trial reset. Publishing after
    // releasing the mutex could let an old callback overwrite the reset
    // boundary's stopped=true acknowledgement.
    publish_transport_stopped(false);
  }
}

void CommandDelayNode::timer_callback()
{
  geometry_msgs::msg::Twist command_to_publish;
  geometry_msgs::msg::Twist target_to_publish;
  geometry_msgs::msg::Twist dispatched_command;
  const auto callback_started_at = std::chrono::steady_clock::now();
  const uint64_t callback_started_steady_time_ns =
    steady_time_nanoseconds(callback_started_at);
  const rclcpp::Time callback_time = now();
  rclcpp::Time dispatch_time(0, 0, callback_time.get_clock_type());
  rclcpp::Time command_received_at(0, 0, callback_time.get_clock_type());
  uint64_t command_received_steady_time_ns = 0u;
  uint64_t dispatch_steady_time_ns = 0u;
  bool command_changed = false;
  bool reset_applied = false;
  bool transport_stopped = false;
  uint64_t applied_sequence = 0;
  bool has_applied_sequence = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool reset_can_be_applied =
      reset_publication_pending_ &&
      callback_started_at >= pending_reset_requested_at_;
    if (!reset_can_be_applied && transport_valid_ &&
      !observe_time_locked(callback_time))
    {
      invalidate_transport(callback_time, "ROS time moved backwards");
    }
    if (has_robot_publish_steady_time_ &&
      callback_started_steady_time_ns >= last_robot_publish_steady_time_ns_ &&
      callback_started_steady_time_ns - last_robot_publish_steady_time_ns_ <
      static_cast<uint64_t>(publish_period_nanoseconds_))
    {
      // A reset request or executor scheduling race can make a callback ready
      // before one complete robot-facing period has elapsed. It must neither
      // consume the FIFO nor publish early.
      return;
    }
    if (reset_can_be_applied) {
      // Reset the complete transport epoch only on this Timer-owned
      // robot-facing boundary. The service response means "scheduled", not
      // that output state has already changed.
      delay_queue_->reset(pending_reset_seed_);
      reset_velocity_response_locked(callback_time);
      last_applied_sequence_ = 0;
      has_applied_sequence_ = false;
      transport_valid_ = true;
      emergency_stop_active_ = false;
      last_observed_time_ = callback_time;
      has_observed_time_ = true;
      last_command_received_time_ =
        rclcpp::Time(0, 0, callback_time.get_clock_type());
      has_command_received_time_ = false;
      last_command_received_steady_time_ns_ = 0u;
      has_command_received_steady_time_ = false;
      reset_publication_pending_ = false;
      reset_applied = true;
    } else if (transport_valid_) {
      advance_velocity_response_locked(callback_time);
      const auto due_command = delay_queue_->pop_due_steady(
        callback_started_steady_time_ns);
      if (due_command.has_value()) {
        last_dispatched_command_ = due_command->command;
        if (velocity_response_model_enabled_) {
          schedule_velocity_response_target_locked(
            last_dispatched_command_, callback_time);
          // Activate zero-dead-time events at this dispatch boundary without
          // adding an unintended extra Timer period.
          advance_velocity_response_locked(callback_time);
        } else {
          last_applied_command_ = last_dispatched_command_;
        }
        last_applied_sequence_ = due_command->sequence;
        has_applied_sequence_ = true;
        command_received_at = due_command->received_at;
        command_received_steady_time_ns =
          due_command->received_steady_time_ns;
        dispatched_command = due_command->command;
        command_changed = true;
      }
    } else {
      last_dispatched_command_ = geometry_msgs::msg::Twist();
      last_applied_command_ = geometry_msgs::msg::Twist();
    }
    command_to_publish = last_applied_command_;
    target_to_publish = last_dispatched_command_;
    transport_stopped = is_transport_stopped_locked();
    applied_sequence = last_applied_sequence_;
    has_applied_sequence = has_applied_sequence_;
    // This is the observable software handoff epoch. Store it before
    // releasing the state lock so a concurrently scheduled reset cannot
    // establish a different pacing boundary.
    dispatch_time = now();
    dispatch_steady_time_ns =
      steady_time_nanoseconds(std::chrono::steady_clock::now());
    last_robot_publish_steady_time_ns_ = dispatch_steady_time_ns;
    has_robot_publish_steady_time_ = true;
  }

  // Re-anchor at the Timer-owned robot handoff epoch before DDS and status
  // work. In accelerated simulation, adding that work to every period makes
  // an equally rated upstream producer slowly fill the bounded FIFO.
  // The strict steady-time handoff gate above still prevents catch-up.
  publish_timer_->reset();

  std::lock_guard<std::mutex> publication_lock(publish_mutex_);
  {
    // The emergency service can invalidate the transport after this Timer
    // selected a command but before DDS publication. Recheck under the
    // publication boundary so a stale non-zero command can never overtake the
    // emergency zero.
    std::lock_guard<std::mutex> lock(mutex_);
    if (emergency_stop_active_) {
      command_to_publish = geometry_msgs::msg::Twist();
      target_to_publish = geometry_msgs::msg::Twist();
      command_changed = false;
      reset_applied = false;
      transport_stopped = true;
    }
  }
  command_publisher_->publish(command_to_publish);
  // This topic keeps the same semantics as real mode: the command handed to
  // the actuator. Gazebo receives the modeled plant response above, while
  // odometry remains the measured response used by Controller Server.
  applied_command_publisher_->publish(target_to_publish);
  if (reset_applied) {
    f_dwa_controller::msg::CommandDispatch reset_dispatch;
    reset_dispatch.header.stamp = dispatch_time;
    reset_dispatch.received_at = dispatch_time;
    reset_dispatch.received_steady_time_ns = dispatch_steady_time_ns;
    reset_dispatch.command = command_to_publish;
    reset_dispatch.has_sequence = false;
    command_dispatch_publisher_->publish(reset_dispatch);
  } else if (command_changed) {
    f_dwa_controller::msg::CommandDispatch dispatch;
    dispatch.header.stamp = dispatch_time;
    dispatch.received_at = command_received_at;
    dispatch.received_steady_time_ns = command_received_steady_time_ns;
    dispatch.command = dispatched_command;
    dispatch.sequence_id = applied_sequence;
    dispatch.has_sequence = has_applied_sequence;
    command_dispatch_publisher_->publish(dispatch);
  }

  if (reset_applied) {
    publish_transport_valid(true);
    // Publish a fresh trial boundary even if the previous state was already
    // stopped. Between boundaries the transient-local state is event-driven.
    publish_transport_stopped(true, true);
  } else {
    publish_transport_stopped(transport_stopped);
  }
}

void CommandDelayNode::reset_trial_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const int64_t random_seed = get_parameter("random_seed").as_int();
  if (random_seed < 0) {
    response->success = false;
    response->message = "random_seed must be non-negative";
    return;
  }

  const auto reset_requested_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_reset_seed_ = static_cast<uint64_t>(random_seed);
    pending_reset_requested_at_ = reset_requested_at;
    reset_publication_pending_ = true;
  }
  // The next Timer callback applies the queue/RNG reset and emits the zero,
  // dispatch, valid, and stopped states together at one pacing boundary.
  publish_timer_->reset();

  response->success = true;
  response->message =
    "transport trial reset scheduled for the next command Timer tick with seed " +
    std::to_string(random_seed);
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void CommandDelayNode::invalidate_trial_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const rclcpp::Time detected_at = now();
  std::lock_guard<std::mutex> lock(mutex_);
  reset_publication_pending_ = false;
  if (transport_valid_) {
    invalidate_transport(
      detected_at, "external command-ledger validation failure");
  }
  response->success = true;
  response->message = "transport trial marked invalid";
}

void CommandDelayNode::emergency_stop_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const uint64_t stopped_steady_time_ns =
    steady_time_nanoseconds(std::chrono::steady_clock::now());
  const rclcpp::Time stopped_at = now();
  geometry_msgs::msg::Twist zero_command;
  std::vector<DelayedCommand> discarded_commands;
  std::size_t discarded_count = 0;
  uint64_t next_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    discarded_commands = delay_queue_->snapshot();
    discarded_count = discarded_commands.size();
    next_sequence = delay_queue_->next_sequence();
    delay_queue_->clear();
    reset_publication_pending_ = false;
    emergency_stop_active_ = true;
    transport_valid_ = false;
    reset_velocity_response_locked(stopped_at);
    last_applied_sequence_ = 0;
    has_applied_sequence_ = false;
    last_observed_time_ = stopped_at;
    has_observed_time_ = true;
    last_command_received_time_ =
      rclcpp::Time(0, 0, stopped_at.get_clock_type());
    has_command_received_time_ = false;
    last_command_received_steady_time_ns_ = 0u;
    has_command_received_steady_time_ = false;
    last_robot_publish_steady_time_ns_ = stopped_steady_time_ns;
    has_robot_publish_steady_time_ = true;
  }

  // Emergency stop is the only deliberate bypass of the nominal-delay FIFO.
  // Publish the zero from this service callback so it is not delayed until the
  // next transport Timer tick. The invalid state prevents a queued or newly
  // arriving command from re-enabling motion before an explicit trial reset.
  std::lock_guard<std::mutex> publication_lock(publish_mutex_);
  command_publisher_->publish(zero_command);
  applied_command_publisher_->publish(zero_command);
  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = stopped_at;
  dispatch.received_at = stopped_at;
  dispatch.received_steady_time_ns = stopped_steady_time_ns;
  dispatch.command = zero_command;
  dispatch.has_sequence = false;
  command_dispatch_publisher_->publish(dispatch);
  publish_transport_valid(false);
  publish_transport_stopped(true, true);
  publish_diagnostic(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "emergency_stop: delayed command FIFO discarded and zero dispatched",
    stopped_at,
    discarded_count,
    next_sequence,
    discarded_commands);
  publish_timer_->reset();

  response->success = true;
  response->message =
    "emergency stop applied immediately; discarded " +
    std::to_string(discarded_count) + " delayed command(s)";
  RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
}

bool CommandDelayNode::observe_time_locked(const rclcpp::Time & observed_at)
{
  if (has_observed_time_ && observed_at < last_observed_time_) {
    return false;
  }
  last_observed_time_ = observed_at;
  has_observed_time_ = true;
  return true;
}

void CommandDelayNode::reset_velocity_response_locked(
  const rclcpp::Time & reset_at)
{
  last_dispatched_command_ = geometry_msgs::msg::Twist();
  last_applied_command_ = geometry_msgs::msg::Twist();
  response_state_time_ = reset_at;
  has_response_state_time_ = true;
  active_linear_response_target_ = 0.0;
  active_angular_response_target_ = 0.0;
  pending_linear_response_targets_.clear();
  pending_angular_response_targets_.clear();
}

void CommandDelayNode::schedule_velocity_response_target_locked(
  const geometry_msgs::msg::Twist & target,
  const rclcpp::Time & dispatched_at)
{
  pending_linear_response_targets_.push_back(
    AxisResponseTarget{
      dispatched_at + rclcpp::Duration::from_seconds(
        linear_velocity_response_model_.dead_time_seconds),
      target.linear.x});
  pending_angular_response_targets_.push_back(
    AxisResponseTarget{
      dispatched_at + rclcpp::Duration::from_seconds(
        angular_velocity_response_model_.dead_time_seconds),
      target.angular.z});
}

void CommandDelayNode::advance_velocity_response_locked(
  const rclcpp::Time & update_at)
{
  if (!velocity_response_model_enabled_) {
    last_applied_command_ = last_dispatched_command_;
    response_state_time_ = update_at;
    has_response_state_time_ = true;
    return;
  }
  if (!has_response_state_time_) {
    response_state_time_ = update_at;
    has_response_state_time_ = true;
    return;
  }
  if (update_at < response_state_time_) {
    return;
  }

  const auto advance_axis =
    [&update_at, this](
    double & response,
    double & active_target,
    std::deque<AxisResponseTarget> & pending_targets,
    const AxisVelocityResponseModel & model)
    {
      rclcpp::Time cursor = response_state_time_;
      const auto integrate =
        [&response, &active_target, &model](const double duration_seconds)
        {
          if (duration_seconds <= 0.0) {
            return;
          }
          const double target = model.steady_state_gain * active_target;
          const double retained_fraction = std::exp(
            -duration_seconds / model.time_constant_seconds);
          response = target + (response - target) * retained_fraction;
        };
      while (!pending_targets.empty() &&
        pending_targets.front().activation_time <= update_at)
      {
        const AxisResponseTarget event = pending_targets.front();
        pending_targets.pop_front();
        if (event.activation_time > cursor) {
          integrate((event.activation_time - cursor).seconds());
          cursor = event.activation_time;
        }
        active_target = event.command;
      }
      if (update_at > cursor) {
        integrate((update_at - cursor).seconds());
      }
    };
  advance_axis(
    last_applied_command_.linear.x,
    active_linear_response_target_,
    pending_linear_response_targets_,
    linear_velocity_response_model_);
  advance_axis(
    last_applied_command_.angular.z,
    active_angular_response_target_,
    pending_angular_response_targets_,
    angular_velocity_response_model_);
  last_applied_command_.linear.y = 0.0;
  last_applied_command_.linear.z = 0.0;
  last_applied_command_.angular.x = 0.0;
  last_applied_command_.angular.y = 0.0;
  response_state_time_ = update_at;
}

void CommandDelayNode::invalidate_transport(
  const rclcpp::Time & detected_at,
  const std::string & reason)
{
  const std::size_t queue_depth = delay_queue_->size();
  const uint64_t next_sequence = delay_queue_->next_sequence();
  const std::vector<DelayedCommand> queued_commands =
    delay_queue_->snapshot();
  const std::string last_applied = command_to_string(last_applied_command_);
  std::ostringstream queued_timing;
  for (std::size_t index = 0; index < queued_commands.size(); ++index) {
    const DelayedCommand & queued = queued_commands[index];
    if (index != 0) {
      queued_timing << ";";
    }
    queued_timing << queued.sequence << "@" <<
      queued.received_at.seconds() <<
      "[steady_ns=" << queued.received_steady_time_ns << "]->" <<
      queued.eligible_at.seconds() <<
      "[steady_ns=" << queued.eligible_steady_time_ns << "]";
  }

  transport_valid_ = false;
  delay_queue_->clear();

  RCLCPP_ERROR(
    get_logger(),
    "transport_invalid: %s at %.9f s; queue_depth=%zu; "
    "next_sequence=%" PRIu64 "; last_applied=%s; queued_timing=%s",
    reason.c_str(),
    detected_at.seconds(),
    queue_depth,
    next_sequence,
    last_applied.c_str(),
    queued_timing.str().c_str());
  publish_transport_valid(false);
  publish_transport_stopped(false);
  publish_diagnostic(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "transport_invalid: " + reason,
    detected_at,
    queue_depth,
    next_sequence,
    queued_commands);
  reset_velocity_response_locked(detected_at);
}

void CommandDelayNode::publish_transport_valid(const bool is_valid)
{
  std_msgs::msg::Bool message;
  message.data = is_valid;
  transport_valid_publisher_->publish(message);
}

void CommandDelayNode::publish_transport_stopped(
  const bool is_stopped,
  const bool force)
{
  const int8_t desired_state = is_stopped ? 1 : 0;
  if (!force) {
    int8_t previous_state =
      last_published_transport_stopped_.load(std::memory_order_relaxed);
    while (previous_state != desired_state) {
      if (last_published_transport_stopped_.compare_exchange_weak(
          previous_state,
          desired_state,
          std::memory_order_relaxed))
      {
        break;
      }
    }
    if (previous_state == desired_state) {
      return;
    }
  } else {
    last_published_transport_stopped_.store(
      desired_state,
      std::memory_order_relaxed);
  }
  std_msgs::msg::Bool message;
  message.data = is_stopped;
  transport_stopped_publisher_->publish(message);
}

bool CommandDelayNode::is_transport_stopped_locked() const
{
  return emergency_stop_active_ ||
         (transport_valid_ && delay_queue_->empty() &&
         CommandDelayQueue::is_zero(
    last_dispatched_command_, stopped_velocity_threshold_) &&
         pending_linear_response_targets_.empty() &&
         pending_angular_response_targets_.empty() &&
         CommandDelayQueue::is_zero(
    last_applied_command_, stopped_velocity_threshold_));
}

void CommandDelayNode::publish_diagnostic(
  const uint8_t level,
  const std::string & message,
  const rclcpp::Time & stamp,
  const std::size_t queue_depth,
  const uint64_t next_sequence,
  const std::vector<DelayedCommand> & queued_commands)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = get_fully_qualified_name() + std::string(": transport");
  status.hardware_id = "simulation_command_transport";
  status.message = message;
  status.values.push_back(make_key_value("queue_depth", std::to_string(queue_depth)));
  status.values.push_back(make_key_value("next_sequence", std::to_string(next_sequence)));
  status.values.push_back(
    make_key_value(
      "last_dispatched_command", command_to_string(last_dispatched_command_)));
  status.values.push_back(
    make_key_value("last_applied_command", command_to_string(last_applied_command_)));
  status.values.push_back(
    make_key_value(
      "last_applied_sequence",
      has_applied_sequence_ ?
      std::to_string(last_applied_sequence_) : "none"));
  status.values.push_back(
    make_key_value(
      "last_command_received_seconds",
      has_command_received_time_ ?
      std::to_string(last_command_received_time_.seconds()) : "none"));
  status.values.push_back(make_key_value("detected_at_seconds", std::to_string(stamp.seconds())));
  for (std::size_t index = 0; index < queued_commands.size(); ++index) {
    const DelayedCommand & queued = queued_commands[index];
    std::ostringstream value;
    value << "sequence=" << queued.sequence <<
      ",received_at=" << queued.received_at.seconds() <<
      ",received_steady_time_ns=" << queued.received_steady_time_ns <<
      ",eligible_at=" << queued.eligible_at.seconds() <<
      ",eligible_steady_time_ns=" << queued.eligible_steady_time_ns <<
      ",sampled_delay_ms=" << queued.sampled_delay_ms <<
      "," << command_to_string(queued.command);
    status.values.push_back(
      make_key_value("queued_command_" + std::to_string(index), value.str()));
  }
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

}  // namespace f_dwa_controller
