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
  const std::string diagnostics_topic =
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  const std::string reset_trial_service_name =
    declare_parameter<std::string>("reset_trial_service_name", "~/reset_trial_state");
  const double publish_frequency_hz =
    declare_parameter<double>("publish_frequency_hz", kMaximumPublishFrequencyHz);

  CommandDelayParameters delay_parameters;
  delay_parameters.min_delay_ms = declare_parameter<double>("min_delay_ms", 60.0);
  delay_parameters.max_delay_ms = declare_parameter<double>("max_delay_ms", 80.0);
  delay_parameters.mean_delay_ms = declare_parameter<double>("mean_delay_ms", 70.0);
  delay_parameters.delay_stddev_ms =
    declare_parameter<double>("delay_stddev_ms", 3.333333333333333);
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
  const int64_t max_queue_depth = declare_parameter<int64_t>("max_queue_depth", 4);
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
    rclcpp::QoS(
      rclcpp::KeepLast(
        static_cast<std::size_t>(max_queue_depth + 1)))
    .reliable().transient_local());
  transport_valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
    transport_valid_topic,
    rclcpp::QoS(1).reliable().transient_local());
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

  const auto publish_period = std::chrono::nanoseconds(
    static_cast<int64_t>(std::llround(1.0e9 / publish_frequency_hz)));
  publish_timer_ = create_timer(
    publish_period,
    std::bind(&CommandDelayNode::timer_callback, this));
  reset_trial_service_ = create_service<std_srvs::srv::Trigger>(
    reset_trial_service_name,
    std::bind(
      &CommandDelayNode::reset_trial_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  publish_transport_valid(true);
  geometry_msgs::msg::Twist zero_command;
  command_publisher_->publish(zero_command);
  applied_command_publisher_->publish(zero_command);
  f_dwa_controller::msg::CommandDispatch initial_dispatch;
  initial_dispatch.header.stamp = now();
  initial_dispatch.command = zero_command;
  initial_dispatch.has_sequence = false;
  command_dispatch_publisher_->publish(initial_dispatch);
  RCLCPP_INFO(
    get_logger(),
    "Command delay transport: %.6f Hz, delay=[%.3f, %.3f] ms, mean=%.3f ms, "
    "stddev=%.3f ms, queue_depth=%" PRId64 ", seed=%" PRId64,
    publish_frequency_hz,
    delay_parameters.min_delay_ms,
    delay_parameters.max_delay_ms,
    delay_parameters.mean_delay_ms,
    delay_parameters.delay_stddev_ms,
    max_queue_depth,
    random_seed);
}

void CommandDelayNode::command_callback(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  const rclcpp::Time received_at = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!transport_valid_) {
    return;
  }

  if (!delay_queue_->enqueue(*message, received_at)) {
    invalidate_transport(received_at);
  }
}

void CommandDelayNode::timer_callback()
{
  geometry_msgs::msg::Twist command_to_publish;
  const rclcpp::Time dispatch_time = now();
  bool command_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transport_valid_) {
      const auto due_command = delay_queue_->pop_due(dispatch_time);
      if (due_command.has_value()) {
        last_applied_command_ = due_command->command;
        last_applied_sequence_ = due_command->sequence;
        has_applied_sequence_ = true;
        command_changed = true;
      }
    } else {
      last_applied_command_ = geometry_msgs::msg::Twist();
    }
    command_to_publish = last_applied_command_;
  }

  command_publisher_->publish(command_to_publish);
  applied_command_publisher_->publish(command_to_publish);
  if (command_changed) {
    f_dwa_controller::msg::CommandDispatch dispatch;
    dispatch.header.stamp = dispatch_time;
    dispatch.command = command_to_publish;
    dispatch.sequence_id = last_applied_sequence_;
    dispatch.has_sequence = has_applied_sequence_;
    command_dispatch_publisher_->publish(dispatch);
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

  geometry_msgs::msg::Twist zero_command;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delay_queue_->reset(static_cast<uint64_t>(random_seed));
    last_applied_command_ = zero_command;
    last_applied_sequence_ = 0;
    has_applied_sequence_ = false;
    transport_valid_ = true;
  }

  command_publisher_->publish(zero_command);
  applied_command_publisher_->publish(zero_command);
  f_dwa_controller::msg::CommandDispatch dispatch;
  dispatch.header.stamp = now();
  dispatch.command = zero_command;
  dispatch.has_sequence = false;
  command_dispatch_publisher_->publish(dispatch);
  publish_transport_valid(true);
  response->success = true;
  response->message =
    "transport trial state reset with seed " + std::to_string(random_seed);
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void CommandDelayNode::invalidate_transport(const rclcpp::Time & detected_at)
{
  const std::size_t queue_depth = delay_queue_->size();
  const uint64_t next_sequence = delay_queue_->next_sequence();
  const std::string last_applied = command_to_string(last_applied_command_);

  transport_valid_ = false;
  delay_queue_->clear();

  RCLCPP_ERROR(
    get_logger(),
    "transport_invalid: queue overflow at %.9f s; queue_depth=%zu; "
    "next_sequence=%" PRIu64 "; last_applied=%s",
    detected_at.seconds(),
    queue_depth,
    next_sequence,
    last_applied.c_str());
  publish_transport_valid(false);
  publish_diagnostic(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "transport_invalid: command queue overflow",
    detected_at,
    queue_depth,
    next_sequence);
  last_applied_command_ = geometry_msgs::msg::Twist();
}

void CommandDelayNode::publish_transport_valid(const bool is_valid)
{
  std_msgs::msg::Bool message;
  message.data = is_valid;
  transport_valid_publisher_->publish(message);
}

void CommandDelayNode::publish_diagnostic(
  const uint8_t level,
  const std::string & message,
  const rclcpp::Time & stamp,
  const std::size_t queue_depth,
  const uint64_t next_sequence)
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
    make_key_value("last_applied_command", command_to_string(last_applied_command_)));
  status.values.push_back(make_key_value("detected_at_seconds", std::to_string(stamp.seconds())));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

}  // namespace f_dwa_controller
