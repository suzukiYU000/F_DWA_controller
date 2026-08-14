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

#include "f_dwa_controller/command_envelope_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace f_dwa_controller
{

namespace
{

diagnostic_msgs::msg::KeyValue key_value(
  const std::string & key, const double value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = std::to_string(value);
  return result;
}

bool finite_command(const geometry_msgs::msg::Twist & command)
{
  return std::isfinite(command.linear.x) &&
         std::isfinite(command.linear.y) &&
         std::isfinite(command.linear.z) &&
         std::isfinite(command.angular.x) &&
         std::isfinite(command.angular.y) &&
         std::isfinite(command.angular.z);
}

bool is_zero(const geometry_msgs::msg::Twist & command, const double tolerance)
{
  return std::abs(command.linear.x) <= tolerance &&
         std::abs(command.angular.z) <= tolerance;
}

bool increases_axis_magnitude(
  const double previous, const double next, const double tolerance)
{
  if (std::abs(next) <= std::abs(previous) + tolerance) {
    return false;
  }
  return previous * next >= -tolerance;
}

}  // namespace

CommandEnvelopeNode::CommandEnvelopeNode(const rclcpp::NodeOptions & options)
: Node("command_envelope", options)
{
  const std::string input_topic =
    declare_parameter<std::string>("input_topic", "cmd_vel_nav");
  const std::string output_topic =
    declare_parameter<std::string>("output_topic", "cmd_vel_envelope");
  const std::string diagnostics_topic =
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  maximum_linear_velocity_ =
    declare_parameter<double>("maximum_linear_velocity", 0.6);
  maximum_angular_velocity_ =
    declare_parameter<double>("maximum_angular_velocity", 0.6);
  maximum_linear_acceleration_ =
    declare_parameter<double>("maximum_linear_acceleration", 0.2);
  maximum_angular_acceleration_ =
    declare_parameter<double>("maximum_angular_acceleration", 0.6);
  nominal_control_period_ =
    declare_parameter<double>("nominal_control_period", 0.05);
  maximum_acceleration_interval_ =
    declare_parameter<double>("maximum_acceleration_interval", 0.1);
  velocity_timeout_ = declare_parameter<double>("velocity_timeout", 1.0);
  numeric_tolerance_ = declare_parameter<double>("numeric_tolerance", 1.0e-6);

  const double values[] = {
    maximum_linear_velocity_, maximum_angular_velocity_,
    maximum_linear_acceleration_, maximum_angular_acceleration_,
    nominal_control_period_, maximum_acceleration_interval_, velocity_timeout_,
    numeric_tolerance_};
  if (!std::all_of(
      std::begin(values), std::end(values),
      [](const double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument(
            "command-envelope limits, periods, timeout, and tolerance must be positive");
  }
  if (maximum_acceleration_interval_ < nominal_control_period_) {
    throw std::invalid_argument(
            "maximum_acceleration_interval must not be shorter than nominal_control_period");
  }

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    output_topic, command_qos);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic, command_qos,
    std::bind(&CommandEnvelopeNode::command_callback, this, std::placeholders::_1));
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(25),
    std::bind(&CommandEnvelopeNode::watchdog_callback, this));
  command_publisher_->publish(last_output_);
  RCLCPP_INFO(
    get_logger(),
    "Value-preserving command envelope active: |vx|<=%.3f m/s, |wz|<=%.3f rad/s, "
    "vx acceleration<=%.3f m/s^2, wz acceleration<=%.3f rad/s^2",
    maximum_linear_velocity_, maximum_angular_velocity_,
    maximum_linear_acceleration_, maximum_angular_acceleration_);
}

void CommandEnvelopeNode::command_callback(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  const rclcpp::Time observed_at = now();
  double elapsed_seconds = nominal_control_period_;
  if (input_observed_ && observed_at > last_input_time_) {
    elapsed_seconds = std::clamp(
      (observed_at - last_input_time_).seconds(),
      nominal_control_period_, maximum_acceleration_interval_);
  }
  last_input_time_ = observed_at;
  input_observed_ = true;
  watchdog_zero_published_ = false;

  std::string reason;
  if (!message || !command_is_valid(*message, elapsed_seconds, reason)) {
    publish_zero(reason.empty() ? "null command" : reason, true);
    return;
  }
  command_publisher_->publish(*message);
  last_output_ = *message;
}

bool CommandEnvelopeNode::command_is_valid(
  const geometry_msgs::msg::Twist & command,
  const double elapsed_seconds,
  std::string & reason) const
{
  if (!finite_command(command)) {
    reason = "non-finite command";
    return false;
  }
  if (std::abs(command.linear.y) > numeric_tolerance_ ||
    std::abs(command.linear.z) > numeric_tolerance_ ||
    std::abs(command.angular.x) > numeric_tolerance_ ||
    std::abs(command.angular.y) > numeric_tolerance_)
  {
    reason = "unsupported non-planar command component";
    return false;
  }
  // A zero command is always accepted: stopping must never wait for a ramp.
  if (is_zero(command, numeric_tolerance_)) {
    return true;
  }
  if (command.linear.x < -numeric_tolerance_ ||
    command.linear.x > maximum_linear_velocity_ + numeric_tolerance_)
  {
    reason = "linear velocity exceeds experiment envelope";
    return false;
  }
  if (std::abs(command.angular.z) >
    maximum_angular_velocity_ + numeric_tolerance_)
  {
    reason = "angular velocity exceeds experiment envelope";
    return false;
  }
  const double maximum_linear_step =
    maximum_linear_acceleration_ * elapsed_seconds + numeric_tolerance_;
  if (command.linear.x - last_output_.linear.x > maximum_linear_step) {
    reason = "linear acceleration exceeds experiment envelope";
    return false;
  }
  const double maximum_angular_step =
    maximum_angular_acceleration_ * elapsed_seconds + numeric_tolerance_;
  const bool changes_angular_direction =
    last_output_.angular.z * command.angular.z < -numeric_tolerance_;
  if ((changes_angular_direction ||
    increases_axis_magnitude(
      last_output_.angular.z, command.angular.z, numeric_tolerance_)) &&
    std::abs(command.angular.z - last_output_.angular.z) > maximum_angular_step)
  {
    reason = changes_angular_direction ?
      "angular direction changed faster than the experiment envelope" :
      "angular acceleration exceeds experiment envelope";
    return false;
  }
  return true;
}

void CommandEnvelopeNode::watchdog_callback()
{
  if (!input_observed_ || watchdog_zero_published_) {
    return;
  }
  if ((now() - last_input_time_).seconds() <= velocity_timeout_) {
    return;
  }
  publish_zero("controller command timeout", true);
  watchdog_zero_published_ = true;
}

void CommandEnvelopeNode::publish_zero(
  const std::string & reason, const bool diagnostic_error)
{
  geometry_msgs::msg::Twist zero;
  command_publisher_->publish(zero);
  last_output_ = zero;
  publish_diagnostic(
    diagnostic_error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    reason, zero);
  if (diagnostic_error) {
    RCLCPP_ERROR(get_logger(), "Command rejected; zero published: %s", reason.c_str());
  }
}

void CommandEnvelopeNode::publish_diagnostic(
  const uint8_t level, const std::string & message,
  const geometry_msgs::msg::Twist & command)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "command_envelope";
  status.hardware_id = "software_command_path";
  status.message = message;
  status.values.push_back(key_value("output_linear_x", command.linear.x));
  status.values.push_back(key_value("output_angular_z", command.angular.z));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

}  // namespace f_dwa_controller
