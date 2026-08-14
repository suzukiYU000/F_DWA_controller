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

#ifndef F_DWA_CONTROLLER__COMMAND_ENVELOPE_NODE_HPP_
#define F_DWA_CONTROLLER__COMMAND_ENVELOPE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f_dwa_controller
{

/// Fail-closed, value-preserving safety boundary for controller commands.
class CommandEnvelopeNode : public rclcpp::Node
{
public:
  explicit CommandEnvelopeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void watchdog_callback();
  void publish_zero(const std::string & reason, bool diagnostic_error);
  void publish_diagnostic(
    uint8_t level, const std::string & message,
    const geometry_msgs::msg::Twist & command);
  [[nodiscard]] bool command_is_valid(
    const geometry_msgs::msg::Twist & command,
    double elapsed_seconds,
    std::string & reason) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  geometry_msgs::msg::Twist last_output_;
  rclcpp::Time last_input_time_;
  bool input_observed_{false};
  bool watchdog_zero_published_{false};
  double maximum_linear_velocity_{0.6};
  double maximum_angular_velocity_{0.6};
  double maximum_linear_acceleration_{0.2};
  double maximum_angular_acceleration_{0.6};
  double nominal_control_period_{0.05};
  double maximum_acceleration_interval_{0.1};
  double velocity_timeout_{1.0};
  double numeric_tolerance_{1.0e-6};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__COMMAND_ENVELOPE_NODE_HPP_
