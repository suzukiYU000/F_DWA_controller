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

#ifndef F_DWA_CONTROLLER__COMMAND_DELAY_NODE_HPP_
#define F_DWA_CONTROLLER__COMMAND_DELAY_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "f_dwa_controller/command_delay_queue.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace f_dwa_controller
{

class CommandDelayNode : public rclcpp::Node
{
public:
  explicit CommandDelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void timer_callback();
  void reset_trial_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void invalidate_trial_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  [[nodiscard]] bool observe_time_locked(const rclcpp::Time & observed_at);
  void invalidate_transport(
    const rclcpp::Time & detected_at,
    const std::string & reason);
  void publish_transport_valid(bool is_valid);
  void publish_transport_stopped(bool is_stopped, bool force = false);
  [[nodiscard]] bool is_transport_stopped_locked() const;
  void publish_diagnostic(
    uint8_t level,
    const std::string & message,
    const rclcpp::Time & stamp,
    std::size_t queue_depth,
    uint64_t next_sequence,
    const std::vector<DelayedCommand> & queued_commands);

  std::mutex mutex_;
  std::unique_ptr<CommandDelayQueue> delay_queue_;
  geometry_msgs::msg::Twist last_applied_command_;
  uint64_t last_applied_sequence_{0};
  bool has_applied_sequence_{false};
  bool transport_valid_{true};
  bool reset_publication_pending_{false};
  uint64_t pending_reset_seed_{0};
  std::chrono::steady_clock::time_point pending_reset_requested_at_;
  double stopped_velocity_threshold_{0.01};
  double minimum_input_interval_seconds_{0.0};
  int64_t publish_period_nanoseconds_{30000000};
  rclcpp::Time last_robot_publish_time_{0, 0, RCL_ROS_TIME};
  bool has_robot_publish_time_{false};
  rclcpp::Time last_observed_time_{0, 0, RCL_ROS_TIME};
  bool has_observed_time_{false};
  rclcpp::Time last_command_received_time_{0, 0, RCL_ROS_TIME};
  bool has_command_received_time_{false};
  std::atomic<int8_t> last_published_transport_stopped_{-1};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_command_publisher_;
  rclcpp::Publisher<f_dwa_controller::msg::CommandDispatch>::SharedPtr
    command_dispatch_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr transport_valid_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr transport_stopped_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trial_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr invalidate_trial_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__COMMAND_DELAY_NODE_HPP_
