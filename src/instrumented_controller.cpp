// Copyright 2026 YT-Lab
//
// Licensed under the MIT License. See LICENSE in the project root for details.

#include "f_dwa_controller/instrumented_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

namespace f_dwa_controller
{

InstrumentedController::InstrumentedController()
: controller_loader_("nav2_core", "nav2_core::Controller")
{
}

void InstrumentedController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock the controller server node");
  }

  parent_ = parent;
  name_ = std::move(name);
  sequence_ = 0;
  const auto parameter_name = name_ + ".primary_controller_type";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_name, rclcpp::ParameterValue(std::string{}));
  const auto primary_controller_type = node->get_parameter(parameter_name).as_string();
  if (primary_controller_type.empty()) {
    throw std::runtime_error(parameter_name + " must not be empty");
  }
  if (primary_controller_type == "f_dwa_controller::InstrumentedController") {
    throw std::runtime_error(parameter_name + " must not select the wrapper itself");
  }

  primary_controller_ = controller_loader_.createUniqueInstance(primary_controller_type);
  primary_controller_->configure(parent, name_, std::move(tf), std::move(costmap_ros));
  if (primary_controller_type == "dwb_core::DWBLocalPlanner") {
    reset_trial_service_ = node->create_service<std_srvs::srv::Trigger>(
      "~/" + name_ + "/reset_trial_state",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(primary_controller_mutex_);
        primary_controller_->reset();
        response->success = true;
        response->message = "DWB trial state and runtime horizon reloaded";
      });
  }
  publisher_ = node->create_publisher<f_dwa_controller::msg::ControllerComputation>(
    "~/computation_time", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
}

void InstrumentedController::cleanup()
{
  reset_trial_service_.reset();
  if (primary_controller_) {
    primary_controller_->cleanup();
  }
  publisher_.reset();
  primary_controller_.reset();
  parent_.reset();
}

void InstrumentedController::activate()
{
  primary_controller_->activate();
  publisher_->on_activate();
}

void InstrumentedController::deactivate()
{
  publisher_->on_deactivate();
  primary_controller_->deactivate();
}

void InstrumentedController::setPlan(const nav_msgs::msg::Path & path)
{
  primary_controller_->setPlan(path);
}

geometry_msgs::msg::TwistStamped InstrumentedController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(primary_controller_mutex_);
  const auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Controller server node expired");
  }
  const auto start_stamp = node->now();
  const auto start = std::chrono::steady_clock::now();
  try {
    auto command = primary_controller_->computeVelocityCommands(
      pose, velocity, goal_checker);
    const auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count();
    publish_computation(start_stamp, duration_ns, true);
    return command;
  } catch (...) {
    const auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count();
    publish_computation(start_stamp, duration_ns, false);
    throw;
  }
}

bool InstrumentedController::cancel()
{
  return primary_controller_->cancel();
}

void InstrumentedController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

void InstrumentedController::reset()
{
  std::lock_guard<std::mutex> lock(primary_controller_mutex_);
  if (primary_controller_) {
    primary_controller_->reset();
  }
}

void InstrumentedController::publish_computation(
  const rclcpp::Time & start_stamp, int64_t duration_ns, bool success) noexcept
{
  if (!publisher_ || !publisher_->is_activated()) {
    return;
  }

  f_dwa_controller::msg::ControllerComputation message;
  message.header.stamp = start_stamp;
  message.sequence = sequence_++;
  message.controller_id = name_;
  message.duration.sec = static_cast<int32_t>(duration_ns / 1000000000LL);
  message.duration.nanosec = static_cast<uint32_t>(duration_ns % 1000000000LL);
  message.success = success;
  try {
    publisher_->publish(message);
  } catch (...) {
    // Timing diagnostics must never change the controller's control flow.
  }
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::InstrumentedController,
  nav2_core::Controller)
