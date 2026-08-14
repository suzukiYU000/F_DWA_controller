// Copyright 2026 YT-Lab
//
// Licensed under the MIT License. See LICENSE in the project root for details.

#ifndef F_DWA_CONTROLLER__INSTRUMENTED_CONTROLLER_HPP_
#define F_DWA_CONTROLLER__INSTRUMENTED_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "f_dwa_controller/msg/controller_computation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace f_dwa_controller
{

/**
 * @brief Transparent Nav2 Controller wrapper that publishes computation timing.
 *
 * The wrapped plugin is configured with the same plugin ID, so its existing
 * parameters and behavior remain unchanged.
 */
class InstrumentedController : public nav2_core::Controller
{
public:
  InstrumentedController();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  void publish_computation(
    const rclcpp::Time & start_stamp, int64_t duration_ns, bool success) noexcept;

  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  nav2_core::Controller::Ptr primary_controller_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecyclePublisher<
    f_dwa_controller::msg::ControllerComputation>::SharedPtr publisher_;
  std::string name_;
  uint64_t sequence_{0};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__INSTRUMENTED_CONTROLLER_HPP_
