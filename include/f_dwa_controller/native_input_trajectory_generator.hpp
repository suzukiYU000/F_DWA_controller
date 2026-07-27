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

#ifndef F_DWA_CONTROLLER__NATIVE_INPUT_TRAJECTORY_GENERATOR_HPP_
#define F_DWA_CONTROLLER__NATIVE_INPUT_TRAJECTORY_GENERATOR_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f_dwa_controller
{

enum class NativeInputOrder
{
  kAcceleration,
  kJerk
};

class NativeInputTrajectoryGenerator
  : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  explicit NativeInputTrajectoryGenerator(NativeInputOrder input_order);
  ~NativeInputTrajectoryGenerator() override = default;

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name) override;
  void reset() override;
  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;
  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_velocity,
    const nav_2d_msgs::msg::Twist2D & command_velocity) override;
  bool generate_stop_trajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> & velocities);

protected:
  struct Candidate
  {
    nav_2d_msgs::msg::Twist2D command_velocity;
    double linear_native_input{0.0};
    double angular_native_input{0.0};
    double initial_linear_velocity{0.0};
    double initial_angular_velocity{0.0};
    double initial_linear_acceleration{0.0};
    double initial_angular_acceleration{0.0};
  };

  ProjectedAxisStep project_axis(
    const AxisState & state,
    const AxisLimits & limits,
    double native_input_reference,
    double time_step,
    int remaining_steps) const;
  FeasibleInterval input_interval(
    const AxisState & state,
    const AxisLimits & limits,
    double time_step) const;

private:
  void applied_command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message);
  AxisLimits linear_limits() const;
  AxisLimits angular_limits() const;
  void validate_parameters() const;

  NativeInputOrder input_order_;
  std::string plugin_name_;
  double control_period_{0.03};
  double maximum_linear_jerk_{1.57};
  double maximum_angular_jerk_{1.57};
  int linear_samples_{11};
  int angular_samples_{11};
  bool require_applied_command_state_{false};

  std::vector<Candidate> candidates_;
  std::size_t candidate_index_{0};
  Candidate active_candidate_;
  bool has_active_candidate_{false};

  std::mutex applied_command_mutex_;
  geometry_msgs::msg::Twist latest_applied_command_;
  double applied_linear_acceleration_{0.0};
  double applied_angular_acceleration_{0.0};
  bool applied_command_state_ready_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    applied_command_subscriber_;
};

class AccelerationTrajectoryGenerator
  : public NativeInputTrajectoryGenerator
{
public:
  AccelerationTrajectoryGenerator();
};

class JerkTrajectoryGenerator
  : public NativeInputTrajectoryGenerator
{
public:
  JerkTrajectoryGenerator();
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__NATIVE_INPUT_TRAJECTORY_GENERATOR_HPP_
