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

#ifndef F_DWA_CONTROLLER__CERTIFIED_DWB_LOCAL_PLANNER_HPP_
#define F_DWA_CONTROLLER__CERTIFIED_DWB_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"

namespace f_dwa_controller
{

class CertifiedDWBLocalPlanner : public dwb_core::DWBLocalPlanner
{
public:
  CertifiedDWBLocalPlanner() = default;
  ~CertifiedDWBLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  dwb_msgs::msg::TrajectoryScore scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double best_score = -1) override;

protected:
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override;

private:
  bool build_stop_trajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> & velocities);
  bool build_revalidated_backup(
    const geometry_msgs::msg::Pose2D & start_pose,
    dwb_msgs::msg::TrajectoryScore & backup_score);
  bool certify_stop_poses(
    const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
    CertificationFailure & failure) const;
  AxisLimits linear_limits() const;
  AxisLimits angular_limits() const;

  bool certification_enabled_{false};
  bool nominal_delay_preview_enabled_{true};
  double nominal_delay_preview_seconds_{0.07};
  double certification_control_period_{0.03};
  double terminal_stop_maximum_time_{8.0};
  double terminal_stop_velocity_threshold_{0.01};
  double minimum_certified_margin_{0.02};
  double maximum_swept_distance_{0.025};
  double minimum_linear_velocity_{0.0};
  double maximum_linear_velocity_{1.2};
  double maximum_angular_velocity_{1.57};
  double maximum_linear_acceleration_{1.2};
  double maximum_linear_deceleration_{-1.2};
  double maximum_angular_acceleration_{1.57};
  double maximum_angular_deceleration_{-1.57};
  std::vector<nav_2d_msgs::msg::Twist2D>
  current_candidate_stop_velocities_;
  std::vector<nav_2d_msgs::msg::Twist2D> retained_backup_commands_;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__CERTIFIED_DWB_LOCAL_PLANNER_HPP_
