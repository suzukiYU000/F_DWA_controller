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

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/planning_snapshot.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

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
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void reset() override;
  dwb_msgs::msg::TrajectoryScore scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double best_score = -1) override;

protected:
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override;

private:
  struct IssuedCommand
  {
    rclcpp::Time issued_at;
    nav_2d_msgs::msg::Twist2D command;
  };

  void command_dispatch_callback(
    const f_dwa_controller::msg::CommandDispatch::SharedPtr message);
  void transport_valid_callback(
    const std_msgs::msg::Bool::SharedPtr message);
  void request_transport_invalidation(const char * reason);
  std::shared_ptr<const PlanningSnapshot> build_planning_snapshot(
    const geometry_msgs::msg::PoseStamped & pose);
  void record_issued_command(
    const geometry_msgs::msg::TwistStamped & command,
    const rclcpp::Time & issued_at);
  void record_planning_duration(
    std::chrono::steady_clock::time_point started_at);
  void report_planning_metrics(const char * scope);
  bool should_publish_evaluation();
  void publish_evaluation(
    const std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & evaluation);
  void prepare_certified_footprint();
  void score_trajectory_components(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double best_score,
    dwb_msgs::msg::TrajectoryScore & score);
  bool build_stop_trajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> * velocities,
    std::vector<NativeInputTrajectoryGenerator::NativeCommandState> *
    command_states,
    std::optional<std::size_t> native_candidate_index = std::nullopt);
  bool build_revalidated_backup(
    const geometry_msgs::msg::Pose2D & start_pose,
    dwb_msgs::msg::TrajectoryScore & backup_score);
  bool certify_stop_poses(
    const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
    CertificationFailure & failure,
    CertificationResult * result = nullptr) const;
  AxisLimits linear_limits() const;
  AxisLimits angular_limits() const;
  void reset_trial_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::mutex controller_state_mutex_;
  std::mutex command_state_mutex_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<
    dwb_msgs::msg::LocalPlanEvaluation>::SharedPtr evaluation_publisher_;
  rclcpp::Subscription<f_dwa_controller::msg::CommandDispatch>::SharedPtr
    command_dispatch_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    transport_valid_subscriber_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    transport_invalidation_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trial_service_;
  bool certification_enabled_{false};
  bool nominal_delay_preview_enabled_{true};
  bool require_command_dispatch_state_{true};
  bool command_dispatch_observed_{false};
  bool command_transport_valid_{false};
  bool command_ledger_valid_{false};
  bool expected_dispatch_sequence_ready_{false};
  uint64_t expected_dispatch_sequence_{0};
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
  nav_2d_msgs::msg::Twist2D dispatched_command_;
  std::deque<IssuedCommand> pending_issued_commands_;
  std::shared_ptr<const PlanningSnapshot> planning_snapshot_;
  bool planning_metrics_enabled_{true};
  int planning_metrics_report_interval_{1000};
  double planning_deadline_seconds_{0.03};
  bool publish_evaluation_{true};
  double evaluation_publish_frequency_{0.0};
  rclcpp::Time last_evaluation_publish_time_{0, 0, RCL_ROS_TIME};
  bool has_evaluation_publish_time_{false};
  std::vector<double> planning_durations_seconds_;
  uint64_t planning_cycle_count_{0};
  uint64_t planning_deadline_miss_count_{0};
  double maximum_planning_duration_seconds_{0.0};
  std::vector<nav_2d_msgs::msg::Twist2D> retained_backup_commands_;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
  retained_backup_states_;
  std::vector<geometry_msgs::msg::Point> certified_footprint_;
  mutable CertificationWorkspace certification_workspace_;
  std::vector<geometry_msgs::msg::Pose2D> stop_pose_scratch_;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__CERTIFIED_DWB_LOCAL_PLANNER_HPP_
