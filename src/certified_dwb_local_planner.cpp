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

#include "f_dwa_controller/certified_dwb_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

namespace
{

geometry_msgs::msg::Pose2D integrate_pose(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double time_step)
{
  geometry_msgs::msg::Pose2D next = pose;
  next.x +=
    (velocity.x * std::cos(pose.theta) -
    velocity.y * std::sin(pose.theta)) * time_step;
  next.y +=
    (velocity.x * std::sin(pose.theta) +
    velocity.y * std::cos(pose.theta)) * time_step;
  next.theta += velocity.theta * time_step;
  return next;
}

bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

void CertifiedDWBLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock controller server node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_certification",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_nominal_delay_preview",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".nominal_delay_preview_seconds",
    rclcpp::ParameterValue(0.07));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".certification_control_period",
    rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_maximum_time",
    rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_velocity_threshold",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_certified_margin",
    rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".maximum_swept_distance",
    rclcpp::ParameterValue(0.025));

  node->get_parameter(name + ".enable_certification", certification_enabled_);
  node->get_parameter(
    name + ".enable_nominal_delay_preview",
    nominal_delay_preview_enabled_);
  node->get_parameter(
    name + ".nominal_delay_preview_seconds",
    nominal_delay_preview_seconds_);
  node->get_parameter(
    name + ".certification_control_period",
    certification_control_period_);
  node->get_parameter(
    name + ".terminal_stop_maximum_time",
    terminal_stop_maximum_time_);
  node->get_parameter(
    name + ".terminal_stop_velocity_threshold",
    terminal_stop_velocity_threshold_);
  node->get_parameter(
    name + ".minimum_certified_margin",
    minimum_certified_margin_);
  node->get_parameter(
    name + ".maximum_swept_distance",
    maximum_swept_distance_);

  if (!std::isfinite(nominal_delay_preview_seconds_) ||
    nominal_delay_preview_seconds_ < 0.0 ||
    !is_positive_finite(certification_control_period_) ||
    !is_positive_finite(terminal_stop_maximum_time_) ||
    !is_positive_finite(terminal_stop_velocity_threshold_) ||
    !std::isfinite(minimum_certified_margin_) ||
    minimum_certified_margin_ < 0.0 ||
    !is_positive_finite(maximum_swept_distance_))
  {
    throw nav2_core::ControllerException(
            "Invalid delay-preview or trajectory-certification parameter");
  }

  const std::string plugin_name = name;
  dwb_core::DWBLocalPlanner::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));

  node->get_parameter(
    plugin_name + ".min_vel_x", minimum_linear_velocity_);
  node->get_parameter(
    plugin_name + ".max_vel_x", maximum_linear_velocity_);
  node->get_parameter(
    plugin_name + ".max_vel_theta", maximum_angular_velocity_);
  node->get_parameter(
    plugin_name + ".acc_lim_x", maximum_linear_acceleration_);
  node->get_parameter(
    plugin_name + ".decel_lim_x", maximum_linear_deceleration_);
  node->get_parameter(
    plugin_name + ".acc_lim_theta", maximum_angular_acceleration_);
  node->get_parameter(
    plugin_name + ".decel_lim_theta", maximum_angular_deceleration_);
}

geometry_msgs::msg::TwistStamped
CertifiedDWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (!nominal_delay_preview_enabled_ ||
    nominal_delay_preview_seconds_ == 0.0)
  {
    return dwb_core::DWBLocalPlanner::computeVelocityCommands(
      pose, velocity, goal_checker);
  }

  nav_2d_msgs::msg::Pose2DStamped preview_pose =
    nav_2d_utils::poseStampedToPose2D(pose);
  preview_pose.pose = integrate_pose(
    preview_pose.pose, nav_2d_utils::twist3Dto2D(velocity),
    nominal_delay_preview_seconds_);
  return dwb_core::DWBLocalPlanner::computeVelocityCommands(
    nav_2d_utils::pose2DToPoseStamped(preview_pose),
    velocity, goal_checker);
}

void CertifiedDWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  retained_backup_commands_.clear();
  current_candidate_stop_velocities_.clear();
  dwb_core::DWBLocalPlanner::setPlan(path);
}

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (!certification_enabled_) {
    return dwb_core::DWBLocalPlanner::coreScoringAlgorithm(
      pose, velocity, results);
  }

  dwb_msgs::msg::TrajectoryScore best;
  dwb_msgs::msg::TrajectoryScore worst;
  best.total = -1.0;
  worst.total = -1.0;
  std::vector<nav_2d_msgs::msg::Twist2D> best_stop_velocities;
  dwb_core::IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D twist =
      traj_generator_->nextTwist();
    const dwb_msgs::msg::Trajectory2D trajectory =
      traj_generator_->generateTrajectory(pose, velocity, twist);
    try {
      const dwb_msgs::msg::TrajectoryScore score =
        scoreTrajectory(trajectory, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0.0 || score.total < best.total) {
        best = score;
        best_stop_velocities = current_candidate_stop_velocities_;
        if (results) {
          results->best_index = results->twists.size() - 1u;
        }
      }
      if (worst.total < 0.0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1u;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & exception) {
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = trajectory;
        dwb_msgs::msg::CriticScore critic_score;
        critic_score.name = exception.getCriticName();
        critic_score.raw_score = -1.0;
        failed_score.scores.push_back(critic_score);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(exception);
    }
  }

  if (best.total >= 0.0) {
    retained_backup_commands_.clear();
    if (best_stop_velocities.size() > 1u) {
      retained_backup_commands_.assign(
        best_stop_velocities.begin() + 1,
        best_stop_velocities.end());
    }
    return best;
  }

  dwb_msgs::msg::TrajectoryScore backup_score;
  if (build_revalidated_backup(pose, backup_score)) {
    if (results) {
      results->twists.push_back(backup_score);
      results->best_index = results->twists.size() - 1u;
    }
    if (retained_backup_commands_.size() > 1u) {
      retained_backup_commands_.erase(
        retained_backup_commands_.begin());
    }
    return backup_score;
  }

  if (debug_trajectory_details_) {
    RCLCPP_ERROR(logger_, "%s", tracker.getMessage().c_str());
    for (const auto & percentage : tracker.getPercentages()) {
      RCLCPP_ERROR(
        logger_, "%.2f: %10s/%s", percentage.second,
        percentage.first.first.c_str(),
        percentage.first.second.c_str());
    }
  }
  throw dwb_core::NoLegalTrajectoriesException(tracker);
}

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double best_score)
{
  current_candidate_stop_velocities_.clear();
  if (certification_enabled_) {
    std::vector<geometry_msgs::msg::Pose2D> stop_poses;
    std::vector<nav_2d_msgs::msg::Twist2D> stop_velocities;
    if (!build_stop_trajectory(
        trajectory, stop_poses, stop_velocities))
    {
      throw dwb_core::IllegalTrajectoryException(
              "SafetyCertificate",
              "No dynamically feasible terminal stop sequence");
    }

    CertificationFailure failure = CertificationFailure::kInvalidInput;
    if (!certify_stop_poses(stop_poses, failure)) {
      throw dwb_core::IllegalTrajectoryException(
              "SafetyCertificate",
              certification_failure_name(failure));
    }
    current_candidate_stop_velocities_ = std::move(stop_velocities);
  }
  return dwb_core::DWBLocalPlanner::scoreTrajectory(
    trajectory, best_score);
}

bool CertifiedDWBLocalPlanner::build_stop_trajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> & velocities)
{
  poses.clear();
  velocities.clear();
  if (trajectory.poses.empty()) {
    return false;
  }
  const int maximum_stop_steps =
    static_cast<int>(
    std::ceil(
      terminal_stop_maximum_time_ / certification_control_period_));
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    return native_generator->generate_stop_trajectory(
      trajectory.poses.front(), maximum_stop_steps,
      terminal_stop_velocity_threshold_, poses, velocities);
  }

  const StopSequence linear_stop =
    generate_acceleration_stop_sequence(
    AxisState{trajectory.velocity.x, 0.0}, linear_limits(),
    certification_control_period_, maximum_stop_steps,
    terminal_stop_velocity_threshold_);
  const StopSequence angular_stop =
    generate_acceleration_stop_sequence(
    AxisState{trajectory.velocity.theta, 0.0}, angular_limits(),
    certification_control_period_, maximum_stop_steps,
    terminal_stop_velocity_threshold_);
  if (!linear_stop.feasible || !angular_stop.feasible ||
    !linear_stop.terminal_state_cleared ||
    !angular_stop.terminal_state_cleared)
  {
    return false;
  }

  geometry_msgs::msg::Pose2D pose = trajectory.poses.front();
  poses.push_back(pose);
  pose = integrate_pose(
    pose, trajectory.velocity, certification_control_period_);
  velocities.push_back(trajectory.velocity);
  poses.push_back(pose);
  const std::size_t stop_step_count =
    std::max(linear_stop.states.size(), angular_stop.states.size());
  for (std::size_t step_index = 0;
    step_index < stop_step_count; ++step_index)
  {
    nav_2d_msgs::msg::Twist2D stop_velocity;
    if (step_index < linear_stop.states.size()) {
      stop_velocity.x = linear_stop.states[step_index].velocity;
    }
    if (step_index < angular_stop.states.size()) {
      stop_velocity.theta = angular_stop.states[step_index].velocity;
    }
    pose = integrate_pose(
      pose, stop_velocity, certification_control_period_);
    velocities.push_back(stop_velocity);
    poses.push_back(pose);
  }
  nav_2d_msgs::msg::Twist2D zero_velocity;
  velocities.push_back(zero_velocity);
  poses.push_back(pose);
  return true;
}

bool CertifiedDWBLocalPlanner::build_revalidated_backup(
  const geometry_msgs::msg::Pose2D & start_pose,
  dwb_msgs::msg::TrajectoryScore & backup_score)
{
  if (retained_backup_commands_.empty()) {
    return false;
  }

  std::vector<geometry_msgs::msg::Pose2D> poses;
  poses.reserve(retained_backup_commands_.size() + 1u);
  geometry_msgs::msg::Pose2D pose = start_pose;
  poses.push_back(pose);
  for (const nav_2d_msgs::msg::Twist2D & command :
    retained_backup_commands_)
  {
    pose = integrate_pose(
      pose, command, certification_control_period_);
    poses.push_back(pose);
  }

  CertificationFailure failure = CertificationFailure::kInvalidInput;
  if (!certify_stop_poses(poses, failure)) {
    RCLCPP_WARN(
      logger_, "Retained stop backup rejected during revalidation: %s",
      certification_failure_name(failure));
    retained_backup_commands_.clear();
    return false;
  }

  backup_score = dwb_msgs::msg::TrajectoryScore();
  backup_score.total = 0.0;
  backup_score.traj.velocity = retained_backup_commands_.front();
  backup_score.traj.poses = std::move(poses);
  dwb_msgs::msg::CriticScore certificate_score;
  certificate_score.name = "RetainedSafetyBackup";
  certificate_score.scale = 1.0;
  certificate_score.raw_score = 0.0;
  backup_score.scores.push_back(certificate_score);
  return true;
}

bool CertifiedDWBLocalPlanner::certify_stop_poses(
  const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
  CertificationFailure & failure) const
{
  std::vector<geometry_msgs::msg::Point> certified_footprint =
    costmap_ros_->getRobotFootprint();
  nav2_costmap_2d::padFootprint(
    certified_footprint, minimum_certified_margin_);
  const CertificationResult result =
    certify_pose_sequence(
    *costmap_ros_->getCostmap(), certified_footprint, stop_poses,
    maximum_swept_distance_);
  failure = result.failure;
  return result.safe;
}

AxisLimits CertifiedDWBLocalPlanner::linear_limits() const
{
  return AxisLimits{
    minimum_linear_velocity_, maximum_linear_velocity_,
    maximum_linear_deceleration_, maximum_linear_acceleration_,
    maximum_linear_deceleration_, maximum_linear_acceleration_};
}

AxisLimits CertifiedDWBLocalPlanner::angular_limits() const
{
  return AxisLimits{
    -maximum_angular_velocity_, maximum_angular_velocity_,
    maximum_angular_deceleration_, maximum_angular_acceleration_,
    maximum_angular_deceleration_, maximum_angular_acceleration_};
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::CertifiedDWBLocalPlanner,
  nav2_core::Controller)
