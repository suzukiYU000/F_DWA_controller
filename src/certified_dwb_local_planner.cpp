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
#include <charconv>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "dispatch_epoch_gate.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/path_subgoal.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "f_dwa_controller/velocity_response_model.hpp"
#include "f_dwa_controller/v_dwb_trajectory_generators.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace f_dwa_controller
{

namespace
{

uint64_t steady_time_nanoseconds(
  const std::chrono::steady_clock::time_point time_point)
{
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
    time_point.time_since_epoch()).count();
  return count > 0 ? static_cast<uint64_t>(count) : 0u;
}

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

double normalized_cost_rank(
  const double score, const std::vector<double> & sorted_costs)
{
  if (sorted_costs.size() < 2u) {
    return 0.0;
  }
  const auto position = std::lower_bound(
    sorted_costs.begin(), sorted_costs.end(), score);
  const auto rank = static_cast<double>(
    std::distance(sorted_costs.begin(), position));
  return std::clamp(
    rank / static_cast<double>(sorted_costs.size() - 1u), 0.0, 1.0);
}

std_msgs::msg::ColorRGBA weighted_cost_color(const double normalized_rank)
{
  const double ratio = std::clamp(normalized_rank, 0.0, 1.0);
  std_msgs::msg::ColorRGBA color;
  if (ratio <= 0.5) {
    const double local = ratio * 2.0;
    color.r = static_cast<float>(0.05 * (1.0 - local));
    color.g = static_cast<float>(0.35 + 0.55 * local);
    color.b = static_cast<float>(1.00 - 0.05 * local);
  } else {
    const double local = (ratio - 0.5) * 2.0;
    color.r = static_cast<float>(local);
    color.g = static_cast<float>(0.90 - 0.08 * local);
    color.b = static_cast<float>(0.95 - 0.90 * local);
  }
  color.a = 0.36F;
  return color;
}

double clearance_constraint_score_limit(
  const bool constraint_active,
  const bool candidate_has_meaningful_progress,
  const bool best_has_meaningful_progress,
  const uint64_t candidate_guard_risk_bucket,
  const uint64_t best_guard_risk_bucket,
  const uint64_t candidate_risk_bucket,
  const uint64_t best_risk_bucket,
  const bool best_uses_reserve_recovery,
  const double best_total)
{
  const double normal_limit = best_uses_reserve_recovery ? -1.0 : best_total;
  if (!constraint_active) {
    return normal_limit;
  }
  if (candidate_guard_risk_bucket != best_guard_risk_bucket) {
    return candidate_guard_risk_bucket < best_guard_risk_bucket ?
           -1.0 : std::numeric_limits<double>::min();
  }
  if (candidate_risk_bucket != best_risk_bucket) {
    return candidate_risk_bucket < best_risk_bucket ?
           -1.0 : std::numeric_limits<double>::min();
  }
  // Once both candidates are in the same physical-clearance risk bands,
  // evaluate the complete weighted DWB objective. The progress flag remains
  // a deterministic exact-score tie-break; a separate stalled-selection gate
  // handles the method-native response needed to leave a zero-command local
  // minimum without changing ordinary avoidance ranking.
  (void)candidate_has_meaningful_progress;
  (void)best_has_meaningful_progress;
  return normal_limit;
}

std::string candidate_diagnostic_metadata(
  const NativeInputTrajectoryGenerator::ActiveCandidateDiagnostics & value)
{
  std::ostringstream stream;
  stream << std::setprecision(17)
         << "canonical_index=" << value.canonical_index
         << ";linear_native_input=" << value.linear_native_input
         << ";angular_native_input=" << value.angular_native_input
         << ";initial_linear_velocity=" << value.initial_linear_velocity
         << ";initial_angular_velocity=" << value.initial_angular_velocity
         << ";initial_linear_acceleration="
         << value.initial_linear_acceleration
         << ";initial_angular_acceleration="
         << value.initial_angular_acceleration
         << ";first_linear_velocity="
         << value.first_command_state.linear_state.velocity
         << ";first_angular_velocity="
         << value.first_command_state.angular_state.velocity
         << ";first_linear_acceleration="
         << value.first_command_state.linear_state.acceleration
         << ";first_angular_acceleration="
         << value.first_command_state.angular_state.acceleration;
  return stream.str();
}

constexpr double kCommandMatchTolerance = 1.0e-9;

bool commands_match(
  const nav_2d_msgs::msg::Twist2D & first,
  const nav_2d_msgs::msg::Twist2D & second)
{
  return std::abs(first.x - second.x) <= kCommandMatchTolerance &&
         std::abs(first.y - second.y) <= kCommandMatchTolerance &&
         std::abs(first.theta - second.theta) <= kCommandMatchTolerance;
}

bool is_safety_command_reduction(
  const nav_2d_msgs::msg::Twist2D & issued,
  const nav_2d_msgs::msg::Twist2D & applied)
{
  const auto axis_is_reduced =
    [](const double expected, const double observed) {
      if (std::abs(expected) <= kCommandMatchTolerance) {
        return std::abs(observed) <= kCommandMatchTolerance;
      }
      return expected * observed >= -kCommandMatchTolerance &&
             std::abs(observed) <=
             std::abs(expected) + kCommandMatchTolerance;
    };
  return axis_is_reduced(issued.x, applied.x) &&
         axis_is_reduced(issued.y, applied.y) &&
         axis_is_reduced(issued.theta, applied.theta);
}

void append_integrated_motion(
  geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double duration,
  const double maximum_time_step,
  std::vector<geometry_msgs::msg::Pose2D> & poses)
{
  if (duration <= 0.0) {
    return;
  }
  const int step_count = std::max(
    1, static_cast<int>(std::ceil(duration / maximum_time_step)));
  const double time_step = duration / static_cast<double>(step_count);
  for (int step_index = 0; step_index < step_count; ++step_index) {
    pose = integrate_pose(pose, velocity, time_step);
    poses.push_back(pose);
  }
}

double quantile(
  const std::vector<double> & sorted_values,
  const double probability)
{
  if (sorted_values.empty()) {
    return 0.0;
  }
  const double index =
    probability * static_cast<double>(sorted_values.size() - 1u);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double ratio = index - static_cast<double>(lower);
  return sorted_values[lower] +
         ratio * (sorted_values[upper] - sorted_values[lower]);
}

}  // namespace

CertifiedDWBLocalPlanner::~CertifiedDWBLocalPlanner()
{
  stop_diagnostic_publisher();
}

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
  bool publish_zero_velocity = false;
  if (!node->get_parameter("publish_zero_velocity", publish_zero_velocity) ||
    !publish_zero_velocity)
  {
    throw nav2_core::ControllerException(
            "CertifiedDWBLocalPlanner requires Controller Server "
            "publish_zero_velocity=true for failure-stop FIFO tracking");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_certification",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_clearance_constraint",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_critic_name",
    rclcpp::ParameterValue(std::string{"FootprintClearance"}));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_trigger_critic_name",
    rclcpp::ParameterValue(std::string{"FootprintClearance"}));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_guard_critic_name",
    rclcpp::ParameterValue(std::string{"FootprintClearance"}));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_admissible_risk",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_trigger_risk",
    rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_risk_resolution",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_guard_admissible_risk",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_guard_risk_resolution",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_minimum_subgoal_distance_progress",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_minimum_subgoal_heading_progress",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_motion_preference_goal_distance",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_include_footprint_approach",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".clearance_constraint_footprint_approach_trigger_risk",
    rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_nominal_delay_preview",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_observed_velocity_for_activation_state",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_velocity_response_prediction",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".nominal_delay_preview_seconds",
    rclcpp::ParameterValue(0.07));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".velocity_response_prediction_seconds",
    rclcpp::ParameterValue(0.12));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".velocity_response_integration_step_seconds",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".linear_velocity_response_dead_time_seconds",
    rclcpp::ParameterValue(0.035));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".linear_velocity_response_time_constant_seconds",
    rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".linear_velocity_response_gain",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angular_velocity_response_dead_time_seconds",
    rclcpp::ParameterValue(0.015));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angular_velocity_response_time_constant_seconds",
    rclcpp::ParameterValue(0.085));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angular_velocity_response_gain",
    rclcpp::ParameterValue(0.95));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".command_dispatch_topic",
    rclcpp::ParameterValue("/controller/command_dispatch"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".require_command_dispatch_state",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_safety_command_reduction",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".transport_valid_topic",
    rclcpp::ParameterValue("/dwa_experiment/transport_valid"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".transport_invalidation_service",
    rclcpp::ParameterValue("/command_delay_transport/invalidate_trial"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".certification_control_period",
    rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_no_valid_control_deceleration_fallback",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_stop_admissibility",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_maximum_time",
    rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_velocity_threshold",
    rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_goal_distance_scale",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_distance_target",
    rclcpp::ParameterValue(std::string("global_goal")));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_path_lookahead_distance",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_path_lateral_weight",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_goal_capture_distance",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_goal_capture_yaw_tolerance",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_certified_margin",
    rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".maximum_swept_distance",
    rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_reserve_recovery",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reserve_recovery_hysteresis",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enable_initial_overlap_recovery",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".initial_overlap_footprint_inset",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".trial_reset_service_name",
    rclcpp::ParameterValue("~/" + name + "/reset_trial_state"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_metrics_enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_metrics_report_interval",
    rclcpp::ParameterValue(1000));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_deadline_seconds",
    rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".publish_evaluation",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".evaluation_publish_frequency",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".publish_candidate_markers",
    rclcpp::ParameterValue(true));

  node->get_parameter(name + ".enable_certification", certification_enabled_);
  node->get_parameter(
    name + ".enable_clearance_constraint", clearance_constraint_enabled_);
  node->get_parameter(
    name + ".clearance_constraint_critic_name",
    clearance_constraint_critic_name_);
  node->get_parameter(
    name + ".clearance_constraint_trigger_critic_name",
    clearance_constraint_trigger_critic_name_);
  node->get_parameter(
    name + ".clearance_constraint_guard_critic_name",
    clearance_constraint_guard_critic_name_);
  node->get_parameter(
    name + ".clearance_constraint_admissible_risk",
    clearance_constraint_admissible_risk_);
  node->get_parameter(
    name + ".clearance_constraint_trigger_risk",
    clearance_constraint_trigger_risk_);
  node->get_parameter(
    name + ".clearance_constraint_risk_resolution",
    clearance_constraint_risk_resolution_);
  node->get_parameter(
    name + ".clearance_constraint_guard_admissible_risk",
    clearance_constraint_guard_admissible_risk_);
  node->get_parameter(
    name + ".clearance_constraint_guard_risk_resolution",
    clearance_constraint_guard_risk_resolution_);
  node->get_parameter(
    name + ".clearance_constraint_minimum_subgoal_distance_progress",
    clearance_constraint_minimum_subgoal_distance_progress_);
  node->get_parameter(
    name + ".clearance_constraint_minimum_subgoal_heading_progress",
    clearance_constraint_minimum_subgoal_heading_progress_);
  node->get_parameter(
    name + ".clearance_constraint_motion_preference_goal_distance",
    clearance_constraint_motion_preference_goal_distance_);
  node->get_parameter(
    name + ".clearance_constraint_include_footprint_approach",
    clearance_constraint_include_footprint_approach_);
  node->get_parameter(
    name + ".clearance_constraint_footprint_approach_trigger_risk",
    clearance_constraint_footprint_approach_trigger_risk_);
  node->get_parameter(
    name + ".enable_no_valid_control_deceleration_fallback",
    no_valid_control_deceleration_fallback_enabled_);
  node->get_parameter(
    name + ".enable_stop_admissibility",
    stop_admissibility_enabled_);
  node->get_parameter(
    name + ".enable_nominal_delay_preview",
    nominal_delay_preview_enabled_);
  node->get_parameter(
    name + ".use_observed_velocity_for_activation_state",
    use_observed_velocity_for_activation_state_);
  node->get_parameter(
    name + ".enable_velocity_response_prediction",
    velocity_response_prediction_enabled_);
  node->get_parameter(
    name + ".nominal_delay_preview_seconds",
    nominal_delay_preview_seconds_);
  node->get_parameter(
    name + ".velocity_response_prediction_seconds",
    velocity_response_prediction_seconds_);
  node->get_parameter(
    name + ".velocity_response_integration_step_seconds",
    velocity_response_integration_step_seconds_);
  node->get_parameter(
    name + ".linear_velocity_response_dead_time_seconds",
    linear_velocity_response_model_.dead_time_seconds);
  node->get_parameter(
    name + ".linear_velocity_response_time_constant_seconds",
    linear_velocity_response_model_.time_constant_seconds);
  node->get_parameter(
    name + ".linear_velocity_response_gain",
    linear_velocity_response_model_.steady_state_gain);
  node->get_parameter(
    name + ".angular_velocity_response_dead_time_seconds",
    angular_velocity_response_model_.dead_time_seconds);
  node->get_parameter(
    name + ".angular_velocity_response_time_constant_seconds",
    angular_velocity_response_model_.time_constant_seconds);
  node->get_parameter(
    name + ".angular_velocity_response_gain",
    angular_velocity_response_model_.steady_state_gain);
  node->get_parameter(
    name + ".require_command_dispatch_state",
    require_command_dispatch_state_);
  node->get_parameter(
    name + ".allow_safety_command_reduction",
    allow_safety_command_reduction_);
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
    name + ".terminal_stop_goal_distance_scale",
    terminal_stop_goal_distance_scale_);
  const std::string terminal_stop_distance_target =
    node->get_parameter(
    name + ".terminal_stop_distance_target").as_string();
  node->get_parameter(
    name + ".terminal_stop_path_lookahead_distance",
    terminal_stop_path_lookahead_distance_);
  node->get_parameter(
    name + ".terminal_stop_path_lateral_weight",
    terminal_stop_path_lateral_weight_);
  if (terminal_stop_distance_target == "global_goal") {
    terminal_stop_score_mode_ =
      TerminalStopScoreMode::kGlobalGoalDistance;
  } else if (terminal_stop_distance_target == "path_subgoal") {
    terminal_stop_score_mode_ =
      TerminalStopScoreMode::kPathSubgoalDistance;
  } else if (terminal_stop_distance_target == "path_progress") {
    terminal_stop_score_mode_ =
      TerminalStopScoreMode::kPathSubgoalProgress;
  } else if (terminal_stop_distance_target == "path_progress_ray") {
    terminal_stop_score_mode_ =
      TerminalStopScoreMode::kPathSubgoalForwardRay;
  } else {
    throw nav2_core::ControllerException(
            "terminal_stop_distance_target must be global_goal, "
            "path_subgoal, path_progress, or path_progress_ray");
  }
  node->get_parameter(
    name + ".terminal_stop_goal_capture_distance",
    terminal_stop_goal_capture_distance_);
  node->get_parameter(
    name + ".terminal_stop_goal_capture_yaw_tolerance",
    terminal_stop_goal_capture_yaw_tolerance_);
  node->get_parameter(
    name + ".minimum_certified_margin",
    minimum_certified_margin_);
  node->get_parameter(
    name + ".maximum_swept_distance",
    maximum_swept_distance_);
  node->get_parameter(
    name + ".enable_reserve_recovery",
    enable_reserve_recovery_);
  node->get_parameter(
    name + ".reserve_recovery_hysteresis",
    reserve_recovery_hysteresis_);
  node->get_parameter(
    name + ".enable_initial_overlap_recovery",
    enable_initial_overlap_recovery_);
  node->get_parameter(
    name + ".initial_overlap_footprint_inset",
    initial_overlap_footprint_inset_);
  node->get_parameter(
    name + ".planning_metrics_enabled", planning_metrics_enabled_);
  node->get_parameter(
    name + ".planning_metrics_report_interval",
    planning_metrics_report_interval_);
  node->get_parameter(
    name + ".planning_deadline_seconds", planning_deadline_seconds_);
  node->get_parameter(
    name + ".publish_evaluation", publish_evaluation_);
  node->get_parameter(
    name + ".evaluation_publish_frequency",
    evaluation_publish_frequency_);
  node->get_parameter(
    name + ".publish_candidate_markers",
    publish_candidate_markers_);
  const std::string command_dispatch_topic =
    node->get_parameter(name + ".command_dispatch_topic").as_string();
  const std::string transport_valid_topic =
    node->get_parameter(name + ".transport_valid_topic").as_string();
  const std::string transport_invalidation_service =
    node->get_parameter(
    name + ".transport_invalidation_service").as_string();
  const std::string trial_reset_service_name =
    node->get_parameter(name + ".trial_reset_service_name").as_string();

  if (!std::isfinite(nominal_delay_preview_seconds_) ||
    nominal_delay_preview_seconds_ < 0.0 ||
    !std::isfinite(velocity_response_prediction_seconds_) ||
    velocity_response_prediction_seconds_ < 0.0 ||
    !is_positive_finite(velocity_response_integration_step_seconds_) ||
    !valid_velocity_response_model(linear_velocity_response_model_) ||
    !valid_velocity_response_model(angular_velocity_response_model_) ||
    !is_positive_finite(certification_control_period_) ||
    !is_positive_finite(terminal_stop_maximum_time_) ||
    !is_positive_finite(terminal_stop_velocity_threshold_) ||
    !std::isfinite(terminal_stop_goal_distance_scale_) ||
    terminal_stop_goal_distance_scale_ < 0.0 ||
    !is_positive_finite(terminal_stop_path_lookahead_distance_) ||
    !std::isfinite(terminal_stop_path_lateral_weight_) ||
    terminal_stop_path_lateral_weight_ < 0.0 ||
    !std::isfinite(terminal_stop_goal_capture_distance_) ||
    terminal_stop_goal_capture_distance_ < 0.0 ||
    !std::isfinite(terminal_stop_goal_capture_yaw_tolerance_) ||
    terminal_stop_goal_capture_yaw_tolerance_ < 0.0 ||
    !std::isfinite(minimum_certified_margin_) ||
    minimum_certified_margin_ < 0.0 ||
    !std::isfinite(initial_overlap_footprint_inset_) ||
    initial_overlap_footprint_inset_ <= 0.0 ||
    !std::isfinite(clearance_constraint_admissible_risk_) ||
    clearance_constraint_admissible_risk_ < 0.0 ||
    clearance_constraint_admissible_risk_ > 1.0 ||
    !std::isfinite(clearance_constraint_trigger_risk_) ||
    clearance_constraint_trigger_risk_ < -1.0 ||
    clearance_constraint_trigger_risk_ > 1.0 ||
    !is_positive_finite(clearance_constraint_risk_resolution_) ||
    clearance_constraint_risk_resolution_ > 1.0 ||
    !std::isfinite(clearance_constraint_guard_admissible_risk_) ||
    clearance_constraint_guard_admissible_risk_ < 0.0 ||
    clearance_constraint_guard_admissible_risk_ > 1.0 ||
    !is_positive_finite(clearance_constraint_guard_risk_resolution_) ||
    clearance_constraint_guard_risk_resolution_ > 1.0 ||
    !std::isfinite(
      clearance_constraint_minimum_subgoal_distance_progress_) ||
    clearance_constraint_minimum_subgoal_distance_progress_ < 0.0 ||
    !std::isfinite(
      clearance_constraint_minimum_subgoal_heading_progress_) ||
    clearance_constraint_minimum_subgoal_heading_progress_ < 0.0 ||
    !std::isfinite(clearance_constraint_motion_preference_goal_distance_) ||
    clearance_constraint_motion_preference_goal_distance_ < 0.0 ||
    !std::isfinite(clearance_constraint_footprint_approach_trigger_risk_) ||
    clearance_constraint_footprint_approach_trigger_risk_ < 0.0 ||
    clearance_constraint_footprint_approach_trigger_risk_ > 1.0 ||
    !is_positive_finite(maximum_swept_distance_) ||
    planning_metrics_report_interval_ <= 0 ||
    !is_positive_finite(planning_deadline_seconds_) ||
    !std::isfinite(evaluation_publish_frequency_) ||
    evaluation_publish_frequency_ < 0.0)
  {
    throw nav2_core::ControllerException(
            "Invalid delay-preview or trajectory-certification parameter");
  }
  if (velocity_response_prediction_enabled_ &&
    !use_observed_velocity_for_activation_state_)
  {
    throw nav2_core::ControllerException(
            "Velocity response prediction requires observed odometry state");
  }

  const std::string plugin_name = name;
  dwb_core::DWBLocalPlanner::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));

  clearance_constraint_critic_.reset();
  clearance_constraint_trigger_critic_.reset();
  clearance_constraint_guard_critic_.reset();
  clearance_constraint_guard_footprint_critic_.reset();
  for (const auto & critic : critics_) {
    if (critic->getName() == clearance_constraint_trigger_critic_name_) {
      if (clearance_constraint_trigger_critic_) {
        throw nav2_core::ControllerException(
                "clearance_constraint_trigger_critic_name is not unique: " +
                clearance_constraint_trigger_critic_name_);
      }
      clearance_constraint_trigger_critic_ = critic;
    }
    if (critic->getName() == clearance_constraint_guard_critic_name_) {
      if (clearance_constraint_guard_critic_) {
        throw nav2_core::ControllerException(
                "clearance_constraint_guard_critic_name is not unique: " +
                clearance_constraint_guard_critic_name_);
      }
      clearance_constraint_guard_critic_ = critic;
    }

    const auto clearance_critic =
      std::dynamic_pointer_cast<FootprintClearanceCritic>(critic);
    if (!clearance_critic) {
      continue;
    }
    if (clearance_critic->getName() == clearance_constraint_critic_name_) {
      if (clearance_constraint_critic_) {
        throw nav2_core::ControllerException(
                "clearance_constraint_critic_name is not unique: " +
                clearance_constraint_critic_name_);
      }
      clearance_constraint_critic_ = clearance_critic;
    }
  }
  if (clearance_constraint_enabled_ && !clearance_constraint_critic_) {
    throw nav2_core::ControllerException(
            "clearance constraint critic was not found: " +
            clearance_constraint_critic_name_);
  }
  if (clearance_constraint_enabled_ &&
    !clearance_constraint_trigger_critic_)
  {
    throw nav2_core::ControllerException(
            "clearance constraint trigger critic was not found: " +
            clearance_constraint_trigger_critic_name_);
  }
  if (clearance_constraint_enabled_ && !clearance_constraint_guard_critic_) {
    throw nav2_core::ControllerException(
            "clearance constraint guard critic was not found: " +
            clearance_constraint_guard_critic_name_);
  }
  clearance_constraint_guard_footprint_critic_ =
    std::dynamic_pointer_cast<FootprintClearanceCritic>(
    clearance_constraint_guard_critic_);
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

  clock_ = node->get_clock();
  // Evaluation is diagnostic output. Buffer two seconds at the common 10 Hz
  // publish rate so recorder discovery or a short storage flush does not
  // overwrite a sample in the reliable writer before it is delivered.
  evaluation_publisher_ =
    node->create_publisher<dwb_msgs::msg::LocalPlanEvaluation>(
    "evaluation", rclcpp::QoS(20));
  candidate_marker_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "dwb_candidate_trajectories_realtime",
    rclcpp::QoS(1).best_effort());
  command_dispatch_subscriber_ =
    node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    command_dispatch_topic,
    rclcpp::QoS(64).reliable().transient_local(),
    std::bind(
      &CertifiedDWBLocalPlanner::command_dispatch_callback,
      this, std::placeholders::_1));
  transport_valid_subscriber_ =
    node->create_subscription<std_msgs::msg::Bool>(
    transport_valid_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &CertifiedDWBLocalPlanner::transport_valid_callback,
      this, std::placeholders::_1));
  transport_invalidation_client_ =
    node->create_client<std_srvs::srv::Trigger>(
    transport_invalidation_service);
  reset_trial_service_ = node->create_service<std_srvs::srv::Trigger>(
    trial_reset_service_name,
    std::bind(
      &CertifiedDWBLocalPlanner::reset_trial_callback,
      this, std::placeholders::_1, std::placeholders::_2));
  if (velocity_response_prediction_enabled_) {
    RCLCPP_INFO(
      logger_,
      "real velocity response prediction enabled: horizon=%.3f s, "
      "linear=(L=%.3f,T=%.3f,K=%.3f), "
      "angular=(L=%.3f,T=%.3f,K=%.3f)",
      velocity_response_prediction_seconds_,
      linear_velocity_response_model_.dead_time_seconds,
      linear_velocity_response_model_.time_constant_seconds,
      linear_velocity_response_model_.steady_state_gain,
      angular_velocity_response_model_.dead_time_seconds,
      angular_velocity_response_model_.time_constant_seconds,
      angular_velocity_response_model_.steady_state_gain);
  }
}  // NOLINT(readability/fn_size)

void CertifiedDWBLocalPlanner::activate()
{
  dwb_core::DWBLocalPlanner::activate();
  evaluation_publisher_->on_activate();
  candidate_marker_publisher_->on_activate();
  start_diagnostic_publisher();
}

void CertifiedDWBLocalPlanner::deactivate()
{
  stop_diagnostic_publisher();
  evaluation_publisher_->on_deactivate();
  candidate_marker_publisher_->on_deactivate();
  dwb_core::DWBLocalPlanner::deactivate();
}

void CertifiedDWBLocalPlanner::cleanup()
{
  stop_diagnostic_publisher();
  command_dispatch_subscriber_.reset();
  transport_valid_subscriber_.reset();
  transport_invalidation_client_.reset();
  reset_trial_service_.reset();
  evaluation_publisher_.reset();
  candidate_marker_publisher_.reset();
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  terminal_stop_goal_capture_active_ = false;
  terminal_stop_goal_capture_committed_ = false;
  planning_snapshot_.reset();
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    pending_issued_commands_.clear();
    command_dispatch_observed_ = false;
    command_transport_valid_ = false;
    command_ledger_valid_ = false;
    expected_dispatch_sequence_ready_ = false;
    expected_dispatch_sequence_ = 0;
    dispatched_command_ = nav_2d_msgs::msg::Twist2D();
    dispatched_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dispatched_command_time_valid_ = false;
  }
  dwb_core::DWBLocalPlanner::cleanup();
}

geometry_msgs::msg::TwistStamped
CertifiedDWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  const auto planning_started_at = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> evaluation;
  const bool publish_full_evaluation = should_publish_evaluation();
  const bool publish_candidate_markers =
    should_publish_candidate_markers();
  if (publish_full_evaluation || publish_candidate_markers) {
    evaluation =
      std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }
  // Candidate markers depend on trajectories, legality, and the rejecting
  // critic, not on every CriticScore string. Keep the full message bit-for-bit
  // compatible on its configured publication cycles while avoiding that
  // allocation graph on the intervening 20 Hz marker cycles.
  record_full_evaluation_details_ = publish_full_evaluation;
  for (const auto & critic : critics_) {
    const auto obstacle_critic =
      std::dynamic_pointer_cast<HorizonObstacleFootprintCritic>(critic);
    if (obstacle_critic) {
      obstacle_critic->setDetailedFailureDiagnostics(
        record_full_evaluation_details_);
      obstacle_critic->setSharedCertificationWorkspace(
        &certification_workspace_);
    }
  }
  try {
    nav2_costmap_2d::Costmap2D * costmap =
      costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>
    certification_costmap_lock(*(costmap->getMutex()), std::defer_lock);
    const bool require_physical_delay_check =
      velocity_response_prediction_enabled_ && stop_admissibility_enabled_;
    if (certification_enabled_ || require_physical_delay_check) {
      // The mutex is recursive; base DWB takes it again while scoring. Keeping
      // this outer lock makes the committed-delay check, broadphase prefix,
      // critic preparation, and candidate certification use one snapshot.
      certification_costmap_lock.lock();
    }
    if (certification_enabled_ || enable_initial_overlap_recovery_) {
      prepare_collision_footprints();
    }
    planning_snapshot_ = build_planning_snapshot(pose, velocity);
    if (!planning_snapshot_->valid) {
      throw nav2_core::NoValidControl(
              "No valid robot-observable command-dispatch state");
    }

    auto native_generator =
      std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
      traj_generator_);
    auto v_dwa_generator =
      std::dynamic_pointer_cast<VLimitedAccelTrajectoryGenerator>(
      traj_generator_);
    if (native_generator) {
      PlanningSnapshot enriched_snapshot = *planning_snapshot_;
      native_generator->enrich_planning_snapshot(enriched_snapshot);
      planning_snapshot_ =
        std::make_shared<const PlanningSnapshot>(
        std::move(enriched_snapshot));
      native_generator->set_planning_snapshot(planning_snapshot_);
      if (!planning_snapshot_->activation_state.native_state_valid) {
        throw nav2_core::NoValidControl(
                "Native command state is not correlated with observed dispatches");
      }
    }
    if (v_dwa_generator) {
      if (!planning_snapshot_->activation_state.native_command_velocity_valid) {
        throw nav2_core::NoValidControl(
                "V-DWA command state is not correlated with observed dispatches");
      }
      v_dwa_generator->set_planning_snapshot(planning_snapshot_);
    }

    if (certification_enabled_ &&
      !planning_snapshot_->delay_trajectory.empty())
    {
      CertificationFailure failure = CertificationFailure::kInvalidInput;
      CertificationResult certification_result;
      if (!certify_stop_poses(
          planning_snapshot_->delay_trajectory, failure,
          &certification_result))
      {
        const CertificationResult planning_footprint_result =
          certify_pose_sequence(
          *costmap, costmap_ros_->getRobotFootprint(),
          planning_snapshot_->delay_trajectory,
          maximum_swept_distance_);
        if (enable_reserve_recovery_ &&
          failure == CertificationFailure::kLethalObstacle &&
          planning_footprint_result.safe)
        {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Committed delay prefix is inside only the additional "
            "certificate reserve; evaluating outward recovery candidates");
        } else {
          const geometry_msgs::msg::Pose2D & start_pose =
            planning_snapshot_->delay_trajectory.front();
          std::string diagnostic =
            std::string("Committed delay trajectory is unsafe: ") +
            certification_failure_name(failure) +
            "; committed_commands=" +
            std::to_string(planning_snapshot_->committed_commands.size()) +
            "; delay_poses=" +
            std::to_string(planning_snapshot_->delay_trajectory.size()) +
            "; activation_velocity=(" +
            std::to_string(planning_snapshot_->activation_state.velocity.x) +
            "," +
            std::to_string(
            planning_snapshot_->activation_state.velocity.theta) +
            "); planning_footprint_safe=" +
            (planning_footprint_result.safe ? "true" : "false") +
            "; planning_footprint_failure=" +
            certification_failure_name(planning_footprint_result.failure);
          if (certification_result.has_failure_pose) {
            const geometry_msgs::msg::Pose2D & failure_pose =
              certification_result.failure_pose;
            diagnostic +=
              "; failure_source_pose_index=" +
              std::to_string(
              certification_result.failure_source_pose_index) +
              "; failure_interpolation_index=" +
              std::to_string(
              certification_result.failure_interpolation_index) +
              "; failure_pose=(" +
              std::to_string(failure_pose.x) + "," +
              std::to_string(failure_pose.y) + "," +
              std::to_string(failure_pose.theta) + ")" +
              "; failure_travel_m=" +
              std::to_string(
              std::hypot(
                failure_pose.x - start_pose.x,
                failure_pose.y - start_pose.y));
          }
          throw nav2_core::NoValidControl(
                  diagnostic);
        }
      }
    }
    if (require_physical_delay_check && !certification_enabled_ &&
      !planning_snapshot_->delay_trajectory.empty())
    {
      const CertificationResult physical_delay_result = certify_pose_sequence(
        *costmap, costmap_ros_->getRobotFootprint(),
        planning_snapshot_->delay_trajectory, maximum_swept_distance_);
      if (!physical_delay_result.safe) {
        const bool recoverable_boundary_overlap =
          enable_initial_overlap_recovery_ &&
          physical_delay_result.failure ==
          CertificationFailure::kLethalObstacle &&
          certify_initial_overlap_margin_sequence(
          *costmap, costmap_ros_->getRobotFootprint(),
          initial_overlap_core_footprint_,
          planning_snapshot_->delay_trajectory,
          maximum_swept_distance_, nullptr,
          &certification_workspace_, true);
        if (recoverable_boundary_overlap) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Committed response overlaps only the physical-footprint "
            "boundary strip; evaluating core-safe maximum-effort candidates");
        } else {
          throw nav2_core::NoValidControl(
                  std::string("Predicted real response prefix is unsafe: ") +
                  certification_failure_name(physical_delay_result.failure));
        }
      }
    }

    nav_2d_msgs::msg::Pose2DStamped activation_pose =
      nav_2d_utils::poseStampedToPose2D(pose);
    activation_pose.pose = planning_snapshot_->activation_state.pose;
    const nav_2d_msgs::msg::Twist2DStamped command_2d =
      dwb_core::DWBLocalPlanner::computeVelocityCommands(
      activation_pose,
      planning_snapshot_->activation_state.velocity,
      evaluation);
    enqueue_diagnostic_publication(
      evaluation, publish_full_evaluation, publish_candidate_markers);
    geometry_msgs::msg::TwistStamped command;
    command.twist =
      nav_2d_utils::twist2Dto3D(command_2d.velocity);
    if (certification_costmap_lock.owns_lock()) {
      certification_costmap_lock.unlock();
    }
    const uint64_t issued_steady_time_ns =
      steady_time_nanoseconds(std::chrono::steady_clock::now());
    const rclcpp::Time issued_at = clock_->now();
    record_issued_command(command, issued_at, issued_steady_time_ns);
    record_planning_duration(planning_started_at);
    return command;
  } catch (const nav2_core::NoValidControl &) {
    enqueue_diagnostic_publication(
      evaluation, publish_full_evaluation, publish_candidate_markers);
    record_planning_duration(planning_started_at);
    // Do not predict a Controller Server zero here. On the final retry Nav2
    // may convert this exception to PatienceExceeded before publishing zero.
    // Only an actually applied CommandDispatch is allowed to change the
    // observable dynamics state or the command ledger.
    throw;
  } catch (...) {
    enqueue_diagnostic_publication(
      evaluation, publish_full_evaluation, publish_candidate_markers);
    record_planning_duration(planning_started_at);
    throw;
  }
}

void CertifiedDWBLocalPlanner::command_dispatch_callback(
  const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
{
  if (!message ||
    !std::isfinite(message->command.linear.x) ||
    !std::isfinite(message->command.angular.z))
  {
    request_transport_invalidation("invalid command-dispatch message");
    return;
  }

  std::lock_guard<std::mutex> controller_lock(controller_state_mutex_);
  const nav_2d_msgs::msg::Twist2D dispatched =
    nav_2d_utils::twist3Dto2D(message->command);
  const rclcpp::Time transport_received_at(message->received_at);
  const rclcpp::Time transport_dispatched_at(message->header.stamp);
  const uint64_t transport_received_steady_time_ns =
    message->received_steady_time_ns;
  if (message->has_sequence &&
    (transport_received_steady_time_ns == 0u ||
    transport_received_at > transport_dispatched_at))
  {
    request_transport_invalidation(
      transport_received_steady_time_ns == 0u ?
      "command-dispatch lacks monotonic reception provenance" :
      "command-dispatch reception time follows dispatch time");
    return;
  }
  bool dispatch_is_valid = true;
  bool dispatch_precedes_reset_boundary = false;
  bool ordered_external_stop = false;
  bool terminal_external_stop = false;
  bool sequence_was_ready = false;
  bool sequence_was_valid = false;
  bool command_was_matched = false;
  bool safety_reduction_was_accepted = false;
  std::size_t skipped_unpublished_command_count = 0u;
  std::size_t eligible_pending_count = 0u;
  uint64_t expected_sequence = 0;
  std::size_t pending_command_count = 0;
  double nearest_command_error = std::numeric_limits<double>::infinity();
  nav_2d_msgs::msg::Twist2D nearest_pending_command;
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    const detail::DispatchEpochAction epoch_action =
      detail::classify_dispatch_epoch(
      message->has_sequence, expected_dispatch_sequence_ready_);
    if (epoch_action ==
      detail::DispatchEpochAction::kIgnoreBeforeResetBoundary)
    {
      // A long Controller computation can leave sequenced dispatch callbacks
      // queued while the lifecycle manager tears this plugin down.  On the
      // next configure those callbacks belong to the previous trial: there is
      // no reset boundary or issued-command ledger with which to correlate
      // them.  Ignore them until the transport publishes its explicit
      // has_sequence=false reset state.  Treating them as a current mismatch
      // would invalidate an otherwise stopped, recoverable transport.
      dispatch_precedes_reset_boundary = true;
    } else if (epoch_action == detail::DispatchEpochAction::kApplyResetBoundary) {
      pending_issued_commands_.clear();
      expected_dispatch_sequence_ = 0;
      expected_dispatch_sequence_ready_ = true;
      command_ledger_valid_ =
        commands_match(dispatched, nav_2d_msgs::msg::Twist2D());
      dispatch_is_valid = command_ledger_valid_;
    } else {
      sequence_was_ready = expected_dispatch_sequence_ready_;
      expected_sequence = expected_dispatch_sequence_;
      const bool sequence_is_valid =
        expected_dispatch_sequence_ready_ &&
        message->sequence_id == expected_dispatch_sequence_;
      sequence_was_valid = sequence_is_valid;
      pending_command_count = pending_issued_commands_.size();
      for (const IssuedCommand & issued_command : pending_issued_commands_) {
        const double command_error = std::max({
            std::abs(issued_command.command.x - dispatched.x),
            std::abs(issued_command.command.y - dispatched.y),
            std::abs(issued_command.command.theta - dispatched.theta)});
        if (command_error < nearest_command_error) {
          nearest_command_error = command_error;
          nearest_pending_command = issued_command.command;
        }
      }
      eligible_pending_count = eligible_command_prefix_size(
        pending_issued_commands_, transport_received_steady_time_ns);
      const auto matching_index = find_eligible_command_index(
        pending_issued_commands_, transport_received_steady_time_ns, dispatched,
        kCommandMatchTolerance);
      const bool matches_pending_command = matching_index.has_value();
      if (matches_pending_command) {
        skipped_unpublished_command_count = *matching_index;
      }
      command_was_matched = matches_pending_command;
      // Collision Monitor is the only real-mode post-processor. It may reduce
      // a command after the plugin returns it. Accept that observable result
      // only when provenance remains unambiguous: the sequence is the next
      // expected one, exactly one FIFO entry is pending, no axis grows or
      // reverses direction, and the real-only launch opt-in is enabled.
      const bool is_accepted_safety_reduction =
        !matches_pending_command &&
        allow_safety_command_reduction_ &&
        sequence_is_valid &&
        eligible_pending_count == 1u &&
        is_safety_command_reduction(
        pending_issued_commands_.front().command, dispatched);
      safety_reduction_was_accepted = is_accepted_safety_reduction;
      // Controller Server may publish zero after NoValidControl without a
      // plugin return value to record. The transport sequence is authoritative:
      // accept only the zero that was actually applied, leave later issued
      // candidates pending, and update native dynamics from this dispatch.
      const bool is_ordered_external_stop =
        !matches_pending_command &&
        commands_match(dispatched, nav_2d_msgs::msg::Twist2D());
      ordered_external_stop = is_ordered_external_stop;
      const bool previous_dispatch_was_captured =
        std::abs(dispatched_command_.x) <=
        terminal_stop_velocity_threshold_ &&
        std::abs(dispatched_command_.theta) <=
        terminal_stop_velocity_threshold_;
      terminal_external_stop =
        is_ordered_external_stop &&
        terminal_stop_goal_capture_committed_ &&
        previous_dispatch_was_captured;
      dispatch_is_valid =
        sequence_is_valid &&
        (matches_pending_command || is_accepted_safety_reduction ||
        is_ordered_external_stop);
      if (dispatch_is_valid) {
        if (matches_pending_command || is_accepted_safety_reduction) {
          if (matches_pending_command) {
            pending_issued_commands_.erase(
              pending_issued_commands_.begin(),
              std::next(
                pending_issued_commands_.begin(),
                static_cast<std::ptrdiff_t>(*matching_index + 1u)));
          } else {
            pending_issued_commands_.pop_front();
          }
        } else if (terminal_external_stop) {
          // The action has ended; Controller Server cannot publish any of the
          // plugin results computed after the last applied native command.
          pending_issued_commands_.clear();
        }
        ++expected_dispatch_sequence_;
        command_ledger_valid_ = true;
      } else {
        pending_issued_commands_.clear();
        command_ledger_valid_ = false;
      }
    }
    if (dispatch_is_valid) {
      dispatched_command_ = dispatched;
      dispatched_command_time_ = transport_dispatched_at;
      dispatched_command_time_valid_ = true;
      command_dispatch_observed_ = true;
    }
  }

  if (dispatch_precedes_reset_boundary) {
    RCLCPP_DEBUG(
      logger_,
      "Ignoring command dispatch sequence=%" PRIu64
      " until the next explicit transport reset boundary",
      message->sequence_id);
    return;
  }

  if (!dispatch_is_valid) {
    RCLCPP_ERROR(
      logger_,
      "Command dispatch ledger mismatch: has_sequence=%s sequence=%" PRIu64
      " expected=%" PRIu64 " sequence_ready=%s sequence_valid=%s"
      " matched=%s pending=%zu eligible=%zu received_steady_ns=%" PRIu64
      " dispatched=(%.17g, %.17g, %.17g)"
      " nearest=(%.17g, %.17g, %.17g) max_error=%.17g",
      message->has_sequence ? "true" : "false",
      message->sequence_id,
      expected_sequence,
      sequence_was_ready ? "true" : "false",
      sequence_was_valid ? "true" : "false",
      command_was_matched ? "true" : "false",
      pending_command_count,
      eligible_pending_count,
      transport_received_steady_time_ns,
      dispatched.x,
      dispatched.y,
      dispatched.theta,
      nearest_pending_command.x,
      nearest_pending_command.y,
      nearest_pending_command.theta,
      nearest_command_error);
    request_transport_invalidation("command-dispatch ledger mismatch");
    return;
  }
  if (safety_reduction_was_accepted) {
    RCLCPP_WARN(
      logger_,
      "Robot-facing safety post-processor reduced command: sequence=%" PRIu64
      " issued=(%.17g, %.17g, %.17g) applied=(%.17g, %.17g, %.17g); "
      "the applied command is now the observable state",
      message->sequence_id,
      nearest_pending_command.x,
      nearest_pending_command.y,
      nearest_pending_command.theta,
      dispatched.x,
      dispatched.y,
      dispatched.theta);
  }
  if (skipped_unpublished_command_count > 0u) {
    RCLCPP_DEBUG(
      logger_,
      "Applied sequence=%" PRIu64 " matched after %zu Controller results "
      "that were not published to the command transport",
      message->sequence_id, skipped_unpublished_command_count);
  }
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    if (terminal_external_stop) {
      if (!native_generator->observe_terminal_controller_stop(*message)) {
        request_transport_invalidation(
          "unable to close committed terminal native state");
        return;
      }
    } else if (ordered_external_stop) {
      // This is an observed sequenced Controller Server zero, not a trial
      // reset. It can be dispatched after a later plugin result has already
      // been recorded, so insert its metadata before every pending result.
      // This preserves F-DWA history and reconstructs A/J acceleration from
      // the actually applied zero without discarding the following command.
      if (!native_generator->commit_observed_controller_stop_before_pending(
          rclcpp::Time(message->header.stamp)))
      {
        request_transport_invalidation(
          "unable to correlate Controller Server stop with native state");
        return;
      }
      native_generator->observe_command_dispatch(*message);
    } else {
      native_generator->observe_command_dispatch(
        *message, safety_reduction_was_accepted,
        skipped_unpublished_command_count);
    }
  }
}

void CertifiedDWBLocalPlanner::transport_valid_callback(
  const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> controller_lock(controller_state_mutex_);
  std::lock_guard<std::mutex> command_lock(command_state_mutex_);
  command_transport_valid_ = message->data;
  if (!message->data) {
    command_dispatch_observed_ = false;
    dispatched_command_time_valid_ = false;
    command_ledger_valid_ = false;
    expected_dispatch_sequence_ready_ = false;
    pending_issued_commands_.clear();
  }
}

void CertifiedDWBLocalPlanner::request_transport_invalidation(
  const char * reason)
{
  RCLCPP_ERROR(
    logger_, "Requesting transport_invalid: %s", reason);
  if (!transport_invalidation_client_ ||
    !transport_invalidation_client_->service_is_ready())
  {
    RCLCPP_ERROR(
      logger_,
      "Transport invalidation service is unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  transport_invalidation_client_->async_send_request(request);
}

std::shared_ptr<const PlanningSnapshot>
CertifiedDWBLocalPlanner::build_planning_snapshot(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & observed_velocity)
{
  PlanningSnapshot snapshot;
  snapshot.measurement_time = clock_->now();
  const double activation_preview_seconds =
    velocity_response_prediction_enabled_ ?
    velocity_response_prediction_seconds_ :
    (nominal_delay_preview_enabled_ ?
    nominal_delay_preview_seconds_ + planning_deadline_seconds_ : 0.0);
  snapshot.activation_time =
    snapshot.measurement_time +
    rclcpp::Duration::from_seconds(activation_preview_seconds);
  snapshot.current_state.pose =
    nav_2d_utils::poseStampedToPose2D(pose).pose;
  snapshot.current_state.activation_time = snapshot.measurement_time;

  std::deque<IssuedCommand> issued_commands;
  nav_2d_msgs::msg::Twist2D dispatched_command;
  rclcpp::Time dispatched_command_time(0, 0, RCL_ROS_TIME);
  bool dispatched_command_time_valid = false;
  const nav_2d_msgs::msg::Twist2D observed_velocity_2d =
    nav_2d_utils::twist3Dto2D(observed_velocity);
  const bool observed_velocity_is_finite =
    std::isfinite(observed_velocity_2d.x) &&
    std::isfinite(observed_velocity_2d.y) &&
    std::isfinite(observed_velocity_2d.theta);
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    snapshot.dispatch_state_observed = command_dispatch_observed_;
    snapshot.valid =
      command_transport_valid_ &&
      command_ledger_valid_ &&
      (command_dispatch_observed_ || !require_command_dispatch_state_);
    snapshot.current_state.velocity =
      use_observed_velocity_for_activation_state_ ?
      observed_velocity_2d : dispatched_command_;
    if (use_observed_velocity_for_activation_state_ &&
      !observed_velocity_is_finite)
    {
      snapshot.valid = false;
    }
    issued_commands = pending_issued_commands_;
    dispatched_command = dispatched_command_;
    dispatched_command_time = dispatched_command_time_;
    dispatched_command_time_valid = dispatched_command_time_valid_;
  }

  snapshot.current_state.native_command_velocity = dispatched_command;
  snapshot.current_state.native_command_velocity_valid =
    snapshot.dispatch_state_observed;

  geometry_msgs::msg::Pose2D rollout_pose = snapshot.current_state.pose;
  nav_2d_msgs::msg::Twist2D rollout_velocity =
    snapshot.current_state.velocity;
  rclcpp::Time rollout_time = snapshot.measurement_time;
  snapshot.delay_trajectory.push_back(rollout_pose);

  rclcpp::Time previous_activation = snapshot.measurement_time;
  const bool replay_pending_commands =
    !use_observed_velocity_for_activation_state_ ||
    snapshot.activation_time > snapshot.measurement_time;
  if (replay_pending_commands) {
    for (const IssuedCommand & issued : issued_commands) {
      rclcpp::Time activation_time =
        issued.issued_at +
        rclcpp::Duration::from_seconds(nominal_delay_preview_seconds_);
      if (activation_time < previous_activation) {
        activation_time = previous_activation;
      }
      previous_activation = activation_time;
      if (activation_time > snapshot.activation_time) {
        break;
      }
      snapshot.committed_commands.push_back(
        ScheduledCommand{
          activation_time, issued.command,
          issued.is_controller_failure_stop});
    }
  }

  nav_2d_msgs::msg::Twist2D activation_command_velocity =
    snapshot.current_state.native_command_velocity;
  for (const ScheduledCommand & scheduled : snapshot.committed_commands) {
    activation_command_velocity = scheduled.command;
  }

  if (velocity_response_prediction_enabled_) {
    const auto dispatch_age_seconds = dispatched_command_time_valid ?
      observed_dispatch_age_seconds(
      snapshot.measurement_time, dispatched_command_time,
      certification_control_period_) : std::nullopt;
    if (!dispatch_age_seconds) {
      snapshot.valid = false;
    } else {
      const VelocityResponsePrediction prediction =
        predict_velocity_response(
        rollout_pose, rollout_velocity, dispatched_command,
        *dispatch_age_seconds, velocity_response_prediction_seconds_,
        velocity_response_integration_step_seconds_,
        linear_velocity_response_model_, angular_velocity_response_model_);
      if (!prediction.valid) {
        snapshot.valid = false;
      } else {
        rollout_pose = prediction.pose;
        rollout_velocity = prediction.velocity;
        snapshot.delay_trajectory = prediction.trajectory;
      }
    }
    snapshot.activation_state = snapshot.current_state;
    snapshot.activation_state.pose = rollout_pose;
    snapshot.activation_state.velocity = rollout_velocity;
    snapshot.activation_state.native_command_velocity =
      activation_command_velocity;
    snapshot.activation_state.native_command_velocity_valid =
      snapshot.current_state.native_command_velocity_valid;
    snapshot.activation_state.activation_time = snapshot.activation_time;
    return std::make_shared<const PlanningSnapshot>(std::move(snapshot));
  }

  for (const ScheduledCommand & scheduled : snapshot.committed_commands) {
    const rclcpp::Time activation_time = scheduled.activation_time;
    if (activation_time > rollout_time) {
      append_integrated_motion(
        rollout_pose, rollout_velocity,
        (activation_time - rollout_time).seconds(),
        certification_control_period_, snapshot.delay_trajectory);
      rollout_time = activation_time;
    }
    rollout_velocity = scheduled.command;
  }
  if (snapshot.activation_time > rollout_time) {
    append_integrated_motion(
      rollout_pose, rollout_velocity,
      (snapshot.activation_time - rollout_time).seconds(),
      certification_control_period_, snapshot.delay_trajectory);
  }

  snapshot.activation_state = snapshot.current_state;
  snapshot.activation_state.pose = rollout_pose;
  snapshot.activation_state.velocity = rollout_velocity;
  snapshot.activation_state.native_command_velocity =
    activation_command_velocity;
  snapshot.activation_state.native_command_velocity_valid =
    snapshot.current_state.native_command_velocity_valid;
  snapshot.activation_state.activation_time = snapshot.activation_time;
  return std::make_shared<const PlanningSnapshot>(std::move(snapshot));
}

void CertifiedDWBLocalPlanner::record_issued_command(
  const geometry_msgs::msg::TwistStamped & command,
  const rclcpp::Time & issued_at,
  const uint64_t issued_steady_time_ns)
{
  const nav_2d_msgs::msg::Twist2D command_2d =
    nav_2d_utils::twist3Dto2D(command.twist);
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  const bool expected_terminal_stop =
    native_generator && terminal_stop_goal_capture_committed_ &&
    commands_match(command_2d, nav_2d_msgs::msg::Twist2D());
  bool ledger_overflow = false;
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    constexpr std::size_t kMaximumPendingCommandCount = 256;
    if (pending_issued_commands_.size() >= kMaximumPendingCommandCount) {
      command_transport_valid_ = false;
      pending_issued_commands_.clear();
      ledger_overflow = true;
    } else {
      pending_issued_commands_.push_back(
        IssuedCommand{
          issued_at, issued_steady_time_ns, command_2d,
          expected_terminal_stop});
    }
  }
  if (ledger_overflow) {
    request_transport_invalidation("local command ledger overflow");
    throw nav2_core::NoValidControl("Local command ledger overflow");
  }
  if (native_generator) {
    if (expected_terminal_stop) {
      // TerminalStoppedGoalHold is a controller policy result, not a sampled
      // native-input candidate.  Commit its exact zero as the explicit action
      // boundary state; treating it as a selected candidate invalidates the
      // native ledger whenever Nav2 evaluates one or more hold cycles before
      // StoppedGoalChecker completes.
      if (!native_generator->commit_expected_controller_stop(issued_at)) {
        request_transport_invalidation(
          "unable to commit terminal goal-hold native state");
        throw nav2_core::NoValidControl(
                "Unable to commit terminal goal-hold native state");
      }
    } else {
      native_generator->commit_selected_command(command_2d, issued_at);
    }
  }
}

void CertifiedDWBLocalPlanner::record_planning_duration(
  const std::chrono::steady_clock::time_point started_at)
{
  if (!planning_metrics_enabled_) {
    return;
  }
  const double duration_seconds =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_at).count();
  planning_durations_seconds_.push_back(duration_seconds);
  ++planning_cycle_count_;
  maximum_planning_duration_seconds_ =
    std::max(maximum_planning_duration_seconds_, duration_seconds);
  if (duration_seconds > planning_deadline_seconds_) {
    ++planning_deadline_miss_count_;
  }
  if (planning_cycle_count_ %
    static_cast<uint64_t>(planning_metrics_report_interval_) != 0u)
  {
    return;
  }
  report_planning_metrics("periodic");
}

void CertifiedDWBLocalPlanner::record_certification_rejection(
  const CertificationFailure failure)
{
  switch (failure) {
    case CertificationFailure::kInvalidInput:
      ++certification_rejections_.invalid_input;
      break;
    case CertificationFailure::kOffCostmap:
      ++certification_rejections_.off_costmap;
      break;
    case CertificationFailure::kLethalObstacle:
      ++certification_rejections_.lethal_obstacle;
      break;
    case CertificationFailure::kUnknownSpace:
      ++certification_rejections_.unknown_space;
      break;
    case CertificationFailure::kNone:
      break;
  }
}

void CertifiedDWBLocalPlanner::report_planning_metrics(
  const char * scope)
{
  if (!planning_metrics_enabled_ ||
    planning_durations_seconds_.empty())
  {
    return;
  }
  std::vector<double> sorted_durations = planning_durations_seconds_;
  std::sort(
    sorted_durations.begin(), sorted_durations.end());
  RCLCPP_INFO(
    logger_,
    "planning_timing scope=%s cycles=%" PRIu64
    " p50=%.6f p95=%.6f p99=%.6f "
    "max=%.6f deadline=%.6f misses=%" PRIu64,
    scope,
    planning_cycle_count_,
    quantile(sorted_durations, 0.50),
    quantile(sorted_durations, 0.95),
    quantile(sorted_durations, 0.99),
    maximum_planning_duration_seconds_,
    planning_deadline_seconds_,
    planning_deadline_miss_count_);
  if (certification_enabled_) {
    RCLCPP_INFO(
      logger_,
      "certificate_rejections scope=%s terminal_stop_infeasible=%" PRIu64
      " invalid_input=%" PRIu64 " off_costmap=%" PRIu64
      " lethal_obstacle=%" PRIu64 " unknown_space=%" PRIu64,
      scope,
      certification_rejections_.terminal_stop_infeasible,
      certification_rejections_.invalid_input,
      certification_rejections_.off_costmap,
      certification_rejections_.lethal_obstacle,
      certification_rejections_.unknown_space);
  }
  uint64_t publication_count = 0u;
  uint64_t full_evaluation_count = 0u;
  uint64_t candidate_marker_count = 0u;
  uint64_t deferred_full_evaluation_count = 0u;
  uint64_t coalesced_stale_marker_count = 0u;
  std::size_t pending_full_evaluation_count = 0u;
  std::size_t maximum_backlog = 0u;
  double maximum_publication_seconds = 0.0;
  {
    std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
    publication_count = diagnostic_publication_count_;
    full_evaluation_count = full_evaluation_publication_count_;
    candidate_marker_count = candidate_marker_publication_count_;
    deferred_full_evaluation_count = deferred_full_evaluation_count_;
    coalesced_stale_marker_count = coalesced_stale_marker_count_;
    pending_full_evaluation_count = pending_full_evaluation_count_;
    maximum_backlog = maximum_diagnostic_backlog_;
    maximum_publication_seconds =
      maximum_diagnostic_publication_seconds_;
  }
  RCLCPP_INFO(
    logger_,
    "diagnostic_timing scope=%s jobs=%" PRIu64
    " full_evaluations=%" PRIu64 " candidate_markers=%" PRIu64
    " deferred_full=%" PRIu64 " coalesced_stale_markers=%" PRIu64
    " pending_full=%zu maximum_backlog=%zu worker_max=%.6f",
    scope, publication_count, full_evaluation_count,
    candidate_marker_count, deferred_full_evaluation_count,
    coalesced_stale_marker_count, pending_full_evaluation_count,
    maximum_backlog,
    maximum_publication_seconds);
}

bool CertifiedDWBLocalPlanner::should_publish_evaluation()
{
  if (!publish_evaluation_) {
    return false;
  }
  const rclcpp::Time current_time = clock_->now();
  if (evaluation_publish_frequency_ > 0.0) {
    const double minimum_period = 1.0 / evaluation_publish_frequency_;
    if (has_evaluation_publish_time_ &&
      current_time >= last_evaluation_publish_time_ &&
      (current_time - last_evaluation_publish_time_).seconds() <
      minimum_period)
    {
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
    if (!has_full_evaluation_capacity(
        pending_full_evaluation_count_))
    {
      // Do not allocate another full payload while DDS is still handling the
      // previous samples. The timestamp is deliberately not advanced, so the
      // next control cycle captures the due sample as soon as a slot frees.
      ++deferred_full_evaluation_count_;
      return false;
    }
  }
  last_evaluation_publish_time_ = current_time;
  has_evaluation_publish_time_ = true;
  return true;
}

bool CertifiedDWBLocalPlanner::has_full_evaluation_capacity(
  const std::size_t pending_full_evaluation_count)
{
  return pending_full_evaluation_count <
         kMaximumPendingFullEvaluations;
}

uint64_t CertifiedDWBLocalPlanner::clearance_constraint_bucket(
  const double clearance_risk,
  const double admissible_risk,
  const double risk_resolution)
{
  if (!std::isfinite(clearance_risk) || clearance_risk < 0.0 ||
    clearance_risk > 1.0 || !std::isfinite(admissible_risk) ||
    admissible_risk < 0.0 || admissible_risk > 1.0 ||
    !is_positive_finite(risk_resolution))
  {
    throw std::invalid_argument{"invalid clearance epsilon constraint"};
  }
  if (clearance_risk <= admissible_risk) {
    return 0u;
  }
  const double violation_bands = std::ceil(
    (clearance_risk - admissible_risk) / risk_resolution - 1.0e-12);
  if (violation_bands >=
    static_cast<double>(std::numeric_limits<uint64_t>::max()))
  {
    return std::numeric_limits<uint64_t>::max();
  }
  return static_cast<uint64_t>(std::max(1.0, violation_bands));
}

double CertifiedDWBLocalPlanner::zero_scale_clearance_diagnostic_score(
  const bool is_clearance_constraint_critic,
  const bool is_clearance_constraint_trigger_critic,
  const bool is_clearance_constraint_guard_critic,
  const std::optional<double> precomputed_clearance_risk,
  const std::optional<double> precomputed_clearance_trigger_risk,
  const std::optional<double> precomputed_clearance_guard_risk)
{
  if (is_clearance_constraint_critic && precomputed_clearance_risk) {
    return *precomputed_clearance_risk;
  }
  if (is_clearance_constraint_trigger_critic &&
    precomputed_clearance_trigger_risk)
  {
    return *precomputed_clearance_trigger_risk;
  }
  if (is_clearance_constraint_guard_critic &&
    precomputed_clearance_guard_risk)
  {
    return *precomputed_clearance_guard_risk;
  }
  return 0.0;
}

bool CertifiedDWBLocalPlanner::should_score_terminal_stop(
  const bool certification_enabled,
  const bool stop_admissibility_enabled,
  const double goal_distance_scale,
  const bool target_pose_valid)
{
  return (certification_enabled || stop_admissibility_enabled) &&
         goal_distance_scale > 0.0 && target_pose_valid;
}

std::optional<double> CertifiedDWBLocalPlanner::fused_clearance_risk(
  const std::optional<double> primary_risk,
  const std::optional<double> guard_risk)
{
  if (primary_risk && guard_risk) {
    return std::max(*primary_risk, *guard_risk);
  }
  return primary_risk ? primary_risk : guard_risk;
}

bool CertifiedDWBLocalPlanner::trajectory_has_meaningful_subgoal_progress(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const nav_2d_msgs::msg::Path2D & path,
  const geometry_msgs::msg::Pose2D & subgoal,
  const double minimum_distance_progress,
  const double minimum_heading_progress)
{
  return trajectory_has_meaningful_path_progress(
    trajectory, path, subgoal, minimum_distance_progress,
    minimum_heading_progress);
}

bool CertifiedDWBLocalPlanner::terminal_plan_fallback_is_applicable(
  const geometry_msgs::msg::Pose2D & pose,
  const geometry_msgs::msg::Pose2D & terminal_pose,
  const double capture_distance)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(terminal_pose.x) &&
         std::isfinite(terminal_pose.y) &&
         std::isfinite(capture_distance) && capture_distance > 0.0 &&
         std::hypot(
    pose.x - terminal_pose.x,
    pose.y - terminal_pose.y) <= capture_distance + 1.0e-12;
}

bool CertifiedDWBLocalPlanner::terminal_goal_hold_is_applicable(
  const geometry_msgs::msg::Pose2D & pose,
  const geometry_msgs::msg::Pose2D & goal_pose,
  const double capture_distance,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const double stop_velocity_threshold)
{
  return terminal_plan_fallback_is_applicable(
    pose, goal_pose, capture_distance) &&
         is_positive_finite(stop_velocity_threshold) &&
         std::isfinite(velocity.x) && std::isfinite(velocity.y) &&
         std::isfinite(velocity.theta) &&
         std::abs(velocity.x) <= stop_velocity_threshold &&
         std::abs(velocity.y) <= stop_velocity_threshold &&
         std::abs(velocity.theta) <= stop_velocity_threshold;
}

bool CertifiedDWBLocalPlanner::clearance_constraint_prefers_candidate(
  const bool candidate_has_meaningful_progress,
  const bool best_has_meaningful_progress,
  const uint64_t candidate_guard_risk_bucket,
  const uint64_t best_guard_risk_bucket,
  const uint64_t candidate_risk_bucket,
  const uint64_t best_risk_bucket,
  const double candidate_total,
  const double best_total,
  const std::size_t candidate_canonical_index,
  const std::size_t best_canonical_index)
{
  if (candidate_guard_risk_bucket != best_guard_risk_bucket) {
    return candidate_guard_risk_bucket < best_guard_risk_bucket;
  }
  if (candidate_risk_bucket != best_risk_bucket) {
    return candidate_risk_bucket < best_risk_bucket;
  }
  if (candidate_total != best_total) {
    return candidate_total < best_total;
  }
  if (candidate_has_meaningful_progress != best_has_meaningful_progress) {
    return candidate_has_meaningful_progress;
  }
  return candidate_canonical_index < best_canonical_index;
}

bool CertifiedDWBLocalPlanner::clearance_constraint_is_active_for_pair(
  const bool clearance_constraint_enabled,
  const double best_total,
  const uint64_t candidate_trigger_risk_bucket,
  const uint64_t best_trigger_risk_bucket)
{
  return clearance_constraint_enabled && best_total >= 0.0 &&
         (candidate_trigger_risk_bucket > 0u ||
         best_trigger_risk_bucket > 0u);
}

CertifiedDWBLocalPlanner::TerminalStopAssessment
CertifiedDWBLocalPlanner::assess_terminal_stop(
  const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
  const geometry_msgs::msg::Pose2D & goal_pose,
  const double terminal_path_heading,
  const double capture_distance,
  const double capture_yaw_tolerance,
  const double maximum_overshoot)
{
  TerminalStopAssessment assessment;
  if (stop_poses.empty() || !std::isfinite(goal_pose.x) ||
    !std::isfinite(goal_pose.y) || !std::isfinite(goal_pose.theta) ||
    !std::isfinite(terminal_path_heading) ||
    !std::isfinite(capture_distance) || capture_distance < 0.0 ||
    !std::isfinite(capture_yaw_tolerance) ||
    capture_yaw_tolerance < 0.0 || !std::isfinite(maximum_overshoot) ||
    maximum_overshoot < 0.0)
  {
    return assessment;
  }

  assessment.terminal_pose = stop_poses.back();
  if (!std::isfinite(assessment.terminal_pose.x) ||
    !std::isfinite(assessment.terminal_pose.y) ||
    !std::isfinite(assessment.terminal_pose.theta))
  {
    return assessment;
  }
  const double delta_x = assessment.terminal_pose.x - goal_pose.x;
  const double delta_y = assessment.terminal_pose.y - goal_pose.y;
  const double tangent_x = std::cos(terminal_path_heading);
  const double tangent_y = std::sin(terminal_path_heading);
  assessment.available = true;
  assessment.position_error = std::hypot(delta_x, delta_y);
  assessment.yaw_error = std::abs(std::remainder(
      assessment.terminal_pose.theta - goal_pose.theta, 2.0 * M_PI));
  assessment.longitudinal_error =
    delta_x * tangent_x + delta_y * tangent_y;
  assessment.lateral_error =
    -delta_x * tangent_y + delta_y * tangent_x;
  assessment.captures_goal =
    assessment.position_error <= capture_distance &&
    assessment.yaw_error <= capture_yaw_tolerance;
  // The terminal tangent defines the endpoint ordering only inside the Goal
  // capture corridor. Applying its infinite half-plane to the entire plan can
  // reject an earlier leg of a curved or returning path as already beyond the
  // endpoint, even when that leg is metres away laterally.
  assessment.crosses_terminal_limit =
    std::abs(assessment.lateral_error) <= capture_distance + 1.0e-12 &&
    assessment.longitudinal_error > maximum_overshoot + 1.0e-12;
  return assessment;
}

bool CertifiedDWBLocalPlanner::terminal_stop_prefers_candidate(
  const bool candidate_captures_goal,
  const bool best_captures_goal,
  const double candidate_total,
  const double best_total,
  const std::size_t candidate_canonical_index,
  const std::size_t best_canonical_index)
{
  if (candidate_captures_goal != best_captures_goal) {
    return candidate_captures_goal;
  }
  if (candidate_total != best_total) {
    return candidate_total < best_total;
  }
  return candidate_canonical_index < best_canonical_index;
}

bool CertifiedDWBLocalPlanner::receding_horizon_recovery_prefers_candidate(
  const double candidate_collision_time,
  const double best_collision_time,
  const double candidate_clearance_risk,
  const double best_clearance_risk,
  const double candidate_path_departure_cost,
  const double best_path_departure_cost,
  const std::size_t candidate_canonical_index,
  const std::size_t best_canonical_index)
{
  if (candidate_canonical_index == std::numeric_limits<std::size_t>::max() ||
    std::isnan(candidate_collision_time) ||
    candidate_collision_time < 0.0 ||
    !std::isfinite(candidate_clearance_risk) ||
    candidate_clearance_risk < 0.0 ||
    !std::isfinite(candidate_path_departure_cost) ||
    candidate_path_departure_cost < 0.0)
  {
    return false;
  }
  if (best_canonical_index == std::numeric_limits<std::size_t>::max()) {
    return true;
  }
  if (candidate_collision_time != best_collision_time) {
    return candidate_collision_time > best_collision_time;
  }
  if (candidate_clearance_risk != best_clearance_risk) {
    return candidate_clearance_risk < best_clearance_risk;
  }
  if (candidate_path_departure_cost != best_path_departure_cost) {
    return candidate_path_departure_cost < best_path_departure_cost;
  }
  return candidate_canonical_index < best_canonical_index;
}

double CertifiedDWBLocalPlanner::predicted_collision_time(
  const CertificationResult & result,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const double maximum_swept_distance,
  const double control_period)
{
  if (result.safe) {
    return std::numeric_limits<double>::infinity();
  }
  if (!result.has_failure_pose || poses.empty() || footprint.size() < 3u ||
    !is_positive_finite(maximum_swept_distance) ||
    !is_positive_finite(control_period))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (result.failure_source_pose_index == 0u) {
    return 0.0;
  }
  const std::size_t source_index = result.failure_source_pose_index;
  if (source_index >= poses.size() ||
    result.failure_interpolation_index == 0u)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double maximum_footprint_radius = 0.0;
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    maximum_footprint_radius = std::max(
      maximum_footprint_radius, std::hypot(point.x, point.y));
  }
  const auto & previous = poses[source_index - 1u];
  const auto & next = poses[source_index];
  const double angle_difference = std::atan2(
    std::sin(next.theta - previous.theta),
    std::cos(next.theta - previous.theta));
  const double swept_distance =
    std::hypot(next.x - previous.x, next.y - previous.y) +
    maximum_footprint_radius * std::abs(angle_difference);
  if (!std::isfinite(swept_distance)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t subdivisions = static_cast<std::size_t>(
    std::max(1.0, std::ceil(swept_distance / maximum_swept_distance)));
  if (result.failure_interpolation_index > subdivisions) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double interpolation_fraction =
    static_cast<double>(result.failure_interpolation_index) /
    static_cast<double>(subdivisions);
  return (static_cast<double>(source_index - 1u) + interpolation_fraction) *
         control_period;
}

double CertifiedDWBLocalPlanner::predicted_collision_time_from_obstacle_rejection(
  const std::string & rejection_detail,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const double maximum_swept_distance,
  const double control_period)
{
  if (poses.empty() || footprint.size() < 3u ||
    !is_positive_finite(maximum_swept_distance) ||
    !is_positive_finite(control_period))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto parse_index = [&rejection_detail](
    const std::string_view field) -> std::optional<std::size_t>
    {
      const std::size_t field_position = rejection_detail.find(field);
      if (field_position == std::string::npos) {
        return std::nullopt;
      }
      const char * first = rejection_detail.data() + field_position +
        field.size();
      const char * last = rejection_detail.data() + rejection_detail.size();
      std::size_t value = 0u;
      const auto result = std::from_chars(first, last, value);
      if (result.ec != std::errc() ||
        (result.ptr != last && *result.ptr != ';'))
      {
        return std::nullopt;
      }
      return value;
    };
  const auto pose_index = parse_index(";pose_index=");
  const auto subdivision_index = parse_index(";subdivision=");
  if (!pose_index || !subdivision_index || *pose_index >= poses.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (*pose_index == 0u) {
    return *subdivision_index == 0u ?
           0.0 : std::numeric_limits<double>::quiet_NaN();
  }
  if (*subdivision_index == 0u) {
    return static_cast<double>(*pose_index) * control_period;
  }

  double maximum_footprint_radius = 0.0;
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    maximum_footprint_radius = std::max(
      maximum_footprint_radius, std::hypot(point.x, point.y));
  }
  const auto & previous = poses[*pose_index - 1u];
  const auto & next = poses[*pose_index];
  const double angle_difference = std::atan2(
    std::sin(next.theta - previous.theta),
    std::cos(next.theta - previous.theta));
  const double swept_distance =
    std::hypot(next.x - previous.x, next.y - previous.y) +
    maximum_footprint_radius * std::abs(angle_difference);
  if (!std::isfinite(swept_distance)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t subdivisions = static_cast<std::size_t>(
    std::max(1.0, std::ceil(swept_distance / maximum_swept_distance)));
  if (*subdivision_index >= subdivisions) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double interpolation_fraction =
    static_cast<double>(*subdivision_index) /
    static_cast<double>(subdivisions);
  return (static_cast<double>(*pose_index - 1u) + interpolation_fraction) *
         control_period;
}

bool CertifiedDWBLocalPlanner::coalesce_stale_marker_publication(
  std::deque<DiagnosticPublication> & publications,
  DiagnosticPublication publication)
{
  if (publication.publish_full_evaluation) {
    return false;
  }
  const auto stale = std::find_if(
    publications.begin(), publications.end(),
    [](const DiagnosticPublication & queued) {
      return !queued.publish_full_evaluation;
    });
  if (stale == publications.end()) {
    return false;
  }
  publications.erase(stale);
  publications.push_back(std::move(publication));
  return true;
}

void CertifiedDWBLocalPlanner::enqueue_diagnostic_publication(
  const std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & evaluation,
  const bool publish_full_evaluation,
  const bool publish_candidate_markers)
{
  if (!evaluation) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
    if (stop_diagnostic_publisher_) {
      return;
    }
    DiagnosticPublication publication{
      evaluation, publish_full_evaluation, publish_candidate_markers};
    if (coalesce_stale_marker_publication(
        diagnostic_publications_, publication))
    {
      ++coalesced_stale_marker_count_;
      diagnostic_publication_condition_.notify_one();
      return;
    }
    if (publish_full_evaluation) {
      // should_publish_evaluation() is the only producer-side admission path,
      // so a full sample always has one of these two reserved practical slots.
      if (!has_full_evaluation_capacity(
          pending_full_evaluation_count_))
      {
        RCLCPP_ERROR(
          logger_,
          "Full diagnostic admission invariant was violated");
        return;
      }
      ++pending_full_evaluation_count_;
    }
    diagnostic_publications_.push_back(std::move(publication));
    maximum_diagnostic_backlog_ = std::max(
      maximum_diagnostic_backlog_, diagnostic_publications_.size());
  }
  diagnostic_publication_condition_.notify_one();
}

void CertifiedDWBLocalPlanner::start_diagnostic_publisher()
{
  stop_diagnostic_publisher();
  {
    std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
    stop_diagnostic_publisher_ = false;
  }
  diagnostic_publisher_thread_ = std::thread(
    &CertifiedDWBLocalPlanner::diagnostic_publisher_loop, this);
}

void CertifiedDWBLocalPlanner::stop_diagnostic_publisher()
{
  {
    std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
    stop_diagnostic_publisher_ = true;
  }
  diagnostic_publication_condition_.notify_all();
  if (diagnostic_publisher_thread_.joinable()) {
    diagnostic_publisher_thread_.join();
  }
}

void CertifiedDWBLocalPlanner::diagnostic_publisher_loop()
{
  while (true) {
    DiagnosticPublication publication;
    {
      std::unique_lock<std::mutex> lock(diagnostic_publication_mutex_);
      diagnostic_publication_condition_.wait(
        lock,
        [this]() {
          return stop_diagnostic_publisher_ ||
                 !diagnostic_publications_.empty();
        });
      if (diagnostic_publications_.empty()) {
        if (stop_diagnostic_publisher_) {
          return;
        }
        continue;
      }
      publication = std::move(diagnostic_publications_.front());
      diagnostic_publications_.pop_front();
    }
    const auto publication_started_at = std::chrono::steady_clock::now();
    try {
      publish_diagnostics_now(publication);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger_, "Asynchronous diagnostic publication failed: %s",
        exception.what());
    } catch (...) {
      RCLCPP_ERROR(
        logger_, "Asynchronous diagnostic publication failed");
    }
    const double publication_seconds =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - publication_started_at).count();
    {
      std::lock_guard<std::mutex> lock(diagnostic_publication_mutex_);
      ++diagnostic_publication_count_;
      if (publication.publish_full_evaluation) {
        ++full_evaluation_publication_count_;
      }
      if (publication.publish_candidate_markers) {
        ++candidate_marker_publication_count_;
      }
      if (publication.publish_full_evaluation &&
        pending_full_evaluation_count_ > 0u)
      {
        --pending_full_evaluation_count_;
      }
      maximum_diagnostic_publication_seconds_ = std::max(
        maximum_diagnostic_publication_seconds_, publication_seconds);
    }
  }
}

void CertifiedDWBLocalPlanner::publish_diagnostics_now(
  const DiagnosticPublication & publication)
{
  if (!publication.evaluation) {
    return;
  }
  if (publication.publish_full_evaluation && evaluation_publisher_ &&
    evaluation_publisher_->is_activated())
  {
    evaluation_publisher_->publish(*publication.evaluation);
  }
  if (publication.publish_candidate_markers &&
    candidate_marker_publisher_ &&
    candidate_marker_publisher_->is_activated())
  {
    publish_candidate_markers(*publication.evaluation);
  }
}

bool CertifiedDWBLocalPlanner::should_publish_candidate_markers() const
{
  return publish_candidate_markers_ && candidate_marker_publisher_ &&
         candidate_marker_publisher_->is_activated() &&
         candidate_marker_publisher_->get_subscription_count() > 0u;
}

void CertifiedDWBLocalPlanner::publish_candidate_markers(
  const dwb_msgs::msg::LocalPlanEvaluation & evaluation)
{
  candidate_marker_publisher_->publish(
    build_candidate_markers(evaluation));
}

visualization_msgs::msg::MarkerArray
CertifiedDWBLocalPlanner::build_candidate_markers(
  const dwb_msgs::msg::LocalPlanEvaluation & evaluation) const
{
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::Marker valid;
  valid.header = evaluation.header;
  valid.ns = "dwb_candidates_valid";
  valid.id = 0;
  valid.type = visualization_msgs::msg::Marker::LINE_LIST;
  valid.action = visualization_msgs::msg::Marker::ADD;
  valid.pose.orientation.w = 1.0;
  valid.scale.x = 0.018;
  valid.color.r = 0.20F;
  valid.color.g = 0.64F;
  valid.color.b = 1.0F;
  valid.color.a = 0.28F;
  valid.lifetime.sec = 0;
  valid.lifetime.nanosec = 150000000u;

  visualization_msgs::msg::Marker rejected = valid;
  rejected.ns = "dwb_candidates_rejected";
  rejected.id = 1;
  rejected.color.r = 1.0F;
  rejected.color.g = 0.23F;
  rejected.color.b = 0.19F;
  rejected.color.a = 0.48F;

  visualization_msgs::msg::Marker best = valid;
  best.ns = "dwb_candidate_selected";
  best.id = 2;
  best.type = visualization_msgs::msg::Marker::LINE_STRIP;
  best.scale.x = 0.060;
  best.color.r = 0.20F;
  best.color.g = 0.84F;
  best.color.b = 0.29F;
  best.color.a = 0.96F;

  visualization_msgs::msg::Marker status = valid;
  status.ns = "dwb_candidate_status_realtime";
  status.id = 3;
  status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status.scale.x = 0.0;
  status.scale.y = 0.0;
  status.scale.z = 0.16;
  status.color.a = 0.96F;

  const int best_index = evaluation.best_index;
  std::vector<double> ranked_valid_costs;
  ranked_valid_costs.reserve(evaluation.twists.size());
  for (std::size_t index = 0u; index < evaluation.twists.size(); ++index) {
    const auto & score = evaluation.twists[index];
    if (static_cast<int>(index) != best_index &&
      std::isfinite(score.total) && score.total >= 0.0)
    {
      ranked_valid_costs.push_back(static_cast<double>(score.total));
    }
  }
  std::sort(ranked_valid_costs.begin(), ranked_valid_costs.end());

  std::size_t valid_count = 0u;
  std::map<std::string, std::size_t> rejection_counts;
  bool status_position_set = false;
  bool selected_receding_horizon_recovery = false;
  for (std::size_t index = 0; index < evaluation.twists.size(); ++index) {
    const auto & score = evaluation.twists[index];
    const auto & poses = score.traj.poses;
    if (poses.size() < 2u) {
      continue;
    }
    const bool legal = std::isfinite(score.total) && score.total >= 0.0;
    if (legal) {
      ++valid_count;
    } else if (!score.scores.empty()) {
      const auto rejection = std::find_if(
        score.scores.rbegin(), score.scores.rend(),
        [](const dwb_msgs::msg::CriticScore & critic_score) {
          return critic_score.raw_score < 0.0 &&
                 critic_score.name.rfind("__", 0u) != 0u;
        });
      ++rejection_counts[
        rejection == score.scores.rend() ?
        std::string("unknown") : rejection->name];
    }
    if (!status_position_set && !poses.empty()) {
      status.pose.position.x = poses.front().x;
      status.pose.position.y = poses.front().y;
      status.pose.position.z = 0.45;
      status_position_set = true;
    }
    const bool selected = legal && static_cast<int>(index) == best_index;
    if (selected) {
      selected_receding_horizon_recovery = std::any_of(
        score.scores.begin(), score.scores.end(),
        [](const dwb_msgs::msg::CriticScore & critic_score) {
          return critic_score.name == "RecedingHorizonRecovery";
        });
      best.points.reserve(poses.size());
      for (const auto & pose : poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        point.z = 0.04;
        best.points.push_back(point);
      }
      continue;
    }
    auto & points = legal ? valid.points : rejected.points;
    auto * colors = legal ? &valid.colors : nullptr;
    const auto color = legal ?
      weighted_cost_color(
      normalized_cost_rank(score.total, ranked_valid_costs)) :
      std_msgs::msg::ColorRGBA();
    points.reserve(points.size() + 2u * (poses.size() - 1u));
    if (colors) {
      colors->reserve(colors->size() + 2u * (poses.size() - 1u));
    }
    for (std::size_t pose_index = 1; pose_index < poses.size(); ++pose_index) {
      const auto & first = poses[pose_index - 1u];
      const auto & second = poses[pose_index];
      if (std::hypot(second.x - first.x, second.y - first.y) <= 1.0e-5) {
        continue;
      }
      geometry_msgs::msg::Point first_point;
      first_point.x = first.x;
      first_point.y = first.y;
      first_point.z = 0.02;
      geometry_msgs::msg::Point second_point;
      second_point.x = second.x;
      second_point.y = second.y;
      second_point.z = 0.02;
      points.push_back(first_point);
      points.push_back(second_point);
      if (colors) {
        colors->push_back(color);
        colors->push_back(color);
      }
    }
  }
  if (selected_receding_horizon_recovery) {
    status.color.r = 1.0F;
    status.color.g = 0.64F;
    status.color.b = 0.10F;
    status.text =
      "No legal full horizon: verified one-step recovery";
  } else if (valid_count == 0u) {
    const auto dominant = std::max_element(
      rejection_counts.begin(), rejection_counts.end(),
      [](const auto & first, const auto & second) {
        return first.second < second.second;
      });
    status.color.r = 1.0F;
    status.color.g = 0.23F;
    status.color.b = 0.19F;
    status.text = "No legal trajectory: " +
      (dominant == rejection_counts.end() ? std::string("unknown") : dominant->first);
  } else {
    status.color.r = 0.20F;
    status.color.g = 0.84F;
    status.color.b = 0.29F;
    status.text = "DWB candidates: " + std::to_string(valid_count) + "/" +
      std::to_string(evaluation.twists.size()) + " valid";
  }
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(5u);
  markers.markers.push_back(std::move(clear));
  markers.markers.push_back(std::move(valid));
  markers.markers.push_back(std::move(rejected));
  markers.markers.push_back(std::move(best));
  markers.markers.push_back(std::move(status));
  return markers;
}

void CertifiedDWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  terminal_stop_goal_capture_active_ = false;
  terminal_stop_goal_capture_committed_ = false;
  has_evaluation_publish_time_ = false;
  current_goal_pose_valid_ = false;
  current_terminal_path_heading_valid_ = false;
  current_terminal_distance_target_pose_valid_ = false;
  current_progress_reference_plan_.poses.clear();
  dwb_core::DWBLocalPlanner::setPlan(path);
  // DWB prunes global_plan_ in place. Preserve the submitted reference so the
  // braking endpoint remains observable after the robot passes or departs from
  // the transformed local segment.
  terminal_reference_plan_ = global_plan_;
}

nav_2d_msgs::msg::Path2D CertifiedDWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  try {
    nav_2d_msgs::msg::Path2D transformed_plan =
      dwb_core::DWBLocalPlanner::transformGlobalPlan(pose);
    current_progress_reference_plan_ = transformed_plan;
    return transformed_plan;
  } catch (const nav2_core::InvalidPath &) {
    if (terminal_reference_plan_.poses.empty()) {
      throw;
    }

    nav_2d_msgs::msg::Pose2DStamped plan_frame_pose;
    if (!nav_2d_utils::transformPose(
        tf_, terminal_reference_plan_.header.frame_id, pose,
        plan_frame_pose, transform_tolerance_))
    {
      throw;
    }
    const geometry_msgs::msg::Pose2D & terminal_pose =
      terminal_reference_plan_.poses.back();
    if (!terminal_plan_fallback_is_applicable(
        plan_frame_pose.pose, terminal_pose,
        terminal_stop_goal_capture_distance_))
    {
      throw;
    }

    // Goal position is already admissible, but StoppedGoalChecker may need
    // another cycle for measured velocity to settle. Restore only the saved
    // endpoint so normal DWB scoring can complete that deceleration instead
    // of aborting after the regular lookahead-limited prune removes all poses.
    global_plan_.header = terminal_reference_plan_.header;
    global_plan_.poses.assign(1u, terminal_pose);
    nav_2d_msgs::msg::Path2D transformed_plan =
      dwb_core::DWBLocalPlanner::transformGlobalPlan(pose);
    current_progress_reference_plan_ = transformed_plan;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "Restored the terminal Path pose while StoppedGoalChecker settles; "
      "position_error=%.3f m, capture_distance=%.3f m",
      std::hypot(
        plan_frame_pose.pose.x - terminal_pose.x,
        plan_frame_pose.pose.y - terminal_pose.y),
      terminal_stop_goal_capture_distance_);
    return transformed_plan;
  }
}

void CertifiedDWBLocalPlanner::reset()
{
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  terminal_stop_goal_capture_active_ = false;
  terminal_stop_goal_capture_committed_ = false;
  planning_snapshot_.reset();
  has_evaluation_publish_time_ = false;
  current_goal_pose_valid_ = false;
  terminal_reference_plan_.poses.clear();
  current_terminal_path_heading_valid_ = false;
  current_terminal_distance_target_pose_valid_ = false;
  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }
  traj_generator_->reset();
}

void CertifiedDWBLocalPlanner::prepare_terminal_targets(
  const geometry_msgs::msg::Pose2D & pose)
{
  current_goal_pose_valid_ = false;
  current_terminal_path_heading_valid_ = false;
  current_terminal_distance_target_pose_valid_ = false;
  if (terminal_reference_plan_.poses.empty()) {
    return;
  }

  nav_2d_msgs::msg::Pose2DStamped goal_pose;
  goal_pose.header.frame_id = terminal_reference_plan_.header.frame_id;
  goal_pose.header.stamp = terminal_reference_plan_.header.stamp;
  goal_pose.pose = terminal_reference_plan_.poses.back();
  nav_2d_msgs::msg::Pose2DStamped transformed_goal;
  if (nav_2d_utils::transformPose(
      tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
      transformed_goal, transform_tolerance_))
  {
    current_goal_pose_ = transformed_goal.pose;
    current_goal_pose_valid_ = true;
  }

  if (current_goal_pose_valid_) {
    for (std::size_t reverse_index = terminal_reference_plan_.poses.size();
      reverse_index > 1u; --reverse_index)
    {
      const auto & previous =
        terminal_reference_plan_.poses[reverse_index - 2u];
      const auto & terminal = terminal_reference_plan_.poses.back();
      if (std::hypot(
          terminal.x - previous.x, terminal.y - previous.y) <= 1.0e-6)
      {
        continue;
      }
      nav_2d_msgs::msg::Pose2DStamped previous_pose;
      previous_pose.header = terminal_reference_plan_.header;
      previous_pose.pose = previous;
      nav_2d_msgs::msg::Pose2DStamped transformed_previous;
      if (nav_2d_utils::transformPose(
          tf_, costmap_ros_->getGlobalFrameID(), previous_pose,
          transformed_previous, transform_tolerance_))
      {
        current_terminal_path_heading_ = std::atan2(
          current_goal_pose_.y - transformed_previous.pose.y,
          current_goal_pose_.x - transformed_previous.pose.x);
        current_terminal_path_heading_valid_ = true;
      }
      break;
    }
    if (!current_terminal_path_heading_valid_ &&
      std::isfinite(current_goal_pose_.theta))
    {
      current_terminal_path_heading_ = current_goal_pose_.theta;
      current_terminal_path_heading_valid_ = true;
    }
  }

  if (terminal_stop_score_mode_ ==
    TerminalStopScoreMode::kGlobalGoalDistance)
  {
    if (current_goal_pose_valid_) {
      current_terminal_distance_target_pose_ = current_goal_pose_;
      current_terminal_distance_target_pose_valid_ = true;
    }
    return;
  }

  nav_2d_msgs::msg::Pose2DStamped costmap_pose;
  costmap_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
  costmap_pose.header.stamp = terminal_reference_plan_.header.stamp;
  costmap_pose.pose = pose;
  nav_2d_msgs::msg::Pose2DStamped plan_frame_pose;
  if (!nav_2d_utils::transformPose(
      tf_, terminal_reference_plan_.header.frame_id, costmap_pose,
      plan_frame_pose, transform_tolerance_))
  {
    return;
  }

  geometry_msgs::msg::Pose2D path_subgoal;
  if (!compute_path_subgoal(
      terminal_reference_plan_, plan_frame_pose.pose,
      terminal_stop_path_lookahead_distance_, path_subgoal))
  {
    return;
  }
  nav_2d_msgs::msg::Pose2DStamped stamped_subgoal;
  stamped_subgoal.header = terminal_reference_plan_.header;
  stamped_subgoal.pose = path_subgoal;
  nav_2d_msgs::msg::Pose2DStamped transformed_subgoal;
  if (nav_2d_utils::transformPose(
      tf_, costmap_ros_->getGlobalFrameID(), stamped_subgoal,
      transformed_subgoal, transform_tolerance_))
  {
    current_terminal_distance_target_pose_ = transformed_subgoal.pose;
    current_terminal_distance_target_pose_valid_ = true;
  }
}

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (certification_enabled_ || stop_admissibility_enabled_) {
    prepare_certification_broadphase(
      *costmap_ros_->getCostmap(), certification_workspace_);
  }
  prepare_terminal_targets(pose);
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  const bool jerk_guard_progress_recovery_enabled =
    std::dynamic_pointer_cast<JerkTrajectoryGenerator>(traj_generator_) !=
    nullptr;
  const bool terminal_stop_policy_enabled =
    native_generator && terminal_stop_goal_capture_distance_ > 0.0 &&
    terminal_stop_goal_capture_yaw_tolerance_ > 0.0;
  const bool terminal_endpoint_policy_enabled =
    terminal_stop_policy_enabled && current_goal_pose_valid_ &&
    current_terminal_path_heading_valid_;
  if ((certification_enabled_ || stop_admissibility_enabled_) &&
    terminal_stop_goal_distance_scale_ > 0.0 &&
    !current_terminal_distance_target_pose_valid_)
  {
    throw nav2_core::ControllerTFError(
            "Unable to prepare terminal-stop distance target");
  }
  // Keep the scoring loop local even for Nav2's velocity generators. Besides
  // preserving the exact DWB ordering and total, this lets marker-only cycles
  // omit CriticScore payloads; delegating to base DWB would allocate the full
  // diagnostic graph merely because RViz subscribes to candidate markers.

  // A feasible stop that has entered the goal capture set is a committed
  // policy. Ordinary recovery dispatches only one verified native response
  // and therefore always replans from fresh obstacle data on the next cycle.
  if (terminal_stop_goal_capture_active_) {
    dwb_msgs::msg::TrajectoryScore backup_score;
    if (build_revalidated_backup(pose, backup_score)) {
      if (results) {
        results->twists.push_back(backup_score);
        results->best_index = results->twists.size() - 1u;
      }
      retained_backup_commands_.erase(
        retained_backup_commands_.begin());
      if (!retained_backup_states_.empty()) {
        retained_backup_states_.erase(retained_backup_states_.begin());
      }
      if (retained_backup_commands_.empty()) {
        terminal_stop_goal_capture_active_ = false;
      }
      return backup_score;
    }
    terminal_stop_goal_capture_active_ = false;
    terminal_stop_goal_capture_committed_ = false;
  }

  if (terminal_stop_policy_enabled && current_goal_pose_valid_ &&
    terminal_goal_hold_is_applicable(
      pose, current_goal_pose_, terminal_stop_goal_capture_distance_,
      velocity, terminal_stop_velocity_threshold_))
  {
    const std::vector<geometry_msgs::msg::Pose2D> stationary_pose{pose};
    if (certify_physical_sequence(stationary_pose, nullptr, nullptr)) {
      terminal_stop_goal_capture_committed_ = true;
      dwb_msgs::msg::TrajectoryScore hold_score;
      hold_score.total = 0.0;
      hold_score.traj.velocity = nav_2d_msgs::msg::Twist2D();
      hold_score.traj.poses = stationary_pose;
      if (record_full_evaluation_details_) {
        dwb_msgs::msg::CriticScore hold_detail;
        hold_detail.name = "TerminalStoppedGoalHold";
        hold_detail.scale = 0.0;
        hold_detail.raw_score = 1.0;
        hold_score.scores.push_back(std::move(hold_detail));
      }
      if (results) {
        results->twists.push_back(hold_score);
        results->best_index = results->twists.size() - 1u;
      }
      return hold_score;
    }
  }

  dwb_msgs::msg::TrajectoryScore best;
  best.total = -1.0;
  double worst_total = -1.0;
  std::size_t best_canonical_index =
    std::numeric_limits<std::size_t>::max();
  std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
  best_command_state;
  bool best_uses_reserve_recovery = false;
  bool best_has_meaningful_progress = false;
  bool best_stop_has_translation_progress = false;
  bool best_stop_has_heading_progress = false;
  bool best_stop_progress_was_evaluated = false;
  bool best_captures_goal = false;
  struct ProgressEscapeCandidate
  {
    bool found{false};
    std::size_t canonical_index{std::numeric_limits<std::size_t>::max()};
    std::size_t evaluation_index{std::numeric_limits<std::size_t>::max()};
    uint64_t clearance_risk_bucket{std::numeric_limits<uint64_t>::max()};
    uint64_t clearance_trigger_bucket{0u};
    uint64_t clearance_guard_bucket{std::numeric_limits<uint64_t>::max()};
    uint64_t mean_path_distance_bucket{std::numeric_limits<uint64_t>::max()};
    double total{-1.0};
    double path_deviation_cost{std::numeric_limits<double>::infinity()};
    double mean_path_distance_cost{std::numeric_limits<double>::infinity()};
    double progress_cost{std::numeric_limits<double>::infinity()};
    double avoidance_horizon_seconds{0.0};
    bool has_translation_progress{false};
    bool has_heading_progress{false};
    bool captures_goal{false};
    bool uses_reserve_recovery{false};
    std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
    command_state;
    dwb_msgs::msg::Trajectory2D trajectory;
    std::vector<geometry_msgs::msg::Pose2D> stop_poses;
  } progress_escape;
  const bool inside_terminal_capture_region =
    current_goal_pose_valid_ && terminal_plan_fallback_is_applicable(
    pose, current_goal_pose_, terminal_stop_goal_capture_distance_);
  struct RejectedRecoveryCandidate
  {
    std::size_t canonical_index{0u};
    std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
    command_state;
    std::optional<double> clearance_risk;
    std::optional<double> primary_clearance_risk;
    std::optional<double> trigger_clearance_risk;
    std::optional<double> guard_clearance_risk;
    std::optional<double> collision_time;
    bool obstacle_footprint_rejection{false};
  };
  std::vector<RejectedRecoveryCandidate> rejected_recovery_candidates;
  const bool collect_recovery_candidates =
    no_valid_control_deceleration_fallback_enabled_ &&
    !inside_terminal_capture_region;
  if (collect_recovery_candidates) {
    rejected_recovery_candidates.reserve(
      native_generator ? native_generator->candidate_count() : 0u);
  }
  std::vector<nav_2d_msgs::msg::Twist2D> best_stop_velocities;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
  best_stop_states;
  dwb_core::IllegalTrajectoryTracker tracker;
  std::size_t evaluation_index = 0u;
  dwb_msgs::msg::Trajectory2D trajectory_scratch;
  dwb_msgs::msg::TrajectoryScore score_scratch;
  std::vector<geometry_msgs::msg::Pose2D> candidate_stop_poses;
  uint64_t best_clearance_risk_bucket =
    std::numeric_limits<uint64_t>::max();
  uint64_t best_clearance_trigger_bucket = 0u;
  uint64_t best_clearance_guard_bucket =
    std::numeric_limits<uint64_t>::max();
  const bool clearance_progress_preference_enabled =
    clearance_constraint_enabled_ && current_goal_pose_valid_ &&
    current_terminal_distance_target_pose_valid_ &&
    (clearance_constraint_minimum_subgoal_distance_progress_ > 0.0 ||
    clearance_constraint_minimum_subgoal_heading_progress_ > 0.0) &&
    std::hypot(
      pose.x - current_goal_pose_.x,
      pose.y - current_goal_pose_.y) >
    clearance_constraint_motion_preference_goal_distance_;
  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    const nav_2d_msgs::msg::Twist2D twist =
      traj_generator_->nextTwist();
    if (native_generator) {
      native_generator->generate_trajectory_into(
        pose, twist, trajectory_scratch);
    } else {
      trajectory_scratch =
        traj_generator_->generateTrajectory(pose, velocity, twist);
    }
    const std::size_t canonical_index =
      native_generator ?
      native_generator->active_candidate_canonical_index().value_or(
      std::numeric_limits<std::size_t>::max()) :
      evaluation_index;
    ++evaluation_index;
    std::optional<double> precomputed_clearance_risk;
    std::optional<double> precomputed_clearance_trigger_risk;
    std::optional<double> precomputed_clearance_guard_risk;
    try {
      TerminalStopAssessment terminal_stop_assessment;
      candidate_stop_poses.clear();
      if (terminal_endpoint_policy_enabled || stop_admissibility_enabled_ ||
        certification_enabled_)
      {
        if (!build_stop_trajectory(
            trajectory_scratch, candidate_stop_poses, nullptr, nullptr,
            canonical_index))
        {
          throw dwb_core::IllegalTrajectoryException(
                  terminal_endpoint_policy_enabled ?
                  "TerminalGoalAdmissibility" :
                  (stop_admissibility_enabled_ ?
                  "StopAdmissibility" : "SafetyCertificate"),
                  "No dynamically feasible terminal stop sequence");
        }
        if (terminal_endpoint_policy_enabled) {
          terminal_stop_assessment = assess_terminal_stop(
            candidate_stop_poses, current_goal_pose_,
            current_terminal_path_heading_,
            terminal_stop_goal_capture_distance_,
            terminal_stop_goal_capture_yaw_tolerance_,
            terminal_stop_goal_capture_distance_);
          if (!terminal_stop_assessment.available) {
            throw dwb_core::IllegalTrajectoryException(
                    "TerminalGoalAdmissibility",
                    "Terminal stop endpoint is not finite");
          }
        }
      }
      uint64_t candidate_clearance_risk_bucket = 0u;
      uint64_t candidate_clearance_trigger_bucket = 0u;
      uint64_t candidate_clearance_guard_bucket = 0u;
      double footprint_approach_risk = 0.0;
      double clearance_guard_approach_risk = 0.0;
      if (clearance_constraint_enabled_) {
        const double clearance_risk =
          clearance_constraint_critic_->scoreTrajectoryWithApproachRisk(
          trajectory_scratch, &footprint_approach_risk);
        if (!std::isfinite(clearance_risk) || clearance_risk < 0.0 ||
          clearance_risk > 1.0 + 1.0e-9)
        {
          throw dwb_core::IllegalTrajectoryException(
                  "ClearanceConstraint",
                  "Footprint clearance risk is outside [0, 1]");
        }
        precomputed_clearance_risk = std::clamp(clearance_risk, 0.0, 1.0);
        candidate_clearance_risk_bucket = clearance_constraint_bucket(
          *precomputed_clearance_risk,
          clearance_constraint_admissible_risk_,
          clearance_constraint_risk_resolution_);
        const double configured_trigger_risk =
          clearance_constraint_trigger_critic_.get() ==
          clearance_constraint_critic_.get() ?
          *precomputed_clearance_risk :
          clearance_constraint_trigger_critic_->scoreTrajectory(
          trajectory_scratch);
        if (!std::isfinite(configured_trigger_risk) ||
          configured_trigger_risk < 0.0 ||
          configured_trigger_risk > 1.0 + 1.0e-9 ||
          !std::isfinite(footprint_approach_risk) ||
          footprint_approach_risk < 0.0 ||
          footprint_approach_risk > 1.0 + 1.0e-9)
        {
          throw dwb_core::IllegalTrajectoryException(
                  "ClearanceConstraint",
                  "Clearance trigger risk is outside [0, 1]");
        }
        precomputed_clearance_trigger_risk =
          std::clamp(configured_trigger_risk, 0.0, 1.0);
        const double trigger_risk =
          clearance_constraint_trigger_risk_ >= 0.0 ?
          clearance_constraint_trigger_risk_ :
          clearance_constraint_admissible_risk_;
        candidate_clearance_trigger_bucket = clearance_constraint_bucket(
          *precomputed_clearance_trigger_risk,
          trigger_risk,
          clearance_constraint_risk_resolution_);
        double clearance_guard_risk = 0.0;
        const bool guard_uses_primary_clearance =
          clearance_constraint_guard_critic_.get() ==
          clearance_constraint_critic_.get();
        const bool guard_uses_trigger_clearance =
          clearance_constraint_guard_critic_.get() ==
          clearance_constraint_trigger_critic_.get();
        const bool guard_has_footprint_clearance =
          clearance_constraint_include_footprint_approach_ &&
          clearance_constraint_guard_footprint_critic_;
        if (guard_uses_primary_clearance) {
          clearance_guard_risk = *precomputed_clearance_risk;
          clearance_guard_approach_risk = footprint_approach_risk;
        } else if (guard_uses_trigger_clearance) {
          clearance_guard_risk = *precomputed_clearance_trigger_risk;
          if (clearance_constraint_include_footprint_approach_ &&
            clearance_constraint_guard_footprint_critic_)
          {
            clearance_guard_risk =
              candidate_stop_poses.empty() ?
              clearance_constraint_guard_footprint_critic_->
              scoreTrajectoryWithApproachRisk(
              trajectory_scratch, &clearance_guard_approach_risk) :
              clearance_constraint_guard_footprint_critic_->
              scoreUniformPoseSequenceWithApproachRisk(
              candidate_stop_poses, &clearance_guard_approach_risk);
          }
        } else if (guard_has_footprint_clearance) {
          clearance_guard_risk =
            candidate_stop_poses.empty() ?
            clearance_constraint_guard_footprint_critic_->
            scoreTrajectoryWithApproachRisk(
            trajectory_scratch, &clearance_guard_approach_risk) :
            clearance_constraint_guard_footprint_critic_->
            scoreUniformPoseSequenceWithApproachRisk(
            candidate_stop_poses, &clearance_guard_approach_risk);
        } else {
          clearance_guard_risk =
            clearance_constraint_guard_critic_->scoreTrajectory(
            trajectory_scratch);
        }
        if (!std::isfinite(clearance_guard_risk) ||
          clearance_guard_risk < 0.0 ||
          clearance_guard_risk > 1.0 + 1.0e-9)
        {
          throw dwb_core::IllegalTrajectoryException(
                  "ClearanceConstraint",
                  "Clearance guard risk is outside [0, 1]");
        }
        precomputed_clearance_guard_risk =
          std::clamp(clearance_guard_risk, 0.0, 1.0);
        candidate_clearance_guard_bucket = clearance_constraint_bucket(
          *precomputed_clearance_guard_risk,
          clearance_constraint_guard_admissible_risk_,
          clearance_constraint_guard_risk_resolution_);
        if (clearance_constraint_include_footprint_approach_) {
          const double maximum_footprint_approach_risk = std::max(
            footprint_approach_risk, clearance_guard_approach_risk);
          footprint_approach_risk = maximum_footprint_approach_risk;
          candidate_clearance_trigger_bucket = std::max(
            candidate_clearance_trigger_bucket,
            clearance_constraint_bucket(
              std::clamp(maximum_footprint_approach_risk, 0.0, 1.0),
              clearance_constraint_footprint_approach_trigger_risk_,
              clearance_constraint_risk_resolution_));
        }
      }
      const bool avoidance_constraint_context =
        clearance_constraint_is_active_for_pair(
        clearance_constraint_enabled_, best.total,
        candidate_clearance_trigger_bucket,
        best_clearance_trigger_bucket);
      bool candidate_has_meaningful_progress = false;
      const bool evaluate_stop_progress =
        clearance_progress_preference_enabled &&
        (candidate_clearance_trigger_bucket > 0u ||
        (jerk_guard_progress_recovery_enabled &&
        candidate_clearance_guard_bucket > 0u)) &&
        !candidate_stop_poses.empty();
      const bool candidate_stop_has_translation_progress =
        evaluate_stop_progress &&
        pose_sequence_has_meaningful_path_progress(
        candidate_stop_poses, current_progress_reference_plan_,
        current_terminal_distance_target_pose_,
        clearance_constraint_minimum_subgoal_distance_progress_,
        clearance_constraint_minimum_subgoal_heading_progress_);
      const bool candidate_stop_has_heading_progress =
        evaluate_stop_progress &&
        pose_sequence_has_meaningful_path_progress(
        candidate_stop_poses, current_progress_reference_plan_,
        current_terminal_distance_target_pose_, 0.0,
        clearance_constraint_minimum_subgoal_heading_progress_);
      // From rest, one V-DWA command followed immediately by its verified
      // stop can move less than the configured half-cell progress threshold.
      // Use the method's ordinary finite-horizon rollout only to establish
      // the direction of a receding-horizon escape.  The command remains
      // eligible only after its separate one-step-plus-stop sequence has
      // passed the unchanged physical-footprint gate below.
      const bool candidate_rollout_has_translation_progress =
        evaluate_stop_progress &&
        !candidate_stop_has_translation_progress &&
        trajectory_has_meaningful_subgoal_progress(
        trajectory_scratch, current_progress_reference_plan_,
        current_terminal_distance_target_pose_,
        clearance_constraint_minimum_subgoal_distance_progress_,
        clearance_constraint_minimum_subgoal_heading_progress_);
      const bool candidate_has_translation_progress =
        candidate_stop_has_translation_progress ||
        candidate_rollout_has_translation_progress;
      const bool candidate_rollout_has_heading_progress =
        evaluate_stop_progress &&
        !candidate_has_translation_progress &&
        !candidate_stop_has_heading_progress &&
        trajectory_has_meaningful_subgoal_progress(
        trajectory_scratch, current_progress_reference_plan_,
        current_terminal_distance_target_pose_, 0.0,
        clearance_constraint_minimum_subgoal_heading_progress_);
      const bool candidate_has_heading_progress =
        candidate_stop_has_heading_progress ||
        candidate_rollout_has_heading_progress;
      const bool candidate_has_receding_horizon_progress =
        candidate_has_translation_progress ||
        candidate_has_heading_progress;
      double candidate_best_score = clearance_constraint_score_limit(
        avoidance_constraint_context,
        candidate_has_meaningful_progress, best_has_meaningful_progress,
        candidate_clearance_guard_bucket,
        best_clearance_guard_bucket, candidate_clearance_risk_bucket,
        best_clearance_risk_bucket, best_uses_reserve_recovery, best.total);
      if (terminal_endpoint_policy_enabled && best.total >= 0.0 &&
        terminal_stop_assessment.captures_goal != best_captures_goal)
      {
        // Goal capture is an admissibility class, not another critic weight.
        // Fully evaluate a capturing candidate and short-circuit one that
        // cannot displace an already capturing candidate.
        candidate_best_score = terminal_stop_assessment.captures_goal ?
          -1.0 : std::numeric_limits<double>::min();
      }
      if (terminal_endpoint_policy_enabled &&
        terminal_stop_assessment.crosses_terminal_limit)
      {
        // Complete the normal hard checks before reporting an endpoint
        // rejection, preserving the physical-footprint gate's precedence.
        candidate_best_score = -1.0;
      }
      bool candidate_uses_reserve_recovery = false;
      bool completed_weighted_score = true;
      double candidate_clearance_risk = 0.0;
      double weighted_path_deviation_cost =
        std::numeric_limits<double>::quiet_NaN();
      double weighted_mean_path_distance_cost =
        std::numeric_limits<double>::quiet_NaN();
      bool strict_physical_stop_safety_proven = false;
      score_trajectory_components(
        trajectory_scratch,
        candidate_best_score,
        score_scratch, record_full_evaluation_details_,
        &candidate_uses_reserve_recovery, precomputed_clearance_risk,
        precomputed_clearance_trigger_risk,
        precomputed_clearance_guard_risk,
        &candidate_clearance_risk,
        candidate_stop_poses.empty() ? nullptr : &candidate_stop_poses,
        &completed_weighted_score, &weighted_path_deviation_cost,
        &weighted_mean_path_distance_cost,
        &strict_physical_stop_safety_proven);
      (void)candidate_clearance_risk;
      if (record_full_evaluation_details_ &&
        clearance_constraint_include_footprint_approach_)
      {
        dwb_msgs::msg::CriticScore approach_detail;
        approach_detail.name = "__footprint_approach__";
        approach_detail.scale = 0.0;
        approach_detail.raw_score = footprint_approach_risk;
        score_scratch.scores.push_back(std::move(approach_detail));
      }
      if (record_full_evaluation_details_ && !completed_weighted_score) {
        dwb_msgs::msg::CriticScore short_circuit_detail;
        short_circuit_detail.name = "__short_circuit__";
        short_circuit_detail.scale = 0.0;
        short_circuit_detail.raw_score = 1.0;
        score_scratch.scores.push_back(std::move(short_circuit_detail));
      }
      if (terminal_endpoint_policy_enabled &&
        terminal_stop_assessment.crosses_terminal_limit)
      {
        std::ostringstream detail;
        detail << std::setprecision(17) <<
          "predicted_stop_beyond_path_end" <<
          ";goal_x=" << current_goal_pose_.x <<
          ";goal_y=" << current_goal_pose_.y <<
          ";terminal_x=" << terminal_stop_assessment.terminal_pose.x <<
          ";terminal_y=" << terminal_stop_assessment.terminal_pose.y <<
          ";longitudinal_error=" <<
          terminal_stop_assessment.longitudinal_error <<
          ";lateral_error=" << terminal_stop_assessment.lateral_error <<
          ";maximum_overshoot=" << terminal_stop_goal_capture_distance_;
        throw dwb_core::IllegalTrajectoryException(
                "TerminalGoalAdmissibility", detail.str());
      }
      if (record_full_evaluation_details_ &&
        terminal_stop_assessment.available)
      {
        std::ostringstream detail;
        detail << std::setprecision(17) <<
          "__terminal_stop__:capture=" <<
          (terminal_stop_assessment.captures_goal ? "true" : "false") <<
          ";position_error=" << terminal_stop_assessment.position_error <<
          ";yaw_error=" << terminal_stop_assessment.yaw_error <<
          ";longitudinal_error=" <<
          terminal_stop_assessment.longitudinal_error <<
          ";lateral_error=" << terminal_stop_assessment.lateral_error <<
          ";terminal_x=" << terminal_stop_assessment.terminal_pose.x <<
          ";terminal_y=" << terminal_stop_assessment.terminal_pose.y;
        dwb_msgs::msg::CriticScore terminal_detail;
        terminal_detail.name = detail.str();
        terminal_detail.scale = 0.0;
        terminal_detail.raw_score = 0.0;
        score_scratch.scores.push_back(std::move(terminal_detail));
      }
      const bool recovery_mode_matches =
        candidate_uses_reserve_recovery == best_uses_reserve_recovery;
      const bool terminal_capture_rank_is_better =
        terminal_endpoint_policy_enabled && recovery_mode_matches &&
        terminal_stop_assessment.captures_goal != best_captures_goal &&
        terminal_stop_assessment.captures_goal;
      const bool terminal_capture_class_matches =
        !terminal_endpoint_policy_enabled ||
        terminal_stop_assessment.captures_goal == best_captures_goal;
      const bool progress_tie_break_is_needed =
        clearance_progress_preference_enabled && best.total >= 0.0 &&
        avoidance_constraint_context && recovery_mode_matches &&
        terminal_capture_class_matches &&
        candidate_clearance_guard_bucket == best_clearance_guard_bucket &&
        candidate_clearance_risk_bucket == best_clearance_risk_bucket &&
        score_scratch.total == best.total;
      if (progress_tie_break_is_needed) {
        candidate_has_meaningful_progress =
          trajectory_has_meaningful_subgoal_progress(
          trajectory_scratch, current_progress_reference_plan_,
          current_terminal_distance_target_pose_,
          clearance_constraint_minimum_subgoal_distance_progress_,
          clearance_constraint_minimum_subgoal_heading_progress_);
        best_has_meaningful_progress =
          trajectory_has_meaningful_subgoal_progress(
          best.traj, current_progress_reference_plan_,
          current_terminal_distance_target_pose_,
          clearance_constraint_minimum_subgoal_distance_progress_,
          clearance_constraint_minimum_subgoal_heading_progress_);
      }
      const bool avoidance_rank_is_better =
        avoidance_constraint_context && recovery_mode_matches &&
        terminal_capture_class_matches &&
        clearance_constraint_prefers_candidate(
        candidate_has_meaningful_progress, best_has_meaningful_progress,
        candidate_clearance_guard_bucket, best_clearance_guard_bucket,
        candidate_clearance_risk_bucket, best_clearance_risk_bucket,
        score_scratch.total, best.total,
        canonical_index, best_canonical_index);
      const bool normal_rank_is_better =
        !avoidance_constraint_context && recovery_mode_matches &&
        terminal_stop_prefers_candidate(
        terminal_stop_assessment.captures_goal, best_captures_goal,
        score_scratch.total, best.total,
        canonical_index, best_canonical_index);
      const bool is_best =
        best.total < 0.0 ||
        (!candidate_uses_reserve_recovery &&
        best_uses_reserve_recovery) ||
        terminal_capture_rank_is_better || avoidance_rank_is_better ||
        normal_rank_is_better;
      bool is_worst = worst_total < 0.0;
      if (!is_worst) {
        is_worst = score_scratch.total > worst_total;
      }
      const double score_total = score_scratch.total;
      tracker.addLegalTrajectory();
      double candidate_path_deviation_cost = 0.0;
      double candidate_mean_path_distance_cost = 0.0;
      double candidate_progress_cost = std::numeric_limits<double>::infinity();
      const double candidate_avoidance_horizon_seconds =
        candidate_has_translation_progress ?
        std::numeric_limits<double>::infinity() : 0.0;
      bool candidate_is_progress_escape = false;
      if (candidate_has_receding_horizon_progress) {
        bool path_deviation_score_is_available =
          std::isfinite(weighted_path_deviation_cost);
        bool mean_path_distance_score_is_available =
          std::isfinite(weighted_mean_path_distance_cost);
        candidate_path_deviation_cost = weighted_path_deviation_cost;
        candidate_mean_path_distance_cost =
          weighted_mean_path_distance_cost;
        for (const auto & critic : critics_) {
          const bool is_path_deviation_critic =
            critic->getName() == "PathDeviation";
          const bool is_mean_path_distance_critic =
            critic->getName() == "MeanPathDist";
          const bool should_score_mean_path_distance =
            !mean_path_distance_score_is_available &&
            is_mean_path_distance_critic;
          if (!path_deviation_score_is_available &&
            is_path_deviation_critic)
          {
            candidate_path_deviation_cost =
              critic->scoreTrajectory(trajectory_scratch) * critic->getScale();
            path_deviation_score_is_available = true;
          } else if (should_score_mean_path_distance) {
            candidate_mean_path_distance_cost =
              critic->scoreTrajectory(trajectory_scratch) * critic->getScale();
            mean_path_distance_score_is_available = true;
          }
          if (path_deviation_score_is_available &&
            mean_path_distance_score_is_available)
          {
            break;
          }
        }
        const bool progress_is_proven_by_stop =
          candidate_stop_has_translation_progress ||
          candidate_stop_has_heading_progress;
        const auto & progress_terminal_pose = progress_is_proven_by_stop ?
          candidate_stop_poses.back() : trajectory_scratch.poses.back();
        const double progress_terminal_heading_error =
          std::abs(std::remainder(
            current_terminal_distance_target_pose_.theta -
            progress_terminal_pose.theta, 2.0 * M_PI));
        candidate_progress_cost = candidate_has_translation_progress ?
          std::hypot(
          path_subgoal_forward_ray_cost(
            progress_terminal_pose, current_terminal_distance_target_pose_,
            terminal_stop_path_lateral_weight_),
          terminal_stop_path_lookahead_distance_ *
          progress_terminal_heading_error) :
          progress_terminal_heading_error;
        candidate_is_progress_escape =
          std::isfinite(candidate_path_deviation_cost) &&
          candidate_path_deviation_cost >= 0.0 &&
          std::isfinite(candidate_mean_path_distance_cost) &&
          candidate_mean_path_distance_cost >= 0.0 &&
          std::isfinite(candidate_progress_cost) &&
          candidate_progress_cost >= 0.0;
      }
      const bool progress_escape_rank_is_better = [&]() {
          if (!progress_escape.found) {
            return true;
          }
          if (jerk_guard_progress_recovery_enabled &&
            candidate_clearance_guard_bucket !=
            progress_escape.clearance_guard_bucket)
          {
            return candidate_clearance_guard_bucket <
                   progress_escape.clearance_guard_bucket;
          }
          if (candidate_has_translation_progress !=
            progress_escape.has_translation_progress)
          {
            return candidate_has_translation_progress;
          }
          if (candidate_avoidance_horizon_seconds !=
            progress_escape.avoidance_horizon_seconds)
          {
            return candidate_avoidance_horizon_seconds >
                   progress_escape.avoidance_horizon_seconds;
          }
          if (candidate_clearance_guard_bucket !=
            progress_escape.clearance_guard_bucket)
          {
            return candidate_clearance_guard_bucket <
                   progress_escape.clearance_guard_bucket;
          }
          if (candidate_clearance_risk_bucket !=
            progress_escape.clearance_risk_bucket)
          {
            return candidate_clearance_risk_bucket <
                   progress_escape.clearance_risk_bucket;
          }
          if (candidate_path_deviation_cost !=
            progress_escape.path_deviation_cost)
          {
            return candidate_path_deviation_cost <
                   progress_escape.path_deviation_cost;
          }
          if (candidate_progress_cost != progress_escape.progress_cost) {
            return candidate_progress_cost < progress_escape.progress_cost;
          }
          const uint64_t candidate_mean_path_distance_bucket =
            static_cast<uint64_t>(candidate_mean_path_distance_cost);
          if (candidate_mean_path_distance_bucket !=
            progress_escape.mean_path_distance_bucket)
          {
            return candidate_mean_path_distance_bucket <
                   progress_escape.mean_path_distance_bucket;
          }
          if (candidate_mean_path_distance_cost !=
            progress_escape.mean_path_distance_cost)
          {
            return candidate_mean_path_distance_cost <
                   progress_escape.mean_path_distance_cost;
          }
          return canonical_index < progress_escape.canonical_index;
        }();
      // Every candidate reaching this point has already passed the hard
      // ObstacleFootprint rollout critic. Weighted short-circuiting can still
      // skip its later StopAdmissibility critic, so explicitly certify only
      // that missing stop suffix for candidates improving the retained rank.
      // Preserve J-DWA's method-specific static-jerk guard first. For V/F,
      // prefer certified translation over a lower soft-clearance bucket so a
      // safe bypass cannot lose indefinitely to in-place heading motion.
      const bool progress_motion_is_strictly_physical_safe =
        progress_escape_rank_is_better && candidate_is_progress_escape &&
        (strict_physical_stop_safety_proven || certify_pose_sequence(
        *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
        candidate_stop_poses, maximum_swept_distance_,
        &certification_workspace_).safe);
      if (progress_motion_is_strictly_physical_safe) {
        progress_escape.found = true;
        progress_escape.canonical_index = canonical_index;
        progress_escape.evaluation_index = results ?
          results->twists.size() : std::numeric_limits<std::size_t>::max();
        progress_escape.clearance_risk_bucket =
          candidate_clearance_risk_bucket;
        progress_escape.clearance_trigger_bucket =
          candidate_clearance_trigger_bucket;
        progress_escape.clearance_guard_bucket =
          candidate_clearance_guard_bucket;
        progress_escape.mean_path_distance_bucket =
          static_cast<uint64_t>(candidate_mean_path_distance_cost);
        progress_escape.total = score_scratch.total;
        progress_escape.path_deviation_cost =
          candidate_path_deviation_cost;
        progress_escape.mean_path_distance_cost =
          candidate_mean_path_distance_cost;
        progress_escape.progress_cost = candidate_progress_cost;
        progress_escape.avoidance_horizon_seconds =
          candidate_avoidance_horizon_seconds;
        progress_escape.has_translation_progress =
          candidate_has_translation_progress;
        progress_escape.has_heading_progress =
          candidate_has_heading_progress;
        progress_escape.captures_goal = terminal_stop_assessment.captures_goal;
        progress_escape.uses_reserve_recovery =
          candidate_uses_reserve_recovery;
        progress_escape.command_state = native_generator ?
          native_generator->active_candidate_command_state() : std::nullopt;
        progress_escape.trajectory = trajectory_scratch;
        progress_escape.stop_poses = candidate_stop_poses;
      }
      if (results) {
        score_scratch.traj = trajectory_scratch;
        // A short-circuited total is a valid lower bound on the final weighted
        // cost. Preserve it for diagnostic color ranking instead of mapping
        // every pruned candidate to the same artificial maximum.
        results->twists.push_back(score_scratch);
        score_scratch.total = score_total;
      }
      if (is_best) {
        if (native_generator) {
          best_command_state =
            native_generator->active_candidate_command_state();
        }
        if (results) {
          best = score_scratch;
        } else {
          best.total = score_scratch.total;
          best.traj.velocity = trajectory_scratch.velocity;
          best.traj.poses.swap(trajectory_scratch.poses);
          best.traj.time_offsets.swap(trajectory_scratch.time_offsets);
        }
        best_canonical_index = canonical_index;
        best_clearance_risk_bucket = candidate_clearance_risk_bucket;
        best_clearance_trigger_bucket = candidate_clearance_trigger_bucket;
        best_clearance_guard_bucket = candidate_clearance_guard_bucket;
        best_has_meaningful_progress = candidate_has_meaningful_progress;
        best_stop_has_translation_progress =
          candidate_stop_has_translation_progress;
        best_stop_has_heading_progress =
          candidate_stop_has_heading_progress;
        best_stop_progress_was_evaluated = evaluate_stop_progress;
        best_captures_goal = terminal_stop_assessment.captures_goal;
        best_uses_reserve_recovery =
          candidate_uses_reserve_recovery;
        if (results) {
          results->best_index = results->twists.size() - 1u;
        }
      }
      if (is_worst) {
        worst_total = score_total;
        if (results) {
          results->worst_index = results->twists.size() - 1u;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & exception) {
      if (collect_recovery_candidates) {
        const double parsed_collision_time =
          exception.getCriticName() == "ObstacleFootprint" ?
          predicted_collision_time_from_obstacle_rejection(
          exception.what(), trajectory_scratch.poses,
          costmap_ros_->getRobotFootprint(), maximum_swept_distance_,
          certification_control_period_) :
          std::numeric_limits<double>::quiet_NaN();
        rejected_recovery_candidates.push_back(
          RejectedRecoveryCandidate{
            canonical_index,
            native_generator ?
            native_generator->active_candidate_command_state() :
            std::nullopt,
            fused_clearance_risk(
              precomputed_clearance_risk,
              precomputed_clearance_guard_risk),
            precomputed_clearance_risk,
            precomputed_clearance_trigger_risk,
            precomputed_clearance_guard_risk,
            std::isnan(parsed_collision_time) ?
            std::nullopt : std::optional<double>{parsed_collision_time},
            exception.getCriticName() == "ObstacleFootprint"});
      }
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = trajectory_scratch;
        dwb_msgs::msg::CriticScore critic_score;
        critic_score.name = exception.getCriticName();
        critic_score.raw_score = -1.0;
        failed_score.scores.push_back(critic_score);
        if (record_full_evaluation_details_ && native_generator) {
          dwb_msgs::msg::CriticScore rejection_detail;
          rejection_detail.name =
            std::string{"__rejection_detail__:"} + exception.what();
          rejection_detail.raw_score = 0.0;
          rejection_detail.scale = 0.0;
          failed_score.scores.push_back(std::move(rejection_detail));
          const auto diagnostics =
            native_generator->active_candidate_diagnostics();
          if (diagnostics) {
            dwb_msgs::msg::CriticScore candidate_detail;
            candidate_detail.name =
              std::string{"__candidate_native__:"} +
            candidate_diagnostic_metadata(*diagnostics);
            candidate_detail.raw_score = 0.0;
            candidate_detail.scale = 0.0;
            failed_score.scores.push_back(std::move(candidate_detail));
          }
        }
        failed_score.total = -1.0;
        results->twists.push_back(std::move(failed_score));
      }
      tracker.addIllegalTrajectory(exception);
    }
  }

  const bool native_window_needs_direct_stop =
    native_generator && best.total < 0.0 &&
    no_valid_control_deceleration_fallback_enabled_;
  if (native_window_needs_direct_stop) {
    const int maximum_stop_steps = static_cast<int>(std::ceil(
        terminal_stop_maximum_time_ / certification_control_period_));
    std::vector<geometry_msgs::msg::Pose2D> direct_stop_poses;
    std::vector<nav_2d_msgs::msg::Twist2D> direct_stop_velocities;
    std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
    direct_stop_states;
    CertificationFailure direct_stop_failure =
      CertificationFailure::kInvalidInput;
    if (native_generator->generate_direct_stop_trajectory(
        pose, maximum_stop_steps, terminal_stop_velocity_threshold_,
        direct_stop_poses, direct_stop_velocities, direct_stop_states) &&
      !direct_stop_velocities.empty() && !direct_stop_states.empty() &&
      certify_stop_poses(direct_stop_poses, direct_stop_failure))
    {
      dwb_msgs::msg::TrajectoryScore direct_stop_score;
      direct_stop_score.total = 0.0;
      direct_stop_score.traj.velocity = direct_stop_velocities.front();
      direct_stop_score.traj.poses = direct_stop_poses;
      dwb_msgs::msg::CriticScore stop_detail;
      stop_detail.name = "EmptyNativeWindowDirectStop";
      stop_detail.scale = 0.0;
      stop_detail.raw_score = 1.0;
      direct_stop_score.scores.push_back(std::move(stop_detail));
      native_generator->select_command_for_dispatch(direct_stop_states.front());
      retained_backup_commands_.assign(
        direct_stop_velocities.begin() + 1, direct_stop_velocities.end());
      retained_backup_states_.assign(
        std::make_move_iterator(direct_stop_states.begin() + 1),
        std::make_move_iterator(direct_stop_states.end()));
      terminal_stop_goal_capture_active_ = false;
      terminal_stop_goal_capture_committed_ = false;
      if (results) {
        results->twists.push_back(direct_stop_score);
        results->best_index = results->twists.size() - 1u;
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Native full-horizon window has no legal response; "
        "dispatching a certified method-native direct-stop command");
      return direct_stop_score;
    }
  }

  const double stopped_native_window_hold_distance =
    terminal_stop_goal_capture_distance_ +
    maximum_linear_velocity_ * certification_control_period_;
  const bool empty_native_window_is_stopped_near_goal =
    native_generator && native_generator->candidate_count() == 0u &&
    terminal_stop_policy_enabled && current_goal_pose_valid_ &&
    terminal_goal_hold_is_applicable(
    pose, current_goal_pose_, stopped_native_window_hold_distance,
    velocity, terminal_stop_velocity_threshold_);
  if (empty_native_window_is_stopped_near_goal) {
    const std::vector<geometry_msgs::msg::Pose2D> stationary_pose{pose};
    if (certify_physical_sequence(stationary_pose, nullptr, nullptr)) {
      // Do not widen GoalChecker success. A stopped native window can be
      // empty for one control cycle immediately outside its 0.25 m boundary;
      // hold a certified zero for at most the distance reachable in one
      // maximum-speed tick, then let the unchanged GoalChecker or progress
      // checker decide the action outcome on subsequent cycles.
      retained_backup_commands_.clear();
      retained_backup_states_.clear();
      terminal_stop_goal_capture_active_ = false;
      terminal_stop_goal_capture_committed_ = true;
      dwb_msgs::msg::TrajectoryScore hold_score;
      hold_score.total = 0.0;
      hold_score.traj.velocity = nav_2d_msgs::msg::Twist2D();
      hold_score.traj.poses = stationary_pose;
      dwb_msgs::msg::CriticScore hold_detail;
      hold_detail.name = "EmptyNativeWindowStoppedHold";
      hold_detail.scale = 0.0;
      hold_detail.raw_score = 1.0;
      hold_score.scores.push_back(std::move(hold_detail));
      if (results) {
        results->twists.push_back(hold_score);
        results->best_index = results->twists.size() - 1u;
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Native candidate window is empty while stopped immediately outside "
        "the terminal capture boundary; dispatching a certified zero hold");
      return hold_score;
    }
  }

  const bool selected_response_lacks_stop_progress =
    best.total >= 0.0 && best_stop_progress_was_evaluated &&
    !best_stop_has_translation_progress &&
    !best_stop_has_heading_progress;
  const bool obstacle_recovery_candidate_is_available = std::any_of(
    rejected_recovery_candidates.begin(),
    rejected_recovery_candidates.end(),
    [](const RejectedRecoveryCandidate & candidate) {
      return candidate.obstacle_footprint_rejection;
    });
  const bool obstacle_rejection_stall_needs_progress =
    best.total >= 0.0 && clearance_progress_preference_enabled &&
    std::abs(best.traj.velocity.x) <= terminal_stop_velocity_threshold_ &&
    obstacle_recovery_candidate_is_available;
  const bool stalled_avoidance_needs_receding_horizon_progress =
    (selected_response_lacks_stop_progress &&
    clearance_progress_preference_enabled &&
    (best_clearance_trigger_bucket > 0u ||
    (jerk_guard_progress_recovery_enabled &&
    best_clearance_guard_bucket > 0u))) ||
    obstacle_rejection_stall_needs_progress;
  if (stalled_avoidance_needs_receding_horizon_progress) {
    // A full ordinary rollout may reach an obstacle even though issuing only
    // its first method-native command and then braking is physically safe.
    // DWB rejects that rollout before it can enter the legal progress set.
    // Reconsider only ObstacleFootprint rejections, require measurable Path
    // progress, and certify the complete first-command-plus-stop sequence.
    // This preserves the hard footprint gate while allowing the next control
    // cycle to replan around newly observed obstacles.
    std::map<std::size_t, dwb_msgs::msg::Trajectory2D>
    non_native_rejected_trajectories;
    if (!native_generator) {
      traj_generator_->startNewIteration(velocity);
      std::size_t canonical_index = 0u;
      std::size_t rejected_index = 0u;
      while (traj_generator_->hasMoreTwists() &&
        rejected_index < rejected_recovery_candidates.size())
      {
        const auto twist = traj_generator_->nextTwist();
        const auto & rejected_candidate =
          rejected_recovery_candidates[rejected_index];
        if (canonical_index == rejected_candidate.canonical_index) {
          non_native_rejected_trajectories.emplace(
            canonical_index,
            traj_generator_->generateTrajectory(pose, velocity, twist));
          ++rejected_index;
        }
        ++canonical_index;
      }
    }
    for (const auto & candidate : rejected_recovery_candidates) {
      if (!candidate.obstacle_footprint_rejection) {
        continue;
      }
      try {
        dwb_msgs::msg::Trajectory2D candidate_trajectory;
        NativeInputTrajectoryGenerator::NativeCommandState command_state;
        if (native_generator) {
          if (!native_generator->materialize_candidate(
              candidate.canonical_index, pose, candidate_trajectory,
              command_state))
          {
            continue;
          }
        } else {
          const auto trajectory = non_native_rejected_trajectories.find(
            candidate.canonical_index);
          if (trajectory == non_native_rejected_trajectories.end()) {
            continue;
          }
          candidate_trajectory = trajectory->second;
        }

        std::vector<geometry_msgs::msg::Pose2D> stop_poses;
        if (!build_stop_trajectory(
            candidate_trajectory, stop_poses, nullptr, nullptr,
            candidate.canonical_index) ||
          !certify_pose_sequence(
            *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
            stop_poses, maximum_swept_distance_,
            &certification_workspace_).safe)
        {
          continue;
        }

        const bool stop_has_translation_progress =
          pose_sequence_has_meaningful_path_progress(
          stop_poses, current_progress_reference_plan_,
          current_terminal_distance_target_pose_,
          clearance_constraint_minimum_subgoal_distance_progress_,
          clearance_constraint_minimum_subgoal_heading_progress_);
        const bool rollout_has_translation_progress =
          !stop_has_translation_progress &&
          trajectory_has_meaningful_subgoal_progress(
          candidate_trajectory, current_progress_reference_plan_,
          current_terminal_distance_target_pose_,
          clearance_constraint_minimum_subgoal_distance_progress_,
          clearance_constraint_minimum_subgoal_heading_progress_);
        const bool has_translation_progress =
          stop_has_translation_progress || rollout_has_translation_progress;
        const bool stop_has_heading_progress =
          !has_translation_progress &&
          pose_sequence_has_meaningful_path_progress(
          stop_poses, current_progress_reference_plan_,
          current_terminal_distance_target_pose_, 0.0,
          clearance_constraint_minimum_subgoal_heading_progress_);
        const bool rollout_has_heading_progress =
          !has_translation_progress && !stop_has_heading_progress &&
          trajectory_has_meaningful_subgoal_progress(
          candidate_trajectory, current_progress_reference_plan_,
          current_terminal_distance_target_pose_, 0.0,
          clearance_constraint_minimum_subgoal_heading_progress_);
        const bool has_heading_progress =
          stop_has_heading_progress || rollout_has_heading_progress;
        const bool has_path_progress =
          has_translation_progress || has_heading_progress;
        const bool has_method_native_motion =
          std::abs(candidate_trajectory.velocity.x) >
          terminal_stop_velocity_threshold_ ||
          std::abs(candidate_trajectory.velocity.theta) >
          terminal_stop_velocity_threshold_;
        const bool is_avoidance_only_escape =
          obstacle_rejection_stall_needs_progress &&
          !has_path_progress && has_method_native_motion;
        if (!has_path_progress && !is_avoidance_only_escape) {
          continue;
        }

        double avoidance_horizon_seconds =
          candidate.collision_time.value_or(0.0);
        if (std::isnan(avoidance_horizon_seconds)) {
          avoidance_horizon_seconds = 0.0;
        }
        avoidance_horizon_seconds =
          std::max(0.0, avoidance_horizon_seconds);

        const double primary_risk =
          candidate.primary_clearance_risk.value_or(
          candidate.clearance_risk.value_or(0.0));
        const double trigger_risk =
          candidate.trigger_clearance_risk.value_or(primary_risk);
        const double guard_risk =
          candidate.guard_clearance_risk.value_or(primary_risk);
        const uint64_t clearance_risk_bucket =
          clearance_constraint_bucket(
          primary_risk, clearance_constraint_admissible_risk_,
          clearance_constraint_risk_resolution_);
        const double configured_trigger_risk =
          clearance_constraint_trigger_risk_ >= 0.0 ?
          clearance_constraint_trigger_risk_ :
          clearance_constraint_admissible_risk_;
        const uint64_t clearance_trigger_bucket =
          clearance_constraint_bucket(
          trigger_risk, configured_trigger_risk,
          clearance_constraint_risk_resolution_);
        const uint64_t clearance_guard_bucket =
          clearance_constraint_bucket(
          guard_risk, clearance_constraint_guard_admissible_risk_,
          clearance_constraint_guard_risk_resolution_);

        double path_deviation_cost = 0.0;
        double mean_path_distance_cost = 0.0;
        for (const auto & critic : critics_) {
          if (critic->getName() == "PathDeviation") {
            path_deviation_cost =
              critic->scoreTrajectory(candidate_trajectory) * critic->getScale();
          } else if (critic->getName() == "MeanPathDist") {
            mean_path_distance_cost =
              critic->scoreTrajectory(candidate_trajectory) * critic->getScale();
          }
        }
        if (!std::isfinite(path_deviation_cost) ||
          path_deviation_cost < 0.0 ||
          !std::isfinite(mean_path_distance_cost) ||
          mean_path_distance_cost < 0.0)
        {
          continue;
        }
        const bool progress_is_proven_by_stop =
          stop_has_translation_progress || stop_has_heading_progress;
        const auto & progress_terminal_pose = progress_is_proven_by_stop ?
          stop_poses.back() : candidate_trajectory.poses.back();
        const double progress_terminal_heading_error =
          std::abs(std::remainder(
          current_terminal_distance_target_pose_.theta -
          progress_terminal_pose.theta, 2.0 * M_PI));
        const double progress_cost = is_avoidance_only_escape ?
          (std::isfinite(avoidance_horizon_seconds) ?
          1.0 / (1.0 + avoidance_horizon_seconds) : 0.0) :
          (has_translation_progress ?
          std::hypot(
          path_subgoal_forward_ray_cost(
            progress_terminal_pose, current_terminal_distance_target_pose_,
            terminal_stop_path_lateral_weight_),
          terminal_stop_path_lookahead_distance_ *
          progress_terminal_heading_error) :
          progress_terminal_heading_error);
        if (!std::isfinite(progress_cost) || progress_cost < 0.0) {
          continue;
        }

        const bool rank_is_better = [&]() {
            if (!progress_escape.found) {
              return true;
            }
            if (jerk_guard_progress_recovery_enabled &&
              clearance_guard_bucket !=
              progress_escape.clearance_guard_bucket)
            {
              return clearance_guard_bucket <
                     progress_escape.clearance_guard_bucket;
            }
            // During an ObstacleFootprint-rejection stall, first turn away
            // from the fused master-map boundary and live-only obstacle. A
            // collision-time-first rank can prefer the wrong side of a wall
            // or blocker when two forward-only V-DWA rollouts differ only by
            // one sensor/control phase. The command actually issued remains
            // only its separately certified first step plus complete stop.
            if (obstacle_rejection_stall_needs_progress &&
              clearance_guard_bucket !=
              progress_escape.clearance_guard_bucket)
            {
              return clearance_guard_bucket <
                     progress_escape.clearance_guard_bucket;
            }
            if (obstacle_rejection_stall_needs_progress &&
              clearance_risk_bucket !=
              progress_escape.clearance_risk_bucket)
            {
              return clearance_risk_bucket <
                     progress_escape.clearance_risk_bucket;
            }
            if (obstacle_rejection_stall_needs_progress &&
              avoidance_horizon_seconds !=
              progress_escape.avoidance_horizon_seconds)
            {
              return avoidance_horizon_seconds >
                     progress_escape.avoidance_horizon_seconds;
            }
            if (has_translation_progress !=
              progress_escape.has_translation_progress)
            {
              return has_translation_progress;
            }
            if (!obstacle_rejection_stall_needs_progress &&
              avoidance_horizon_seconds !=
              progress_escape.avoidance_horizon_seconds)
            {
              return avoidance_horizon_seconds >
                     progress_escape.avoidance_horizon_seconds;
            }
            if (clearance_guard_bucket !=
              progress_escape.clearance_guard_bucket)
            {
              return clearance_guard_bucket <
                     progress_escape.clearance_guard_bucket;
            }
            if (clearance_risk_bucket !=
              progress_escape.clearance_risk_bucket)
            {
              return clearance_risk_bucket <
                     progress_escape.clearance_risk_bucket;
            }
            if (path_deviation_cost != progress_escape.path_deviation_cost) {
              return path_deviation_cost <
                     progress_escape.path_deviation_cost;
            }
            if (progress_cost != progress_escape.progress_cost) {
              return progress_cost < progress_escape.progress_cost;
            }
            const uint64_t mean_path_distance_bucket =
              static_cast<uint64_t>(mean_path_distance_cost);
            if (mean_path_distance_bucket !=
              progress_escape.mean_path_distance_bucket)
            {
              return mean_path_distance_bucket <
                     progress_escape.mean_path_distance_bucket;
            }
            if (mean_path_distance_cost !=
              progress_escape.mean_path_distance_cost)
            {
              return mean_path_distance_cost <
                     progress_escape.mean_path_distance_cost;
            }
            return candidate.canonical_index <
                   progress_escape.canonical_index;
          }();
        if (!rank_is_better) {
          continue;
        }

        progress_escape.found = true;
        progress_escape.canonical_index = candidate.canonical_index;
        progress_escape.evaluation_index =
          std::numeric_limits<std::size_t>::max();
        progress_escape.clearance_risk_bucket = clearance_risk_bucket;
        progress_escape.clearance_trigger_bucket = clearance_trigger_bucket;
        progress_escape.clearance_guard_bucket = clearance_guard_bucket;
        progress_escape.mean_path_distance_bucket =
          static_cast<uint64_t>(mean_path_distance_cost);
        progress_escape.total = 0.0;
        progress_escape.path_deviation_cost = path_deviation_cost;
        progress_escape.mean_path_distance_cost = mean_path_distance_cost;
        progress_escape.progress_cost = progress_cost;
        progress_escape.avoidance_horizon_seconds =
          avoidance_horizon_seconds;
        progress_escape.has_translation_progress = has_translation_progress;
        progress_escape.has_heading_progress = has_heading_progress;
        progress_escape.captures_goal = false;
        progress_escape.uses_reserve_recovery = false;
        progress_escape.command_state = native_generator ?
          std::make_optional(command_state) : std::nullopt;
        progress_escape.trajectory = std::move(candidate_trajectory);
        progress_escape.stop_poses = std::move(stop_poses);
      } catch (const dwb_core::IllegalTrajectoryException &) {
        continue;
      }
    }
  }
  // The J-DWA guard is the method-specific static-jerk recovery boundary and
  // must not be crossed from its admissible band.  V/F candidates already
  // pass the full rollout and stop-suffix physical-footprint certificates;
  // treating their soft FootprintClearance band as a hard invariant creates
  // a zero-command deadlock before an otherwise certified bypass can begin.
  const bool escape_preserves_guard_admissibility =
    !jerk_guard_progress_recovery_enabled ||
    best_clearance_guard_bucket > 0u ||
    progress_escape.clearance_guard_bucket == 0u;
  const bool progress_escape_is_eligible =
    progress_escape.found &&
    (selected_response_lacks_stop_progress ||
    obstacle_rejection_stall_needs_progress) &&
    (best_clearance_trigger_bucket > 0u ||
    progress_escape.clearance_trigger_bucket > 0u ||
    (jerk_guard_progress_recovery_enabled &&
    best_clearance_guard_bucket > 0u) ||
    obstacle_rejection_stall_needs_progress) &&
    (best_stop_progress_was_evaluated ||
    obstacle_rejection_stall_needs_progress) &&
    (obstacle_rejection_stall_needs_progress ||
    (progress_escape.has_translation_progress ?
    !best_stop_has_translation_progress :
    !best_stop_has_heading_progress)) &&
    escape_preserves_guard_admissibility;
  const bool use_progress_escape = progress_escape_is_eligible;
  if (use_progress_escape) {
    const bool selected_avoidance_only_escape =
      !progress_escape.has_translation_progress &&
      !progress_escape.has_heading_progress;
    dwb_msgs::msg::CriticScore escape_detail;
    escape_detail.name = selected_avoidance_only_escape ?
      "ObstacleRejectionEscapeRecovery" :
      "StalledNativeProgressRecovery";
    escape_detail.scale = 0.0;
    escape_detail.raw_score = 1.0;
    if (results && progress_escape.evaluation_index < results->twists.size()) {
      results->twists[progress_escape.evaluation_index].scores.push_back(
        escape_detail);
      results->best_index = progress_escape.evaluation_index;
      best = results->twists[progress_escape.evaluation_index];
    } else {
      best = dwb_msgs::msg::TrajectoryScore();
      best.total = progress_escape.total;
      best.traj = std::move(progress_escape.trajectory);
      best.scores.push_back(std::move(escape_detail));
      if (results) {
        results->twists.push_back(best);
        results->best_index = results->twists.size() - 1u;
      }
    }
    best_canonical_index = progress_escape.canonical_index;
    best_clearance_risk_bucket = progress_escape.clearance_risk_bucket;
    best_clearance_trigger_bucket = progress_escape.clearance_trigger_bucket;
    best_clearance_guard_bucket = progress_escape.clearance_guard_bucket;
    best_command_state = progress_escape.command_state;
    best_has_meaningful_progress = !selected_avoidance_only_escape;
    best_stop_has_translation_progress =
      progress_escape.has_translation_progress;
    best_stop_has_heading_progress = progress_escape.has_heading_progress;
    best_stop_progress_was_evaluated = true;
    best_captures_goal = progress_escape.captures_goal;
    best_uses_reserve_recovery = progress_escape.uses_reserve_recovery;
    if (selected_avoidance_only_escape) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Path-progress candidates were stalled behind an obstacle; "
        "dispatching stop-admissible method-native escape candidate=%zu "
        "guard_bucket=%" PRIu64 " dynamic_bucket=%" PRIu64
        " collision_horizon=%.3f "
        "path_deviation=%.3f mean_path=%.3f progress=%.3f",
        best_canonical_index,
        progress_escape.clearance_guard_bucket,
        progress_escape.clearance_risk_bucket,
        progress_escape.avoidance_horizon_seconds,
        progress_escape.path_deviation_cost,
        progress_escape.mean_path_distance_cost,
        progress_escape.progress_cost);
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Selected translational command was stalled during unknown-"
        "obstacle avoidance; dispatching stop-admissible method-native "
        "progress candidate=%zu guard_bucket=%" PRIu64
        " dynamic_bucket=%" PRIu64 " "
        "collision_horizon=%.3f path_deviation=%.3f mean_path=%.3f "
        "progress=%.3f",
        best_canonical_index,
        progress_escape.clearance_guard_bucket,
        progress_escape.clearance_risk_bucket,
        progress_escape.avoidance_horizon_seconds,
        progress_escape.path_deviation_cost,
        progress_escape.mean_path_distance_cost,
        progress_escape.progress_cost);
    }
  }

  if (best.total < 0.0 &&
    no_valid_control_deceleration_fallback_enabled_ &&
    !inside_terminal_capture_region)
  {
    if (best.total < 0.0) {
      dwb_msgs::msg::TrajectoryScore retained_stop_score;
      const auto consume_retained_stop_command = [this]() {
          const bool keep_captured_terminal_command =
            retained_backup_commands_.size() == 1u &&
            std::abs(retained_backup_commands_.front().x) <=
            terminal_stop_velocity_threshold_ &&
            std::abs(retained_backup_commands_.front().y) <=
            terminal_stop_velocity_threshold_ &&
            std::abs(retained_backup_commands_.front().theta) <=
            terminal_stop_velocity_threshold_;
          if (!keep_captured_terminal_command) {
            retained_backup_commands_.erase(retained_backup_commands_.begin());
            if (!retained_backup_states_.empty()) {
              retained_backup_states_.erase(retained_backup_states_.begin());
            }
          }
        };
      if (build_revalidated_backup(pose, retained_stop_score)) {
        if (!retained_stop_score.scores.empty()) {
          retained_stop_score.scores.front().name =
            "RetainedRevalidatedStopRecovery";
        }
        if (results) {
          results->twists.push_back(retained_stop_score);
          results->best_index = results->twists.size() - 1u;
        }
        consume_retained_stop_command();
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "All nominal trajectories were invalid; dispatching the next "
          "method-native stop command after revalidating its complete suffix");
        return retained_stop_score;
      }

      const std::size_t required_safe_step_count =
        static_cast<std::size_t>(std::max(
          1.0, std::ceil(
            (planning_deadline_seconds_ + certification_control_period_) /
            certification_control_period_)));
      const bool retained_prefix_state_is_available =
        !native_generator || !retained_backup_states_.empty();
      if (retained_prefix_state_is_available &&
        !retained_backup_commands_.empty())
      {
        std::vector<geometry_msgs::msg::Pose2D> retained_prefix_poses;
        retained_prefix_poses.reserve(required_safe_step_count + 1u);
        geometry_msgs::msg::Pose2D retained_pose = pose;
        retained_prefix_poses.push_back(retained_pose);
        for (std::size_t step = 0u; step < required_safe_step_count; ++step) {
          const std::size_t command_index = std::min(
            step, retained_backup_commands_.size() - 1u);
          retained_pose = integrate_pose(
            retained_pose, retained_backup_commands_[command_index],
            certification_control_period_);
          retained_prefix_poses.push_back(retained_pose);
        }
        if (certify_physical_sequence(
            retained_prefix_poses, nullptr, nullptr))
        {
          retained_stop_score = dwb_msgs::msg::TrajectoryScore();
          retained_stop_score.total = 0.0;
          retained_stop_score.traj.velocity =
            retained_backup_commands_.front();
          retained_stop_score.traj.poses = std::move(retained_prefix_poses);
          dwb_msgs::msg::CriticScore prefix_score;
          prefix_score.name = "RetainedRevalidatedStopPrefix";
          prefix_score.scale = 0.0;
          prefix_score.raw_score = 1.0;
          retained_stop_score.scores.push_back(std::move(prefix_score));
          if (native_generator) {
            native_generator->select_command_for_dispatch(
              retained_backup_states_.front());
          }
          if (results) {
            results->twists.push_back(retained_stop_score);
            results->best_index = results->twists.size() - 1u;
          }
          consume_retained_stop_command();
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Complete retained stop became obstructed; dispatching its next "
            "method-native command after revalidating the bounded prefix");
          return retained_stop_score;
        }
      }
    }
    retained_backup_commands_.clear();
    retained_backup_states_.clear();

    std::size_t recovery_canonical_index =
      std::numeric_limits<std::size_t>::max();
    double recovery_collision_time = 0.0;
    double recovery_clearance_risk = 0.0;
    double recovery_path_departure_cost = 0.0;
    dwb_msgs::msg::TrajectoryScore recovery_score;
    std::optional<NativeInputTrajectoryGenerator::NativeCommandState>
    recovery_command_state;
    bool recovery_candidate_found = false;

    const auto consider_recovery_candidate =
      [this, &recovery_canonical_index, &recovery_collision_time,
        &recovery_clearance_risk, &recovery_path_departure_cost,
        &recovery_score, &recovery_command_state,
        &recovery_candidate_found](
      const RejectedRecoveryCandidate & candidate,
      const dwb_msgs::msg::Trajectory2D & candidate_trajectory)
      {
        try {
          const std::size_t required_safe_step_count =
            static_cast<std::size_t>(std::max(
            1.0, std::ceil(
                (planning_deadline_seconds_ + certification_control_period_) /
              certification_control_period_)));
          if (candidate_trajectory.poses.size() <= required_safe_step_count) {
            return;
          }

          double collision_time = candidate.collision_time.value_or(
          std::numeric_limits<double>::quiet_NaN());
          if (!candidate.collision_time) {
            const CertificationResult full_result = certify_pose_sequence(
            *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
            candidate_trajectory.poses, maximum_swept_distance_,
            &certification_workspace_);
            collision_time = full_result.safe ?
              std::numeric_limits<double>::infinity() : predicted_collision_time(
            full_result, candidate_trajectory.poses,
            costmap_ros_->getRobotFootprint(), maximum_swept_distance_,
            certification_control_period_);
          }
          if (std::isnan(collision_time)) {
            return;
          }
          if (recovery_candidate_found &&
            collision_time < recovery_collision_time)
          {
            return;
          }

          double clearance_risk = candidate.clearance_risk.value_or(0.0);
          if (clearance_constraint_enabled_) {
            if (!candidate.clearance_risk) {
              const double primary_clearance_risk =
                clearance_constraint_critic_->scoreTrajectoryWithApproachRisk(
              candidate_trajectory, nullptr);
              const double guard_clearance_risk =
                clearance_constraint_guard_critic_.get() ==
                clearance_constraint_critic_.get() ?
                primary_clearance_risk :
                clearance_constraint_guard_critic_->scoreTrajectory(
              candidate_trajectory);
              clearance_risk = std::max(
              primary_clearance_risk, guard_clearance_risk);
            }
            if (!std::isfinite(clearance_risk) || clearance_risk < 0.0 ||
              clearance_risk > 1.0 + 1.0e-9)
            {
              return;
            }
            clearance_risk = std::clamp(clearance_risk, 0.0, 1.0);
          }
          if (recovery_candidate_found &&
            collision_time == recovery_collision_time &&
            clearance_risk > recovery_clearance_risk)
          {
            return;
          }

          const std::vector<geometry_msgs::msg::Pose2D> safe_prefix{
            candidate_trajectory.poses.begin(),
            candidate_trajectory.poses.begin() +
            static_cast<std::ptrdiff_t>(required_safe_step_count + 1u)};
          if (!certify_physical_sequence(safe_prefix, nullptr, nullptr)) {
            return;
          }

          double path_departure_cost = 0.0;
          for (const auto & critic : critics_) {
            if (critic->getName() != "PathDeviation" &&
              critic->getName() != "MeanPathDist")
            {
              continue;
            }
            const double raw_score = critic->scoreTrajectory(
            candidate_trajectory);
            path_departure_cost += raw_score * critic->getScale();
          }
          if (!std::isfinite(path_departure_cost) ||
            path_departure_cost < 0.0)
          {
            return;
          }

          if (!receding_horizon_recovery_prefers_candidate(
            collision_time, recovery_collision_time,
            clearance_risk, recovery_clearance_risk,
            path_departure_cost, recovery_path_departure_cost,
            candidate.canonical_index, recovery_canonical_index))
          {
            return;
          }

          recovery_canonical_index = candidate.canonical_index;
          recovery_collision_time = collision_time;
          recovery_clearance_risk = clearance_risk;
          recovery_path_departure_cost = path_departure_cost;
          recovery_command_state = candidate.command_state;
          recovery_candidate_found = true;
          recovery_score = dwb_msgs::msg::TrajectoryScore();
          recovery_score.total = 0.0;
          recovery_score.traj = candidate_trajectory;
          recovery_score.traj.poses.resize(2u);
          if (recovery_score.traj.time_offsets.size() > 1u) {
            recovery_score.traj.time_offsets.resize(1u);
          }
        } catch (const dwb_core::IllegalTrajectoryException &) {
          return;
        }
      };

    if (native_generator) {
      std::vector<const RejectedRecoveryCandidate *> ordered_candidates;
      std::vector<const RejectedRecoveryCandidate *> unknown_collision_time;
      ordered_candidates.reserve(rejected_recovery_candidates.size());
      unknown_collision_time.reserve(rejected_recovery_candidates.size());
      for (const auto & candidate : rejected_recovery_candidates) {
        (candidate.collision_time ? ordered_candidates :
        unknown_collision_time).push_back(&candidate);
      }
      std::stable_sort(
        ordered_candidates.begin(), ordered_candidates.end(),
        [](const RejectedRecoveryCandidate * first,
        const RejectedRecoveryCandidate * second)
        {
          if (*first->collision_time != *second->collision_time) {
            return *first->collision_time > *second->collision_time;
          }
          if (first->clearance_risk && second->clearance_risk &&
          *first->clearance_risk != *second->clearance_risk)
          {
            return *first->clearance_risk < *second->clearance_risk;
          }
          if (first->clearance_risk.has_value() !=
          second->clearance_risk.has_value())
          {
            return first->clearance_risk.has_value();
          }
          return first->canonical_index < second->canonical_index;
        });
      for (const auto * candidate : ordered_candidates) {
        if (recovery_candidate_found &&
          *candidate->collision_time < recovery_collision_time)
        {
          break;
        }
        if (recovery_candidate_found &&
          *candidate->collision_time == recovery_collision_time &&
          candidate->clearance_risk &&
          *candidate->clearance_risk > recovery_clearance_risk)
        {
          continue;
        }
        dwb_msgs::msg::Trajectory2D candidate_trajectory;
        NativeInputTrajectoryGenerator::NativeCommandState ignored_state;
        if (!native_generator->materialize_candidate(
            candidate->canonical_index, pose, candidate_trajectory,
            ignored_state))
        {
          continue;
        }
        consider_recovery_candidate(*candidate, candidate_trajectory);
      }
      for (const auto * candidate : unknown_collision_time) {
        dwb_msgs::msg::Trajectory2D candidate_trajectory;
        NativeInputTrajectoryGenerator::NativeCommandState ignored_state;
        if (!native_generator->materialize_candidate(
            candidate->canonical_index, pose, candidate_trajectory,
            ignored_state))
        {
          continue;
        }
        consider_recovery_candidate(*candidate, candidate_trajectory);
      }
    } else {
      traj_generator_->startNewIteration(velocity);
      std::size_t canonical_index = 0u;
      std::size_t rejected_index = 0u;
      while (traj_generator_->hasMoreTwists() &&
        rejected_index < rejected_recovery_candidates.size())
      {
        const auto twist = traj_generator_->nextTwist();
        const auto & candidate =
          rejected_recovery_candidates[rejected_index];
        if (canonical_index == candidate.canonical_index) {
          const auto candidate_trajectory =
            traj_generator_->generateTrajectory(pose, velocity, twist);
          consider_recovery_candidate(candidate, candidate_trajectory);
          ++rejected_index;
        }
        ++canonical_index;
      }
    }

    if (recovery_candidate_found &&
      recovery_canonical_index != std::numeric_limits<std::size_t>::max())
    {
      const auto append_recovery_detail =
        [&recovery_score](
        const std::string & name, const double raw_score)
        {
          dwb_msgs::msg::CriticScore detail;
          detail.name = name;
          detail.scale = 0.0;
          detail.raw_score = raw_score;
          recovery_score.scores.push_back(std::move(detail));
        };
      append_recovery_detail("RecedingHorizonRecovery", 1.0);
      append_recovery_detail(
        "RecoveryPredictedCollisionTime",
        std::isfinite(recovery_collision_time) ?
        recovery_collision_time : std::numeric_limits<double>::max());
      append_recovery_detail(
        "RecoveryFusedClearanceRisk", recovery_clearance_risk);
      append_recovery_detail(
        "RecoveryPathDepartureCost", recovery_path_departure_cost);
      retained_backup_commands_.clear();
      retained_backup_states_.clear();
      terminal_stop_goal_capture_active_ = false;
      terminal_stop_goal_capture_committed_ = false;
      if (native_generator) {
        native_generator->select_command_for_dispatch(
          recovery_command_state);
      }
      if (results) {
        results->twists.push_back(recovery_score);
        results->best_index = results->twists.size() - 1u;
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "All nominal trajectories were invalid; dispatching verified "
        "method-native recovery prefix candidate=%zu collision_time=%.3f "
        "clearance_risk=%.3f path_departure_cost=%.3f",
        recovery_canonical_index, recovery_collision_time,
        recovery_clearance_risk, recovery_path_departure_cost);
      return recovery_score;
    }
  }

  if (best.total >= 0.0 && clearance_progress_preference_enabled) {
    if (best_stop_progress_was_evaluated) {
      best_has_meaningful_progress =
        best_stop_has_translation_progress ||
        best_stop_has_heading_progress;
    } else {
      best_has_meaningful_progress =
        trajectory_has_meaningful_subgoal_progress(
        best.traj, current_progress_reference_plan_,
        current_terminal_distance_target_pose_,
        clearance_constraint_minimum_subgoal_distance_progress_,
        clearance_constraint_minimum_subgoal_heading_progress_);
    }
  }
  const bool avoidance_progress_unavailable =
    best.total >= 0.0 &&
    (best_clearance_trigger_bucket > 0u ||
    (jerk_guard_progress_recovery_enabled &&
    best_clearance_guard_bucket > 0u)) &&
    clearance_progress_preference_enabled &&
    !best_has_meaningful_progress;
  if (avoidance_progress_unavailable) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Selected avoidance candidate has no measurable Path-subgoal progress; "
      "using the best available stop-admissible soft-risk class");
  }

  if (best.total >= 0.0) {
    if (!certification_enabled_ && !terminal_stop_policy_enabled &&
      !stop_admissibility_enabled_)
    {
      retained_backup_commands_.clear();
      retained_backup_states_.clear();
      terminal_stop_goal_capture_active_ = false;
      terminal_stop_goal_capture_committed_ = false;
      if (native_generator) {
        native_generator->select_command_for_dispatch(best_command_state);
      }
      return best;
    }
    std::vector<geometry_msgs::msg::Pose2D> best_stop_poses;
    const std::optional<std::size_t> native_candidate_index =
      native_generator ?
      std::make_optional(best_canonical_index) : std::nullopt;
    if (!build_stop_trajectory(
        best.traj, best_stop_poses, &best_stop_velocities,
        &best_stop_states, native_candidate_index))
    {
      if (certification_enabled_) {
        throw nav2_core::NoValidControl(
                "Certified best trajectory could not be materialized");
      }
      retained_backup_commands_.clear();
      retained_backup_states_.clear();
      terminal_stop_goal_capture_active_ = false;
      terminal_stop_goal_capture_committed_ = false;
      if (native_generator) {
        native_generator->select_command_for_dispatch(best_command_state);
      }
      return best;
    }
    retained_backup_commands_.clear();
    retained_backup_states_.clear();
    if (native_generator) {
      if (best_stop_states.empty()) {
        native_generator->select_command_for_dispatch(std::nullopt);
      } else {
        native_generator->select_command_for_dispatch(
          best_stop_states.front());
      }
    }
    terminal_stop_goal_capture_active_ = false;
    if (terminal_endpoint_policy_enabled) {
      terminal_stop_goal_capture_active_ = best_captures_goal;
    } else {
      if (
        terminal_stop_goal_capture_distance_ > 0.0 &&
        terminal_stop_goal_capture_yaw_tolerance_ > 0.0 &&
        current_goal_pose_valid_ &&
        !best_stop_poses.empty())
      {
        const geometry_msgs::msg::Pose2D & terminal_pose =
          best_stop_poses.back();
        const double yaw_error = std::abs(std::atan2(
            std::sin(terminal_pose.theta - current_goal_pose_.theta),
            std::cos(terminal_pose.theta - current_goal_pose_.theta)));
        const double position_error = std::hypot(
          terminal_pose.x - current_goal_pose_.x,
          terminal_pose.y - current_goal_pose_.y);
        if (
          position_error <= terminal_stop_goal_capture_distance_ &&
          yaw_error <= terminal_stop_goal_capture_yaw_tolerance_)
        {
          terminal_stop_goal_capture_active_ = true;
        }
      }
    }
    terminal_stop_goal_capture_committed_ =
      terminal_stop_goal_capture_active_;
    // Every accepted native candidate already carries a fully certified stop
    // sequence. Retain only its not-yet-issued suffix so a later sensor update
    // can trigger one revalidated braking command instead of Controller
    // Server's discontinuous emergency zero. Ordinary cycles still replan and
    // replace this suffix; it is never dispatched without fresh validation.
    if (best_stop_velocities.size() > 1u) {
      retained_backup_commands_.assign(
        best_stop_velocities.begin() + 1,
        best_stop_velocities.end());
    }
    if (best_stop_states.size() > 1u) {
      retained_backup_states_.assign(
        std::make_move_iterator(best_stop_states.begin() + 1),
        std::make_move_iterator(best_stop_states.end()));
    }
    return best;
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
}  // NOLINT(readability/fn_size)

dwb_msgs::msg::TrajectoryScore
CertifiedDWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double best_score)
{
  dwb_msgs::msg::TrajectoryScore score;
  score.traj = trajectory;
  score_trajectory_components(
    trajectory, best_score, score, true, nullptr);
  return score;
}

void CertifiedDWBLocalPlanner::score_trajectory_components(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double best_score,
  dwb_msgs::msg::TrajectoryScore & score,
  const bool record_score_details,
  bool * used_reserve_recovery,
  const std::optional<double> precomputed_clearance_risk,
  const std::optional<double> precomputed_clearance_trigger_risk,
  const std::optional<double> precomputed_clearance_guard_risk,
  double * clearance_risk,
  const std::vector<geometry_msgs::msg::Pose2D> * precomputed_stop_poses,
  bool * completed_weighted_score,
  double * path_deviation_cost,
  double * mean_path_distance_cost,
  bool * strict_physical_stop_safety_proven)
{
  if (completed_weighted_score) {
    *completed_weighted_score = true;
  }
  if (used_reserve_recovery) {
    *used_reserve_recovery = false;
  }
  score.total = 0.0;
  score.scores.clear();
  if (clearance_risk) {
    *clearance_risk = std::numeric_limits<double>::quiet_NaN();
  }
  if (path_deviation_cost) {
    *path_deviation_cost = std::numeric_limits<double>::quiet_NaN();
  }
  if (mean_path_distance_cost) {
    *mean_path_distance_cost = std::numeric_limits<double>::quiet_NaN();
  }
  if (strict_physical_stop_safety_proven) {
    *strict_physical_stop_safety_proven = false;
  }
  const bool score_terminal_stop = should_score_terminal_stop(
    certification_enabled_, stop_admissibility_enabled_,
    terminal_stop_goal_distance_scale_,
    current_terminal_distance_target_pose_valid_);
  if (record_score_details) {
    score.scores.reserve(
      critics_.size() + (score_terminal_stop ? 1u : 0u) +
      (stop_admissibility_enabled_ ? 1u : 0u) +
      (certification_enabled_ ? 2u : 0u));
  }
  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    const double critic_scale = critic->getScale();
    const bool is_clearance_constraint_critic =
      clearance_constraint_critic_ &&
      critic.get() == clearance_constraint_critic_.get();
    const bool is_clearance_constraint_trigger_critic =
      clearance_constraint_trigger_critic_ &&
      critic.get() == clearance_constraint_trigger_critic_.get();
    const bool is_clearance_constraint_guard_critic =
      clearance_constraint_guard_critic_ &&
      critic.get() == clearance_constraint_guard_critic_.get();
    if (critic_scale == 0.0) {
      const double raw_score = zero_scale_clearance_diagnostic_score(
        is_clearance_constraint_critic,
        is_clearance_constraint_trigger_critic,
        is_clearance_constraint_guard_critic,
        precomputed_clearance_risk,
        precomputed_clearance_trigger_risk,
        precomputed_clearance_guard_risk);
      if (is_clearance_constraint_critic && clearance_risk) {
        *clearance_risk = raw_score;
      }
      if (record_score_details) {
        dwb_msgs::msg::CriticScore critic_score;
        critic_score.name = critic->getName();
        critic_score.scale = critic_scale;
        critic_score.raw_score = raw_score;
        score.scores.push_back(std::move(critic_score));
      }
      continue;
    }

    const double raw_score =
      is_clearance_constraint_critic && precomputed_clearance_risk ?
      *precomputed_clearance_risk :
      (is_clearance_constraint_trigger_critic &&
      precomputed_clearance_trigger_risk ?
      *precomputed_clearance_trigger_risk :
      (is_clearance_constraint_guard_critic &&
      precomputed_clearance_guard_risk ?
      *precomputed_clearance_guard_risk :
      critic->scoreTrajectory(trajectory)));
    if (is_clearance_constraint_critic && clearance_risk) {
      *clearance_risk = raw_score;
    }
    if (path_deviation_cost && critic->getName() == "PathDeviation") {
      *path_deviation_cost = raw_score * critic_scale;
    }
    if (mean_path_distance_cost && critic->getName() == "MeanPathDist") {
      *mean_path_distance_cost = raw_score * critic_scale;
    }
    if (record_score_details) {
      dwb_msgs::msg::CriticScore critic_score;
      critic_score.name = critic->getName();
      critic_score.scale = critic_scale;
      critic_score.raw_score = raw_score;
      score.scores.push_back(std::move(critic_score));
    }
    score.total += raw_score * critic_scale;
    if (short_circuit_trajectory_evaluation_ &&
      best_score > 0.0 && score.total > best_score)
    {
      if (completed_weighted_score) {
        *completed_weighted_score = false;
      }
      break;
    }
  }

  if (certification_enabled_ || stop_admissibility_enabled_ ||
    score_terminal_stop)
  {
    if (best_score >= 0.0 && score.total > best_score) {
      return;
    }
    if (!precomputed_stop_poses) {
      stop_pose_scratch_.clear();
      if (!build_stop_trajectory(
          trajectory, stop_pose_scratch_, nullptr, nullptr))
      {
        if (certification_enabled_) {
          ++certification_rejections_.terminal_stop_infeasible;
        }
        throw dwb_core::IllegalTrajectoryException(
                certification_enabled_ ? "SafetyCertificate" :
                (stop_admissibility_enabled_ ?
                "StopAdmissibility" : "TerminalStopDynamics"),
                "No dynamically feasible terminal stop sequence");
      }
      precomputed_stop_poses = &stop_pose_scratch_;
    }
    if (precomputed_stop_poses->empty()) {
      throw dwb_core::IllegalTrajectoryException(
              "TerminalStopDynamics", "Terminal stop sequence is empty");
    }

    if (score_terminal_stop) {
      const geometry_msgs::msg::Pose2D & terminal_pose =
        precomputed_stop_poses->back();
      const char * stop_goal_score_name = nullptr;
      double stop_goal_raw_score = 0.0;
      switch (terminal_stop_score_mode_) {
        case TerminalStopScoreMode::kPathSubgoalProgress:
          stop_goal_score_name = "TerminalStopPathProgress";
          stop_goal_raw_score = path_subgoal_progress_cost(
            terminal_pose, current_terminal_distance_target_pose_);
          break;
        case TerminalStopScoreMode::kPathSubgoalForwardRay:
          stop_goal_score_name = "TerminalStopPathProgressRay";
          stop_goal_raw_score = path_subgoal_forward_ray_cost(
            terminal_pose, current_terminal_distance_target_pose_,
            terminal_stop_path_lateral_weight_);
          break;
        default:
          stop_goal_score_name =
            terminal_stop_score_mode_ ==
            TerminalStopScoreMode::kPathSubgoalDistance ?
            "TerminalStopPathSubgoalDist" : "TerminalStopGoalDist";
          stop_goal_raw_score = std::hypot(
            terminal_pose.x - current_terminal_distance_target_pose_.x,
            terminal_pose.y - current_terminal_distance_target_pose_.y);
          break;
      }
      if (record_score_details) {
        dwb_msgs::msg::CriticScore stop_goal_score;
        stop_goal_score.name = stop_goal_score_name;
        stop_goal_score.scale = terminal_stop_goal_distance_scale_;
        stop_goal_score.raw_score = stop_goal_raw_score;
        score.scores.push_back(std::move(stop_goal_score));
      }
      score.total += stop_goal_raw_score * terminal_stop_goal_distance_scale_;
      if (
        short_circuit_trajectory_evaluation_ &&
        best_score >= 0.0 && score.total > best_score)
      {
        return;
      }
    }
  }

  CertificationFailure stop_certification_failure =
    CertificationFailure::kInvalidInput;
  bool stop_certification_was_evaluated = false;
  bool stop_is_certified = false;
  bool certificate_used_reserve_recovery = false;
  if (stop_admissibility_enabled_ && certification_enabled_) {
    stop_certification_was_evaluated = true;
    stop_is_certified = certify_stop_poses(
      *precomputed_stop_poses, stop_certification_failure, nullptr,
      &certificate_used_reserve_recovery);
  }

  if (stop_admissibility_enabled_) {
    CertificationResult stop_admissibility_result;
    bool used_initial_overlap_recovery = false;
    // certified_footprint_ is the measured footprint padded outward by the
    // non-negative certification margin. A direct padded-footprint pass
    // therefore proves the contained physical footprint too. Reserve
    // recovery is checked separately because it intentionally substitutes
    // the physical footprint for the padded one.
    const bool padded_pass_proves_physical_safety =
      stop_certification_was_evaluated && stop_is_certified &&
      !certificate_used_reserve_recovery;
    const bool stop_is_admissible = padded_pass_proves_physical_safety ||
      certify_physical_sequence(
      *precomputed_stop_poses, &stop_admissibility_result,
      &used_initial_overlap_recovery);
    if (!stop_is_admissible) {
      throw dwb_core::IllegalTrajectoryException(
              "StopAdmissibility",
              certification_failure_name(
                stop_admissibility_result.failure));
    }
    if (strict_physical_stop_safety_proven) {
      // A direct physical-footprint pass, or a pass by its padded superset,
      // is exactly the strict gate used by progress-candidate retention.
      // Initial-overlap recovery is intentionally excluded so that boundary
      // recovery cannot be promoted to an ordinary strict-safe candidate.
      *strict_physical_stop_safety_proven =
        padded_pass_proves_physical_safety ||
        stop_admissibility_result.safe;
    }
    if (record_score_details) {
      dwb_msgs::msg::CriticScore admissibility_score;
      admissibility_score.name = "StopAdmissibility";
      admissibility_score.scale = 0.0;
      admissibility_score.raw_score = 0.0;
      score.scores.push_back(std::move(admissibility_score));
      if (used_initial_overlap_recovery) {
        dwb_msgs::msg::CriticScore recovery_score;
        recovery_score.name = "InitialOverlapRecovery";
        recovery_score.scale = 0.0;
        recovery_score.raw_score = 1.0;
        score.scores.push_back(std::move(recovery_score));
      }
    }
  }

  if (certification_enabled_) {
    if (!stop_certification_was_evaluated) {
      stop_certification_was_evaluated = true;
      stop_is_certified = certify_stop_poses(
        *precomputed_stop_poses, stop_certification_failure, nullptr,
        &certificate_used_reserve_recovery);
    }
    if (!stop_is_certified) {
      record_certification_rejection(stop_certification_failure);
      throw dwb_core::IllegalTrajectoryException(
              "SafetyCertificate",
              certification_failure_name(stop_certification_failure));
    }
    if (record_score_details) {
      dwb_msgs::msg::CriticScore certificate_score;
      certificate_score.name = "SafetyCertificate";
      certificate_score.scale = 1.0;
      certificate_score.raw_score = 0.0;
      score.scores.push_back(std::move(certificate_score));
    }
    if (certificate_used_reserve_recovery) {
      if (record_score_details) {
        dwb_msgs::msg::CriticScore recovery_score;
        recovery_score.name = "ReserveRecovery";
        recovery_score.scale = 0.0;
        recovery_score.raw_score = 1.0;
        score.scores.push_back(std::move(recovery_score));
      }
      if (used_reserve_recovery) {
        *used_reserve_recovery = true;
      }
    }
  }
}

bool CertifiedDWBLocalPlanner::build_stop_trajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  std::vector<geometry_msgs::msg::Pose2D> & poses,
  std::vector<nav_2d_msgs::msg::Twist2D> * velocities,
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState> *
  command_states,
  const std::optional<std::size_t> native_candidate_index)
{
  poses.clear();
  if (velocities) {
    velocities->clear();
  }
  if (command_states) {
    command_states->clear();
  }
  if (command_states && !velocities) {
    return false;
  }
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
    if (native_candidate_index) {
      if (!velocities && !command_states) {
        return native_generator->generate_stop_poses_for_candidate(
          *native_candidate_index, trajectory.poses.front(),
          maximum_stop_steps, terminal_stop_velocity_threshold_, poses);
      }
      return native_generator->generate_stop_trajectory_for_candidate(
        *native_candidate_index, trajectory.poses.front(),
        maximum_stop_steps, terminal_stop_velocity_threshold_, poses,
        *velocities, command_states);
    }
    if (!velocities && !command_states) {
      return native_generator->generate_stop_poses(
        trajectory.poses.front(), maximum_stop_steps,
        terminal_stop_velocity_threshold_, poses);
    }
    return native_generator->generate_stop_trajectory(
      trajectory.poses.front(), maximum_stop_steps,
      terminal_stop_velocity_threshold_, poses, *velocities,
      command_states);
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
  const int delayed_candidate_steps = std::max(
    1, static_cast<int>(
      std::ceil(
        nominal_delay_preview_seconds_ /
        certification_control_period_)));
  for (int step_index = 0;
    step_index < delayed_candidate_steps; ++step_index)
  {
    pose = integrate_pose(
      pose, trajectory.velocity, certification_control_period_);
    if (velocities) {
      velocities->push_back(trajectory.velocity);
    }
    poses.push_back(pose);
  }
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
    if (velocities) {
      velocities->push_back(stop_velocity);
    }
    poses.push_back(pose);
  }
  nav_2d_msgs::msg::Twist2D zero_velocity;
  if (velocities) {
    velocities->push_back(zero_velocity);
  }
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

  bool used_reserve_recovery = false;
  bool used_initial_overlap_recovery = false;
  CertificationFailure failure = CertificationFailure::kInvalidInput;
  bool backup_is_valid = false;
  if (certification_enabled_) {
    backup_is_valid = certify_stop_poses(
      poses, failure, nullptr, &used_reserve_recovery);
    if (!backup_is_valid) {
      CertificationResult physical_result;
      const bool physical_recovery_is_valid = certify_physical_sequence(
        poses, &physical_result, &used_initial_overlap_recovery);
      backup_is_valid =
        physical_recovery_is_valid && used_initial_overlap_recovery;
      if (backup_is_valid) {
        failure = physical_result.failure;
      }
    }
  } else {
    CertificationResult planning_footprint_result;
    backup_is_valid = certify_physical_sequence(
      poses, &planning_footprint_result,
      &used_initial_overlap_recovery);
    failure = planning_footprint_result.failure;
  }
  if (!backup_is_valid) {
    const CertificationResult planning_footprint_result =
      certify_pose_sequence(
      *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
      poses, maximum_swept_distance_);
    if (!backup_is_valid) {
      RCLCPP_WARN(
        logger_,
        "Retained stop backup rejected during revalidation: %s; "
        "planning_footprint_safe=%s; planning_footprint_failure=%s",
        certification_failure_name(failure),
        planning_footprint_result.safe ? "true" : "false",
        certification_failure_name(planning_footprint_result.failure));
      return false;
    }
  }

  backup_score = dwb_msgs::msg::TrajectoryScore();
  backup_score.total = 0.0;
  backup_score.traj.velocity = retained_backup_commands_.front();
  backup_score.traj.poses = std::move(poses);
  dwb_msgs::msg::CriticScore certificate_score;
  certificate_score.name = "RetainedTerminalStop";
  certificate_score.scale = 1.0;
  certificate_score.raw_score = 0.0;
  backup_score.scores.push_back(certificate_score);
  if (used_reserve_recovery) {
    dwb_msgs::msg::CriticScore recovery_score;
    recovery_score.name = "ReserveRecovery";
    recovery_score.scale = 0.0;
    recovery_score.raw_score = 1.0;
    backup_score.scores.push_back(recovery_score);
  }
  if (used_initial_overlap_recovery) {
    dwb_msgs::msg::CriticScore recovery_score;
    recovery_score.name = "InitialOverlapRecovery";
    recovery_score.scale = 0.0;
    recovery_score.raw_score = 1.0;
    backup_score.scores.push_back(recovery_score);
  }
  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    if (retained_backup_states_.empty()) {
      native_generator->select_command_for_dispatch(std::nullopt);
    } else {
      native_generator->select_command_for_dispatch(
        retained_backup_states_.front());
    }
  }
  return true;
}

bool CertifiedDWBLocalPlanner::certify_stop_poses(
  const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
  CertificationFailure & failure,
  CertificationResult * output_result,
  bool * used_reserve_recovery) const
{
  if (used_reserve_recovery) {
    *used_reserve_recovery = false;
  }
  const CertificationResult result =
    certify_pose_sequence(
      *costmap_ros_->getCostmap(), certified_footprint_, stop_poses,
      maximum_swept_distance_, &certification_workspace_);
  if (output_result) {
    *output_result = result;
  }
  failure = result.failure;
  if (result.safe) {
    return true;
  }
  if (!enable_reserve_recovery_ ||
    result.failure != CertificationFailure::kLethalObstacle ||
    !certify_reserve_recovery_sequence(
      *costmap_ros_->getCostmap(), certified_footprint_,
      costmap_ros_->getRobotFootprint(), stop_poses,
      maximum_swept_distance_, reserve_recovery_hysteresis_,
      &certification_workspace_))
  {
    return false;
  }
  if (used_reserve_recovery) {
    *used_reserve_recovery = true;
  }
  return true;
}

bool CertifiedDWBLocalPlanner::certify_physical_sequence(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  CertificationResult * output_result,
  bool * used_initial_overlap_recovery) const
{
  if (used_initial_overlap_recovery) {
    *used_initial_overlap_recovery = false;
  }
  const CertificationResult result = certify_pose_sequence(
    *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
    poses, maximum_swept_distance_, &certification_workspace_);
  if (output_result) {
    *output_result = result;
  }
  if (result.safe) {
    return true;
  }
  if (!enable_initial_overlap_recovery_ ||
    result.failure != CertificationFailure::kLethalObstacle)
  {
    return false;
  }
  if (!certify_initial_overlap_margin_sequence(
      *costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
      initial_overlap_core_footprint_, poses, maximum_swept_distance_,
      nullptr, &certification_workspace_))
  {
    return false;
  }
  if (used_initial_overlap_recovery) {
    *used_initial_overlap_recovery = true;
  }
  return true;
}

void CertifiedDWBLocalPlanner::prepare_collision_footprints()
{
  invalidate_certification_broadphase(certification_workspace_);
  certified_footprint_ = costmap_ros_->getRobotFootprint();
  if (certification_enabled_) {
    nav2_costmap_2d::padFootprint(
      certified_footprint_, minimum_certified_margin_);
  }
  initial_overlap_core_footprint_ = costmap_ros_->getRobotFootprint();
  if (!enable_initial_overlap_recovery_) {
    return;
  }
  nav2_costmap_2d::padFootprint(
    initial_overlap_core_footprint_, -initial_overlap_footprint_inset_);
  double twice_area = 0.0;
  for (std::size_t index = 0u;
    index < initial_overlap_core_footprint_.size(); ++index)
  {
    const auto & first = initial_overlap_core_footprint_[index];
    const auto & second = initial_overlap_core_footprint_[
      (index + 1u) % initial_overlap_core_footprint_.size()];
    if (!std::isfinite(first.x) || !std::isfinite(first.y)) {
      throw nav2_core::ControllerException(
              "Initial-overlap inset produced a non-finite footprint");
    }
    twice_area += first.x * second.y - second.x * first.y;
  }
  if (initial_overlap_core_footprint_.size() < 3u ||
    std::abs(twice_area) <= 1.0e-9)
  {
    throw nav2_core::ControllerException(
            "Initial-overlap inset collapses the planning footprint");
  }
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

void CertifiedDWBLocalPlanner::reset_trial_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(controller_state_mutex_);
  report_planning_metrics("trial_end");
  planning_durations_seconds_.clear();
  planning_cycle_count_ = 0;
  planning_deadline_miss_count_ = 0;
  maximum_planning_duration_seconds_ = 0.0;
  certification_rejections_ = CertificationRejectionCounters();
  {
    std::lock_guard<std::mutex> diagnostic_lock(
      diagnostic_publication_mutex_);
    diagnostic_publication_count_ = 0u;
    full_evaluation_publication_count_ = 0u;
    candidate_marker_publication_count_ = 0u;
    deferred_full_evaluation_count_ = 0u;
    coalesced_stale_marker_count_ = 0u;
    maximum_diagnostic_backlog_ = diagnostic_publications_.size();
    maximum_diagnostic_publication_seconds_ = 0.0;
  }
  retained_backup_commands_.clear();
  retained_backup_states_.clear();
  terminal_stop_goal_capture_active_ = false;
  terminal_stop_goal_capture_committed_ = false;
  global_plan_.poses.clear();
  terminal_reference_plan_.poses.clear();
  current_progress_reference_plan_.poses.clear();
  current_goal_pose_valid_ = false;
  current_terminal_path_heading_valid_ = false;
  current_terminal_distance_target_pose_valid_ = false;
  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }

  auto native_generator =
    std::dynamic_pointer_cast<NativeInputTrajectoryGenerator>(
    traj_generator_);
  if (native_generator) {
    native_generator->reset_trial_state();
  } else if (traj_generator_) {
    traj_generator_->reset();
  }
  {
    std::lock_guard<std::mutex> command_lock(command_state_mutex_);
    pending_issued_commands_.clear();
    dispatched_command_ = nav_2d_msgs::msg::Twist2D();
    dispatched_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dispatched_command_time_valid_ = false;
    // The transport reset service is called after this controller reset. Do
    // not certify a new epoch until its observable reset dispatch and valid
    // state have both arrived.
    command_dispatch_observed_ = false;
    command_transport_valid_ = false;
    command_ledger_valid_ = false;
    expected_dispatch_sequence_ready_ = false;
    expected_dispatch_sequence_ = 0;
  }

  response->success = true;
  response->message =
    "controller trial state cleared; awaiting observable transport reset";
  RCLCPP_INFO(logger_, "%s", response->message.c_str());
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::CertifiedDWBLocalPlanner,
  nav2_core::Controller)
