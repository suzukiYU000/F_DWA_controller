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
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "f_dwa_controller/footprint_clearance_critic.hpp"
#include "f_dwa_controller/issued_command_ledger.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"
#include "f_dwa_controller/native_input_trajectory_generator.hpp"
#include "f_dwa_controller/planning_snapshot.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "f_dwa_controller/velocity_response_model.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace f_dwa_controller
{

class CertifiedDWBLocalPlanner : public dwb_core::DWBLocalPlanner
{
public:
  CertifiedDWBLocalPlanner() = default;
  ~CertifiedDWBLocalPlanner() override;

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
  struct TerminalStopAssessment
  {
    bool available{false};
    bool captures_goal{false};
    bool crosses_terminal_limit{false};
    double position_error{0.0};
    double yaw_error{0.0};
    double longitudinal_error{0.0};
    double lateral_error{0.0};
    geometry_msgs::msg::Pose2D terminal_pose;
  };

  struct ProgressEscapeRank
  {
    uint64_t clearance_guard_bucket{std::numeric_limits<uint64_t>::max()};
    bool has_translation_progress{false};
    double progress_cost{std::numeric_limits<double>::infinity()};
    double path_deviation_cost{std::numeric_limits<double>::infinity()};
    uint64_t mean_path_distance_bucket{
      std::numeric_limits<uint64_t>::max()};
    double mean_path_distance_cost{std::numeric_limits<double>::infinity()};
    bool consumes_uncertainty_reserve{false};
    bool recovers_initial_clearance{false};
    double approach_risk{std::numeric_limits<double>::infinity()};
    uint64_t clearance_risk_bucket{std::numeric_limits<uint64_t>::max()};
    double avoidance_horizon_seconds{0.0};
    std::size_t canonical_index{std::numeric_limits<std::size_t>::max()};
  };

  struct DiagnosticPublication
  {
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> evaluation;
    bool publish_full_evaluation{false};
    bool publish_candidate_markers{false};
  };

  static constexpr std::size_t kMaximumPendingFullEvaluations = 2u;

  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override;

  void score_trajectory_components(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double best_score,
    dwb_msgs::msg::TrajectoryScore & score,
    bool record_score_details,
    bool * used_reserve_recovery = nullptr,
    std::optional<double> precomputed_clearance_risk = std::nullopt,
    std::optional<double> precomputed_clearance_trigger_risk = std::nullopt,
    std::optional<double> precomputed_clearance_guard_risk = std::nullopt,
    double * clearance_risk = nullptr,
    const std::vector<geometry_msgs::msg::Pose2D> *
    precomputed_stop_poses = nullptr,
    bool * completed_weighted_score = nullptr,
    double * path_deviation_cost = nullptr,
    double * mean_path_distance_cost = nullptr,
    bool * strict_physical_stop_safety_proven = nullptr,
    double * guard_clearance_risk = nullptr,
    double * guard_approach_risk = nullptr);
  visualization_msgs::msg::MarkerArray build_candidate_markers(
    const dwb_msgs::msg::LocalPlanEvaluation & evaluation) const;
  static bool coalesce_stale_marker_publication(
    std::deque<DiagnosticPublication> & publications,
    DiagnosticPublication publication);
  static uint64_t clearance_constraint_bucket(
    double clearance_risk,
    double admissible_risk,
    double risk_resolution);
  static double zero_scale_clearance_diagnostic_score(
    bool is_clearance_constraint_critic,
    bool is_clearance_constraint_trigger_critic,
    bool is_clearance_constraint_guard_critic,
    std::optional<double> precomputed_clearance_risk,
    std::optional<double> precomputed_clearance_trigger_risk,
    std::optional<double> precomputed_clearance_guard_risk);
  static bool should_score_terminal_stop(
    bool certification_enabled,
    bool stop_admissibility_enabled,
    double goal_distance_scale,
    bool target_pose_valid);
  static std::optional<double> fused_clearance_risk(
    std::optional<double> primary_risk,
    std::optional<double> guard_risk);
  static bool trajectory_has_meaningful_subgoal_progress(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const nav_2d_msgs::msg::Path2D & path,
    const geometry_msgs::msg::Pose2D & subgoal,
    double minimum_distance_progress,
    double minimum_heading_progress);
  static bool trajectory_has_observable_motion(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double minimum_translation,
    double minimum_rotation);
  static bool pose_sequence_has_observable_rotation(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    double minimum_rotation);
  static bool pose_sequence_has_observable_translation(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    double minimum_translation);
  static bool receding_horizon_progress_is_executable(
    bool stop_has_translation_progress,
    bool rollout_has_translation_progress,
    bool stop_has_heading_progress,
    bool rollout_has_heading_progress,
    bool stop_has_observable_heading_motion);
  static bool preserves_established_turn_direction(
    double candidate_angular_velocity,
    double established_angular_velocity);
  nav_2d_msgs::msg::Path2D transformGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose) override;
  static bool terminal_plan_fallback_is_applicable(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & terminal_pose,
    double capture_distance);
  static bool terminal_goal_hold_is_applicable(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & goal_pose,
    double capture_distance,
    const nav_2d_msgs::msg::Twist2D & velocity,
    double stop_velocity_threshold);
  static bool terminal_goal_resume_is_applicable(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Pose2D & goal_pose,
    double capture_distance,
    const nav_2d_msgs::msg::Twist2D & velocity,
    double stop_velocity_threshold);
  static bool clearance_constraint_prefers_candidate(
    bool candidate_has_meaningful_progress,
    bool best_has_meaningful_progress,
    uint64_t candidate_guard_risk_bucket,
    uint64_t best_guard_risk_bucket,
    uint64_t candidate_risk_bucket,
    uint64_t best_risk_bucket,
    double candidate_total,
    double best_total,
    std::size_t candidate_canonical_index,
    std::size_t best_canonical_index);
  static bool clearance_constraint_is_active_for_pair(
    bool clearance_constraint_enabled,
    double best_total,
    uint64_t candidate_trigger_risk_bucket,
    uint64_t best_trigger_risk_bucket);
  static TerminalStopAssessment assess_terminal_stop(
    const std::vector<geometry_msgs::msg::Pose2D> & stop_poses,
    const geometry_msgs::msg::Pose2D & goal_pose,
    double terminal_path_heading,
    double capture_distance,
    double capture_yaw_tolerance,
    double maximum_overshoot);
  static bool terminal_stop_prefers_candidate(
    bool candidate_captures_goal,
    bool best_captures_goal,
    double candidate_total,
    double best_total,
    std::size_t candidate_canonical_index,
    std::size_t best_canonical_index);
  static bool receding_horizon_recovery_prefers_candidate(
    double candidate_collision_time,
    double best_collision_time,
    double candidate_clearance_risk,
    double best_clearance_risk,
    double candidate_path_departure_cost,
    double best_path_departure_cost,
    std::size_t candidate_canonical_index,
    std::size_t best_canonical_index);
  static bool recovery_candidate_preserves_uncertainty_reserve(
    double collision_time,
    uint64_t clearance_guard_bucket,
    double approach_risk,
    double maximum_approach_risk,
    double minimum_collision_horizon);
  static double uncertainty_reserve_approach_limit(
    bool recovers_initial_clearance);
  static bool consumes_uncertainty_reserve(
    double initial_clearance,
    double terminal_clearance,
    double uncertainty_margin,
    double tolerance);
  static bool progress_escape_prefers_candidate(
    const ProgressEscapeRank & candidate,
    const ProgressEscapeRank & best);
  static bool progress_escape_should_replace_weighted_winner(
    bool candidate_found,
    bool selected_progress_was_evaluated,
    bool selected_has_receding_horizon_progress);
  static bool legal_avoidance_escape_prefers_candidate(
    uint64_t candidate_guard_risk_bucket,
    uint64_t best_guard_risk_bucket,
    uint64_t candidate_risk_bucket,
    uint64_t best_risk_bucket,
    double candidate_approach_risk,
    double best_approach_risk,
    bool candidate_preserves_turn_direction,
    bool best_preserves_turn_direction,
    double candidate_heading_excursion,
    double best_heading_excursion,
    double candidate_translation_distance,
    double best_translation_distance,
    std::size_t candidate_canonical_index,
    std::size_t best_canonical_index);
  static double predicted_collision_time(
    const CertificationResult & result,
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    double maximum_swept_distance,
    double control_period);
  static double predicted_collision_time_from_obstacle_rejection(
    const std::string & rejection_detail,
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    double maximum_swept_distance,
    double control_period);
  static bool has_full_evaluation_capacity(
    std::size_t pending_full_evaluation_count);

private:
  enum class TerminalStopScoreMode
  {
    kGlobalGoalDistance,
    kPathSubgoalDistance,
    kPathSubgoalProgress,
    kPathSubgoalForwardRay
  };

  using IssuedCommand = IssuedCommandLedgerEntry;

  struct CertificationRejectionCounters
  {
    uint64_t terminal_stop_infeasible{0};
    uint64_t invalid_input{0};
    uint64_t off_costmap{0};
    uint64_t lethal_obstacle{0};
    uint64_t unknown_space{0};
  };

  struct DurationCounter
  {
    uint64_t calls{0u};
    double total_seconds{0.0};
  };

  class ScopedDuration
  {
public:
    ScopedDuration(bool enabled, DurationCounter & counter);
    ~ScopedDuration();

    ScopedDuration(const ScopedDuration &) = delete;
    ScopedDuration & operator=(const ScopedDuration &) = delete;

private:
    DurationCounter * counter_{nullptr};
    std::chrono::steady_clock::time_point started_at_;
  };

  void command_dispatch_callback(
    const f_dwa_controller::msg::CommandDispatch::SharedPtr message);
  void transport_valid_callback(
    const std_msgs::msg::Bool::SharedPtr message);
  void request_transport_invalidation(const char * reason);
  std::shared_ptr<const PlanningSnapshot> build_planning_snapshot(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & observed_velocity);
  void record_issued_command(
    const geometry_msgs::msg::TwistStamped & command,
    const rclcpp::Time & issued_at,
    uint64_t issued_steady_time_ns);
  void record_planning_duration(
    std::chrono::steady_clock::time_point started_at);
  void record_certification_rejection(CertificationFailure failure);
  void report_planning_metrics(const char * scope);
  void reset_detailed_timing_metrics();
  bool should_publish_evaluation();
  void enqueue_diagnostic_publication(
    const std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & evaluation,
    bool publish_full_evaluation,
    bool publish_candidate_markers);
  void start_diagnostic_publisher();
  void stop_diagnostic_publisher();
  void diagnostic_publisher_loop();
  void publish_diagnostics_now(const DiagnosticPublication & publication);
  bool should_publish_candidate_markers() const;
  void publish_candidate_markers(
    const dwb_msgs::msg::LocalPlanEvaluation & evaluation);
  void prepare_collision_footprints();
  void prepare_terminal_targets(const geometry_msgs::msg::Pose2D & pose);
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
    CertificationResult * result = nullptr,
    bool * used_reserve_recovery = nullptr) const;
  bool certify_physical_sequence(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    CertificationResult * result = nullptr,
    bool * used_boundary_margin_recovery = nullptr,
    bool allow_persistent_localization_overlap = false) const;
  AxisLimits linear_limits() const;
  AxisLimits angular_limits() const;
  void reset_trial_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::mutex controller_state_mutex_;
  std::mutex command_state_mutex_;
  std::mutex diagnostic_publication_mutex_;
  std::condition_variable diagnostic_publication_condition_;
  std::deque<DiagnosticPublication> diagnostic_publications_;
  std::thread diagnostic_publisher_thread_;
  bool stop_diagnostic_publisher_{true};
  uint64_t diagnostic_publication_count_{0};
  uint64_t full_evaluation_publication_count_{0};
  uint64_t candidate_marker_publication_count_{0};
  uint64_t deferred_full_evaluation_count_{0};
  uint64_t coalesced_stale_marker_count_{0};
  std::size_t pending_full_evaluation_count_{0};
  std::size_t maximum_diagnostic_backlog_{0};
  double maximum_diagnostic_publication_seconds_{0.0};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<
    dwb_msgs::msg::LocalPlanEvaluation>::SharedPtr evaluation_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr candidate_marker_publisher_;
  rclcpp::Subscription<f_dwa_controller::msg::CommandDispatch>::SharedPtr
    command_dispatch_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    transport_valid_subscriber_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    transport_invalidation_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trial_service_;
  bool certification_enabled_{false};
  bool clearance_constraint_enabled_{false};
  std::string clearance_constraint_critic_name_{"FootprintClearance"};
  std::string clearance_constraint_trigger_critic_name_{
    "FootprintClearance"};
  std::string clearance_constraint_guard_critic_name_{"FootprintClearance"};
  double clearance_constraint_admissible_risk_{0.05};
  double clearance_constraint_trigger_risk_{-1.0};
  double clearance_constraint_risk_resolution_{0.01};
  double clearance_constraint_guard_admissible_risk_{1.0};
  double clearance_constraint_guard_risk_resolution_{0.01};
  double clearance_constraint_minimum_subgoal_distance_progress_{0.0};
  double clearance_constraint_minimum_subgoal_heading_progress_{0.0};
  double clearance_constraint_motion_preference_goal_distance_{0.0};
  bool clearance_constraint_include_footprint_approach_{false};
  double clearance_constraint_footprint_approach_trigger_risk_{0.1};
  std::shared_ptr<FootprintClearanceCritic> clearance_constraint_critic_;
  dwb_core::TrajectoryCritic::Ptr clearance_constraint_trigger_critic_;
  std::shared_ptr<FootprintClearanceCritic>
  clearance_constraint_trigger_footprint_critic_;
  dwb_core::TrajectoryCritic::Ptr clearance_constraint_guard_critic_;
  std::shared_ptr<FootprintClearanceCritic>
  clearance_constraint_guard_footprint_critic_;
  bool no_valid_control_deceleration_fallback_enabled_{false};
  bool stop_admissibility_enabled_{false};
  bool nominal_delay_preview_enabled_{true};
  bool use_observed_velocity_for_activation_state_{false};
  bool velocity_response_prediction_enabled_{false};
  bool require_command_dispatch_state_{true};
  bool allow_safety_command_reduction_{false};
  bool command_dispatch_observed_{false};
  bool command_transport_valid_{false};
  bool command_ledger_valid_{false};
  bool expected_dispatch_sequence_ready_{false};
  uint64_t expected_dispatch_sequence_{0};
  double nominal_delay_preview_seconds_{0.07};
  double velocity_response_prediction_seconds_{0.12};
  double velocity_response_integration_step_seconds_{0.01};
  AxisVelocityResponseModel linear_velocity_response_model_{0.035, 0.02, 1.0};
  AxisVelocityResponseModel angular_velocity_response_model_{0.015, 0.085, 0.95};
  double certification_control_period_{0.03};
  double terminal_stop_maximum_time_{8.0};
  double terminal_stop_velocity_threshold_{0.01};
  double terminal_stop_goal_distance_scale_{0.0};
  double terminal_stop_path_lookahead_distance_{1.5};
  double terminal_stop_path_lateral_weight_{1.0};
  TerminalStopScoreMode terminal_stop_score_mode_{
    TerminalStopScoreMode::kGlobalGoalDistance};
  double terminal_stop_goal_capture_distance_{0.0};
  double terminal_stop_goal_capture_yaw_tolerance_{0.0};
  double minimum_certified_margin_{0.02};
  double maximum_swept_distance_{0.025};
  bool enable_reserve_recovery_{false};
  bool reserve_recovery_hysteresis_{true};
  bool enable_initial_overlap_recovery_{false};
  double initial_overlap_footprint_inset_{0.05};
  double localization_uncertainty_footprint_inset_{0.0};
  bool enable_transient_boundary_margin_recovery_{false};
  bool transient_boundary_margin_require_clearance_{true};
  double transient_boundary_margin_maximum_overlap_fraction_{0.25};
  double transient_boundary_margin_minimum_clear_suffix_fraction_{0.20};
  double minimum_linear_velocity_{0.0};
  double maximum_linear_velocity_{1.2};
  double maximum_angular_velocity_{1.57};
  double maximum_linear_acceleration_{1.2};
  double maximum_linear_deceleration_{-1.2};
  double maximum_angular_acceleration_{1.57};
  double maximum_angular_deceleration_{-1.57};
  nav_2d_msgs::msg::Twist2D dispatched_command_;
  rclcpp::Time dispatched_command_time_{0, 0, RCL_ROS_TIME};
  bool dispatched_command_time_valid_{false};
  std::deque<IssuedCommand> pending_issued_commands_;
  std::shared_ptr<const PlanningSnapshot> planning_snapshot_;
  bool planning_metrics_enabled_{true};
  int planning_metrics_report_interval_{1000};
  double planning_deadline_seconds_{0.03};
  bool publish_evaluation_{true};
  bool publish_candidate_markers_{true};
  bool record_full_evaluation_details_{false};
  double evaluation_publish_frequency_{0.0};
  rclcpp::Time last_evaluation_publish_time_{0, 0, RCL_ROS_TIME};
  bool has_evaluation_publish_time_{false};
  std::vector<double> planning_durations_seconds_;
  uint64_t planning_cycle_count_{0};
  uint64_t planning_deadline_miss_count_{0};
  double maximum_planning_duration_seconds_{0.0};
  uint64_t shared_clearance_risk_path_candidate_count_{0};
  uint64_t independent_clearance_risk_path_candidate_count_{0};
  DurationCounter core_scoring_timing_;
  DurationCounter candidate_evaluation_timing_;
  DurationCounter trajectory_generation_timing_;
  DurationCounter stop_trajectory_generation_timing_;
  DurationCounter clearance_constraint_timing_;
  DurationCounter clearance_primary_timing_;
  DurationCounter clearance_trigger_timing_;
  DurationCounter clearance_guard_timing_;
  DurationCounter weighted_scoring_timing_;
  DurationCounter stop_safety_timing_;
  std::vector<DurationCounter> critic_scoring_timings_;
  CertificationRejectionCounters certification_rejections_;
  std::vector<nav_2d_msgs::msg::Twist2D> retained_backup_commands_;
  std::vector<NativeInputTrajectoryGenerator::NativeCommandState>
  retained_backup_states_;
  bool terminal_stop_goal_capture_active_{false};
  bool terminal_stop_goal_capture_committed_{false};
  std::vector<geometry_msgs::msg::Point> certified_footprint_;
  std::vector<geometry_msgs::msg::Point> initial_overlap_core_footprint_;
  std::vector<geometry_msgs::msg::Point> localization_core_footprint_;
  mutable CertificationWorkspace certification_workspace_;
  mutable ObservationLayerCertificationWorkspace
    observation_layer_certification_workspace_;
  std::vector<geometry_msgs::msg::Pose2D> stop_pose_scratch_;
  geometry_msgs::msg::Pose2D current_goal_pose_;
  bool current_goal_pose_valid_{false};
  nav_2d_msgs::msg::Path2D terminal_reference_plan_;
  nav_2d_msgs::msg::Path2D current_progress_reference_plan_;
  double current_terminal_path_heading_{0.0};
  bool current_terminal_path_heading_valid_{false};
  geometry_msgs::msg::Pose2D current_terminal_distance_target_pose_;
  bool current_terminal_distance_target_pose_valid_{false};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__CERTIFIED_DWB_LOCAL_PLANNER_HPP_
