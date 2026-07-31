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
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "f_dwa_controller/fir_input_dynamics.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"
#include "f_dwa_controller/planning_snapshot.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f_dwa_controller
{

enum class NativeInputOrder
{
  kAcceleration,
  kJerk,
  kFir
};

class NativeInputTrajectoryGenerator
  : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  struct NativeCommandState
  {
    nav_2d_msgs::msg::Twist2D command_velocity;
    AxisState linear_state;
    AxisState angular_state;
    std::vector<double> linear_fir_history;
    std::vector<double> angular_fir_history;
    bool valid{false};
  };

  explicit NativeInputTrajectoryGenerator(NativeInputOrder input_order);
  ~NativeInputTrajectoryGenerator() override = default;

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name) override;
  void reset() override;
  // Trial reset is called only after the simulator and delayed transport have
  // been stopped and reset, so the applied state is known to be exactly zero.
  void reset_trial_state();
  void enrich_planning_snapshot(PlanningSnapshot & snapshot) const;
  void set_planning_snapshot(
    std::shared_ptr<const PlanningSnapshot> snapshot);
  void observe_command_dispatch(
    const f_dwa_controller::msg::CommandDispatch & dispatch);
  [[nodiscard]] std::optional<NativeCommandState>
  active_candidate_command_state() const;
  [[nodiscard]] std::optional<std::size_t>
  active_candidate_canonical_index() const;
  void select_command_for_dispatch(
    const std::optional<NativeCommandState> & command_state);
  void commit_selected_command(
    const nav_2d_msgs::msg::Twist2D & command,
    const rclcpp::Time & issued_at);
  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;
  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_velocity,
    const nav_2d_msgs::msg::Twist2D & command_velocity) override;
  void generate_trajectory_into(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & command_velocity,
    dwb_msgs::msg::Trajectory2D & trajectory);
  bool generate_stop_trajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
    std::vector<NativeCommandState> * command_states = nullptr);
  bool generate_stop_trajectory_for_candidate(
    std::size_t canonical_index,
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> & velocities,
    std::vector<NativeCommandState> * command_states = nullptr);
  bool generate_stop_poses(
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses);
  bool generate_stop_poses_for_candidate(
    std::size_t canonical_index,
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses);

protected:
  struct AxisStopCache
  {
    int maximum_stop_steps{0};
    double stop_velocity_threshold{0.0};
    std::vector<AxisState> delayed_states;
    StopSequence stop_sequence;
    bool computed{false};
    bool valid{false};
  };

  struct AngularPoseIntegrationStep
  {
    double heading_cosine{0.0};
    double heading_sine{0.0};
    double theta_after_step{0.0};
  };

  struct AngularPoseIntegrationCache
  {
    double start_theta{0.0};
    std::vector<AngularPoseIntegrationStep> steps;
    bool valid{false};
  };

  struct AxisRollout
  {
    std::vector<AxisState> states;
    std::vector<double> first_fir_history;
    double native_input{0.0};
    bool valid{false};
    mutable AxisStopCache stop_cache;
    mutable AngularPoseIntegrationCache pose_integration_cache;
    mutable AngularPoseIntegrationCache stop_pose_integration_cache;
  };

  struct Candidate
  {
    std::size_t canonical_index{0};
    nav_2d_msgs::msg::Twist2D command_velocity;
    double linear_native_input{0.0};
    double angular_native_input{0.0};
    NativeCommandState first_command_state;
    std::shared_ptr<const AxisRollout> linear_rollout;
    std::shared_ptr<const AxisRollout> angular_rollout;
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
  struct PendingNativeCommand
  {
    rclcpp::Time issued_at;
    NativeCommandState state;
  };

  AxisLimits linear_limits() const;
  AxisLimits angular_limits() const;
  void validate_parameters() const;
  const std::vector<AngularPoseIntegrationStep> &
  angular_pose_integration_steps(
    const AxisRollout & angular_rollout,
    double start_theta,
    const std::vector<double> & time_steps) const;
  const std::vector<AngularPoseIntegrationStep> &
  angular_stop_pose_integration_steps(
    const AxisRollout & angular_rollout,
    const AxisStopCache & angular_stop_cache,
    double start_theta,
    std::size_t stop_step_count) const;
  const AxisStopCache & get_axis_stop_cache(
    const AxisRollout & rollout,
    const AxisState & initial_state,
    const AxisLimits & limits,
    const std::vector<double> & initial_fir_history,
    int rollout_step_count,
    int command_delay_steps,
    int maximum_stop_steps,
    double stop_velocity_threshold);
  bool generate_candidate_stop_trajectory(
    const Candidate & candidate,
    const geometry_msgs::msg::Pose2D & start_pose,
    int maximum_stop_steps,
    double stop_velocity_threshold,
    std::vector<geometry_msgs::msg::Pose2D> & poses,
    std::vector<nav_2d_msgs::msg::Twist2D> * velocities,
    std::vector<NativeCommandState> * command_states);

  NativeInputOrder input_order_;
  std::string plugin_name_;
  double control_period_{0.03};
  double maximum_linear_jerk_{1.57};
  double maximum_angular_jerk_{1.57};
  double maximum_linear_raw_input_{1.2};
  double maximum_angular_raw_input_{1.57};
  double fir_prediction_pulse_duration_{0.0};
  double stop_capture_velocity_{0.01};
  double stop_command_delay_seconds_{0.07};
  int linear_samples_{11};
  int angular_samples_{11};
  bool fir_coefficients_generated_{false};
  bool require_applied_command_state_{false};
  bool prefer_previous_selected_candidate_{false};
  std::vector<double> fixed_time_steps_;

  std::vector<Candidate> candidates_;
  std::vector<std::size_t> candidate_order_;
  std::size_t candidate_index_{0};
  const Candidate * active_candidate_{nullptr};
  double iteration_initial_linear_velocity_{0.0};
  double iteration_initial_angular_velocity_{0.0};
  double iteration_initial_linear_acceleration_{0.0};
  double iteration_initial_angular_acceleration_{0.0};
  std::vector<double> iteration_initial_linear_fir_history_;
  std::vector<double> iteration_initial_angular_fir_history_;

  mutable std::mutex applied_command_mutex_;
  nav_2d_msgs::msg::Twist2D latest_applied_command_;
  rclcpp::Time applied_dispatch_time_;
  NativeCommandState applied_native_state_;
  std::vector<double> fir_coefficients_;
  FirStopCoefficientResponse fir_stop_coefficient_response_;
  bool applied_command_state_ready_{false};
  std::deque<PendingNativeCommand> pending_native_commands_;
  std::optional<NativeCommandState> selected_command_state_;
  std::optional<nav_2d_msgs::msg::Twist2D> previous_selected_velocity_;
  std::shared_ptr<const PlanningSnapshot> planning_snapshot_;
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

class FirTrajectoryGenerator
  : public NativeInputTrajectoryGenerator
{
public:
  FirTrajectoryGenerator();
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__NATIVE_INPUT_TRAJECTORY_GENERATOR_HPP_
