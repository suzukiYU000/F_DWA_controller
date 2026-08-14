/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__FIXED_DISTANCE_RISK_PATH_HPP_
#define F_DWA_CONTROLLER__FIXED_DISTANCE_RISK_PATH_HPP_

#include <cstddef>
#include <vector>

#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace f_dwa_controller
{

struct RiskPathSample
{
  geometry_msgs::msg::Pose2D pose;
  double arc_length{0.0};
};

class RiskPathWorkspace;

/**
 * @brief Immutable geometry prepared once for plan-continuation scoring.
 *
 * The implementation owns a snapshot of the plan and its derived segment and
 * vertex geometry. Keeping the representation private prevents callers from
 * creating a partially updated cache; rebuild it with
 * prepare_plan_continuation_geometry() whenever the transformed plan changes.
 */
class PreparedPlanGeometry
{
public:
  bool empty() const noexcept
  {
    return poses_.empty();
  }

private:
  struct Segment
  {
    double delta_x{0.0};
    double delta_y{0.0};
    double length{0.0};
    double heading{0.0};
    double heading_sine{0.0};
    double heading_cosine{1.0};
    double start_progress{0.0};
  };

  std::vector<geometry_msgs::msg::Pose2D> poses_;
  std::vector<Segment> segments_;
  std::vector<double> vertex_headings_;
  std::size_t last_moving_segment_{0u};
  bool has_plan_motion_{false};

  friend PreparedPlanGeometry prepare_plan_continuation_geometry(
    const nav_2d_msgs::msg::Path2D & global_plan);

  friend std::vector<RiskPathSample> build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const PreparedPlanGeometry & plan_geometry,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance);

  friend const std::vector<RiskPathSample> &
  build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const PreparedPlanGeometry & plan_geometry,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance,
    RiskPathWorkspace & workspace);
};

/**
 * @brief Per-critic reusable storage for fixed-distance risk-path scoring.
 *
 * A workspace is intentionally not shared between critics or Controller
 * instances. Each successful build overwrites the previous logical contents
 * while retaining vector capacity. The reference returned by the workspace
 * overloads remains valid only until that same workspace is used again.
 */
class RiskPathWorkspace
{
private:
  dwb_msgs::msg::Trajectory2D combined_trajectory_;
  std::vector<double> cumulative_distance_;
  std::vector<double> target_distances_;
  std::vector<RiskPathSample> samples_;

  friend std::vector<RiskPathSample> build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const nav_2d_msgs::msg::Path2D & global_plan,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance);

  friend std::vector<RiskPathSample> build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const PreparedPlanGeometry & plan_geometry,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance);

  friend const std::vector<RiskPathSample> &
  build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const nav_2d_msgs::msg::Path2D & global_plan,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance,
    RiskPathWorkspace & workspace);

  friend const std::vector<RiskPathSample> &
  build_plan_continued_risk_path(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const PreparedPlanGeometry & plan_geometry,
    double risk_distance,
    double sample_resolution,
    double risk_seed_time,
    double heading_relaxation_distance,
    RiskPathWorkspace & workspace);
};

/**
 * @brief Validate and precompute geometry shared by every candidate.
 *
 * @throws std::invalid_argument if the plan is empty, contains a non-finite
 * pose or segment, or its cumulative length cannot be represented.
 */
PreparedPlanGeometry prepare_plan_continuation_geometry(
  const nav_2d_msgs::msg::Path2D & global_plan);

/**
 * @brief Resample a candidate on one common spatial risk horizon.
 *
 * The returned samples always span [0, risk_distance]. If the nominal
 * polyline is shorter, its terminal spatial curvature is continued as a soft
 * terminal-value probe. This makes a constant-curvature candidate describe
 * the same fixed-distance path when only its temporal horizon changes. A
 * stationary or pure-rotation candidate has no spatial curvature, so it uses
 * one common straight probe from the initial pose and heading instead of a
 * horizon-dependent terminal yaw. This extension is never used for hard
 * collision rejection.
 *
 * @throws std::invalid_argument if the trajectory is empty, contains a
 * non-finite pose, or if either distance argument is invalid.
 */
std::vector<RiskPathSample> build_fixed_distance_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  double risk_distance,
  double sample_resolution);

/**
 * @brief Build the fixed-distance path from a common timed trajectory prefix.
 *
 * Nav2's trajectory generators store the start pose and timed poses using
 * poses.size() == time_offsets.size() + 1. The final extra pose is not paired
 * with a time offset, so it is deliberately excluded when the trajectory is
 * cut at risk_seed_time. The equal-size layout is also accepted for callers
 * which provide one timestamp for every pose.
 *
 * @throws std::invalid_argument if the timing layout is unsupported, the
 * offsets are not finite and strictly increasing from zero, or the timed
 * trajectory does not reach risk_seed_time.
 */
std::vector<RiskPathSample> build_fixed_distance_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  double risk_distance,
  double sample_resolution,
  double risk_seed_time);

/**
 * @brief Continue a common timed candidate prefix along the prepared plan.
 *
 * The candidate is cut at risk_seed_time and its native prefix is preserved.
 * From that endpoint, the continuation advances from the closest forward
 * projection on global_plan while preserving the endpoint's signed lateral
 * offset. A spatially credible candidate heading is held for a bounded 0.50 m
 * maneuver phase, then relaxed back to the plan tangent over
 * heading_relaxation_distance while its lateral displacement is integrated.
 * Candidates without 0.10 m of seed translation retain only their
 * already-achieved lateral offset; pure rotation therefore cannot gain a
 * fictional long-distance bypass. At the end of a finite plan, its final
 * tangent is extended only for this soft risk observation.
 *
 * @throws std::invalid_argument for the timed-prefix errors above, an empty or
 * non-finite plan, or a plan which cannot provide a finite continuation.
 */
std::vector<RiskPathSample> build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const nav_2d_msgs::msg::Path2D & global_plan,
  double risk_distance,
  double sample_resolution,
  double risk_seed_time,
  double heading_relaxation_distance = 1.0);

/**
 * @brief Workspace-backed Path2D compatibility overload.
 *
 * The returned samples are numerically identical to the value-return API and
 * are overwritten by the next call using the same workspace.
 */
const std::vector<RiskPathSample> & build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const nav_2d_msgs::msg::Path2D & global_plan,
  double risk_distance,
  double sample_resolution,
  double risk_seed_time,
  double heading_relaxation_distance,
  RiskPathWorkspace & workspace);

/**
 * @brief Continue a candidate using plan geometry prepared by the critic.
 *
 * This overload is equivalent to the Path2D overload, but performs no plan
 * validation, segment-length calculation, or vertex-heading calculation per
 * candidate. A default-constructed geometry is rejected fail-closed.
 */
std::vector<RiskPathSample> build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const PreparedPlanGeometry & plan_geometry,
  double risk_distance,
  double sample_resolution,
  double risk_seed_time,
  double heading_relaxation_distance = 1.0);

/**
 * @brief Workspace-backed prepared-plan overload used by realtime critics.
 *
 * It performs the same validation and arithmetic in the same order as the
 * value-return overload, but reuses candidate-prefix and resampling buffers.
 */
const std::vector<RiskPathSample> & build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const PreparedPlanGeometry & plan_geometry,
  double risk_distance,
  double sample_resolution,
  double risk_seed_time,
  double heading_relaxation_distance,
  RiskPathWorkspace & workspace);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FIXED_DISTANCE_RISK_PATH_HPP_
