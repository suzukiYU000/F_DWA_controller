/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_
#define F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "f_dwa_controller/fixed_distance_risk_path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace f_dwa_controller
{

/**
 * @brief Softly penalize obstacles within bands outside the physical footprint.
 *
 * ObstacleFootprintCritic remains the independent hard collision gate.  This
 * critic never rejects a trajectory: it only gives a bounded [0, 1] cost when
 * a lethal costmap cell lies within the configured clearance margin. The
 * penalty varies continuously with clearance instead of quantizing candidate
 * differences into the diagnostic footprint bands. prepare() refreshes a
 * linear-time Euclidean distance field from the dynamic lethal source after
 * bounded static-cell exclusion, reusing it only while the exact input mask and
 * grid geometry remain unchanged. Scoring queries that field along footprint
 * boundary probes instead of filling every expanded polygon. The conservative
 * cell/probe correction prevents obstacles between probes from disappearing.
 * The distance-normalized exposure integral over a fixed-distance risk path is
 * blended with the peak exposure. The integral preserves an escape gradient
 * when the current pose is already inside the margin, while the peak responds
 * early. The nominal seed is cut at risk_seed_time and continued along the
 * prepared transformed plan. Its spatial heading relaxes over
 * heading_relaxation_distance so it does not inherit a longer sim_time suffix
 * or erase the lateral separation of an early avoidance turn.
 */
class FootprintClearanceCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory) override;

  /**
   * @brief Score clearance and report whether the footprint moves closer.
   *
   * The returned score is identical to scoreTrajectory(). approach_risk is the
   * bounded [0, 1] loss of signed conservative clearance from the best point
   * already reached along the executable prefix. A constant parallel-wall
   * clearance therefore reports zero, while deeper entry remains visible even
   * after the ordinary soft score has saturated at one.
   */
  double scoreTrajectoryWithApproachRisk(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double * approach_risk);

  /**
   * @brief Whether two critics build the same fixed-distance risk path.
   *
   * This deliberately compares every path-construction input exactly.  It
   * does not compare either critic's Costmap or clearance parameters because
   * those affect scoring after the shared path has been built.
   */
  bool hasEquivalentRiskPathDefinition(
    const FootprintClearanceCritic & other) const noexcept;

  /**
   * @brief Build this critic's risk path in its reusable workspace.
   *
   * The returned reference remains valid until this critic builds another
   * risk path.  It can be scored by another critic only when
   * hasEquivalentRiskPathDefinition() is true.
   */
  const std::vector<RiskPathSample> & buildRiskPath(
    const dwb_msgs::msg::Trajectory2D & trajectory);

  /**
   * @brief Score an already-built equivalent risk path.
   *
   * Candidate-prefix approach risk is still calculated with this critic's
   * own distance field.  Only path construction is shared.
   */
  double scoreTrajectoryWithPreparedRiskPathAndApproachRisk(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    const std::vector<RiskPathSample> & risk_path,
    double * approach_risk);

  /**
   * @brief Score a uniformly timed, executable pose sequence.
   *
   * This uses the same bounded footprint-clearance penalty as the ordinary
   * trajectory score, but it does not append a soft Path continuation.  It is
   * intended for complete method-native stop sequences sampled at the
   * Controller certification period, so angular and braking inertia are part
   * of the clearance ranking.  The result blends peak exposure with the
   * time-sample mean and remains in [0, 1].
   */
  double scoreUniformPoseSequenceWithApproachRisk(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    double * approach_risk);

  /**
   * @brief Whether a complete executable rollout recovers its initial clearance.
   *
   * This compares signed conservative physical-footprint clearance at the
   * first and final poses with one effective soft margin for the whole
   * rollout. It does not certify collision safety; callers must first pass the
   * independent hard ObstacleFootprint rollout and complete-stop checks. The
   * result lets a hard-legal avoidance rollout enter localization uncertainty
   * temporarily only when its full method-native horizon exits at least as
   * far from obstacles as it started.
   */
  bool trajectoryRecoversInitialClearance(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double tolerance,
    double * initial_clearance = nullptr,
    double * terminal_clearance = nullptr) const;

  /**
   * @brief Whether a uniformly timed executable stop recovers initial clearance.
   *
   * The caller must independently certify the complete sequence with the
   * physical footprint. This endpoint test uses the same speed-dependent soft
   * margin as scoreUniformPoseSequenceWithApproachRisk(), allowing temporary
   * localization-margin exposure only when the method-native stop finishes no
   * closer to an obstacle than it started.
   */
  bool poseSequenceRecoversInitialClearance(
    const std::vector<geometry_msgs::msg::Pose2D> & poses,
    double tolerance,
    double * initial_clearance = nullptr,
    double * terminal_clearance = nullptr) const;

  double localizationUncertaintyMargin() const noexcept
  {
    return localization_uncertainty_margin_;
  }

protected:
  bool excludedByStaticLayer(double world_x, double world_y) const;

  void refreshProjectedExclusionCostmap();

  bool refreshPenalizedCellMask();

  bool refreshFootprintBoundarySamples();

  bool refreshFootprintGeometry(
    const nav2_costmap_2d::Footprint & footprint);

  bool posePenaltyIsProvablyZero(
    const geometry_msgs::msg::Pose2D & pose) const;

  bool posePenaltyIsProvablyZero(
    const geometry_msgs::msg::Pose2D & pose,
    double effective_margin) const;

  virtual double minimumFootprintClearance(
    const geometry_msgs::msg::Pose2D & pose) const;

  virtual bool expandedFootprintHitsLethal(
    const geometry_msgs::msg::Pose2D & pose,
    std::size_t band_index) const;

  double scorePoseClearance(
    const geometry_msgs::msg::Pose2D & pose) const;

  double scorePoseClearance(
    const geometry_msgs::msg::Pose2D & pose,
    double effective_margin) const;

  double scorePoseClearance(
    const geometry_msgs::msg::Pose2D & pose,
    double effective_margin,
    double * directional_clearance) const;

  double scorePoseClearanceWithPreparedPoseCache(
    const geometry_msgs::msg::Pose2D & pose,
    double effective_margin,
    double * directional_clearance = nullptr) const;

  double trajectoryEffectiveMargin(
    const dwb_msgs::msg::Trajectory2D & trajectory) const;

  double uniformSequenceEffectiveMargin(
    const std::vector<geometry_msgs::msg::Pose2D> & poses) const;

  double effectiveMarginForSweepSpeed(double sweep_speed) const;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav2_costmap_2d::Costmap2D * exclusion_costmap_{nullptr};
  nav2_costmap_2d::CostmapLayer * exclusion_costmap_layer_{nullptr};
  nav2_costmap_2d::Costmap2D projected_exclusion_costmap_;
  bool project_exclusion_costmap_{false};
  nav_2d_msgs::msg::Path2D global_plan_;
  PreparedPlanGeometry prepared_plan_geometry_;
  RiskPathWorkspace risk_path_workspace_;
  std::vector<RiskPathSample> native_time_risk_path_workspace_;
  std::string source_layer_;
  std::string exclude_layer_;
  nav2_costmap_2d::Footprint physical_footprint_;
  std::vector<nav2_costmap_2d::Footprint> expanded_footprints_;
  nav2_costmap_2d::Footprint footprint_boundary_samples_;
  std::vector<unsigned char> penalized_cell_mask_;
  std::vector<unsigned char> penalized_cell_mask_scratch_;
  // Exact integer-cell disk used by bounded static-layer exclusion. Reusing
  // it avoids rebuilding the same tolerance geometry for every live lethal
  // cell in every Controller cycle.
  std::vector<std::pair<int, int>> exclusion_tolerance_offsets_;
  double exclusion_offsets_resolution_{
    std::numeric_limits<double>::quiet_NaN()};
  double exclusion_offsets_tolerance_{
    std::numeric_limits<double>::quiet_NaN()};
  bool exclusion_tolerance_active_{false};
  std::vector<float> obstacle_distance_field_;
  // Reused by the separable Euclidean distance transform.  These buffers are
  // scratch only: their logical sizes are reset for every rebuild and no
  // value from an earlier Costmap geometry participates in the result.
  std::vector<double> distance_transform_input_scratch_;
  std::vector<double> distance_transform_output_scratch_;
  std::vector<int> distance_transform_sites_scratch_;
  std::vector<double> distance_transform_boundaries_scratch_;
  std::vector<double> row_distance_scratch_;
  unsigned int distance_field_size_x_{0u};
  unsigned int distance_field_size_y_{0u};
  double distance_field_resolution_{
    std::numeric_limits<double>::quiet_NaN()};
  bool distance_field_has_penalized_cell_{false};
  double maximum_footprint_probe_gap_{0.0};
  double maximum_physical_footprint_radius_{0.0};
  bool footprint_geometry_cache_valid_{false};
  double footprint_geometry_resolution_{
    std::numeric_limits<double>::quiet_NaN()};
  double footprint_geometry_clearance_margin_{
    std::numeric_limits<double>::quiet_NaN()};
  double footprint_geometry_sample_resolution_{
    std::numeric_limits<double>::quiet_NaN()};
  int footprint_geometry_clearance_bands_{0};
  std::uint64_t footprint_geometry_rebuild_count_{0u};
  bool prepared_{false};
  geometry_msgs::msg::Pose2D prepared_pose_;
  double prepared_pose_penalty_{1.0};
  double prepared_pose_directional_clearance_{0.0};
  double prepared_pose_effective_margin_{0.0};
  bool prepared_pose_penalty_valid_{false};
  double clearance_margin_{0.25};
  // Worst-case translational localization error. Clearance inside this band
  // receives maximum soft risk, but remains selectable until the independent
  // physical-footprint critic reports an actual collision.
  double localization_uncertainty_margin_{0.0};
  // The hard footprint remains unchanged. This bounded extra soft margin
  // covers motion during measured transport/localization timing uncertainty.
  double motion_uncertainty_seconds_{0.0};
  double maximum_motion_margin_{0.0};
  double uniform_sequence_period_{0.05};
  double exclude_layer_tolerance_{0.0};
  bool apply_exclude_tolerance_on_aligned_grids_{false};
  double risk_distance_{2.5};
  double risk_seed_time_{1.4};
  // plan_continuation preserves the historical fixed-distance terminal probe.
  // native_time scores only the method-native executable prediction over the
  // common risk_seed_time horizon and never projects it back onto the Path.
  std::string risk_path_mode_{"plan_continuation"};
  double heading_relaxation_distance_{1.0};
  double sample_resolution_{0.10};
  double peak_weight_{0.5};
  double penalty_power_{1.0};
  int clearance_bands_{5};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_
