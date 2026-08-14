/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_
#define F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_

#include <cstddef>
#include <limits>
#include <string>
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
 * exact static-cell exclusion, reusing it only while the exact input mask and
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

protected:
  bool excludedByStaticLayer(double world_x, double world_y) const;

  bool refreshPenalizedCellMask();

  bool refreshFootprintBoundarySamples();

  virtual double minimumFootprintClearance(
    const geometry_msgs::msg::Pose2D & pose) const;

  virtual bool expandedFootprintHitsLethal(
    const geometry_msgs::msg::Pose2D & pose,
    std::size_t band_index) const;

  double scorePoseClearance(
    const geometry_msgs::msg::Pose2D & pose) const;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav2_costmap_2d::Costmap2D * exclusion_costmap_{nullptr};
  nav_2d_msgs::msg::Path2D global_plan_;
  PreparedPlanGeometry prepared_plan_geometry_;
  std::string source_layer_;
  std::string exclude_layer_;
  nav2_costmap_2d::Footprint physical_footprint_;
  std::vector<nav2_costmap_2d::Footprint> expanded_footprints_;
  nav2_costmap_2d::Footprint footprint_boundary_samples_;
  std::vector<unsigned char> penalized_cell_mask_;
  std::vector<unsigned char> penalized_cell_mask_scratch_;
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
  double maximum_footprint_probe_gap_{0.0};
  bool prepared_{false};
  double clearance_margin_{0.25};
  double exclude_layer_tolerance_{0.0};
  double risk_distance_{2.5};
  double risk_seed_time_{1.4};
  double heading_relaxation_distance_{1.0};
  double sample_resolution_{0.10};
  double peak_weight_{0.5};
  double penalty_power_{1.0};
  int clearance_bands_{5};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FOOTPRINT_CLEARANCE_CRITIC_HPP_
