/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#ifndef F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_
#define F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_

#include <cstddef>
#include <vector>

#include "dwb_critics/obstacle_footprint.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"

namespace f_dwa_controller
{

/**
 * @brief Hard-check the complete nominal rollout with the physical footprint.
 *
 * Every generated pose and every swept segment is checked through Nav2's
 * ObstacleFootprintCritic. A collision throws IllegalTrajectoryException;
 * every legal trajectory returns a neutral raw score of zero. Soft obstacle
 * ranking belongs to ForwardObstacle, FootprintClearance, and BaseObstacle.
 * score_time_horizon remains a validated compatibility parameter only and no
 * longer truncates either the hard check or a soft score.
 */
class HorizonObstacleFootprintCritic
  : public dwb_critics::ObstacleFootprintCritic
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
  void setDetailedFailureDiagnostics(bool enabled) noexcept;
  void setSharedCertificationWorkspace(
    CertificationWorkspace * workspace) noexcept;

protected:
  bool prepareCertificationBroadphaseIfNeeded();

  // Deprecated compatibility parameter. It has no scoring or gating effect.
  double score_time_horizon_{1.25};
  double maximum_swept_distance_{0.025};
  double footprint_radius_{0.0};
  bool enable_initial_overlap_recovery_{false};
  double initial_overlap_footprint_inset_{0.05};
  double initial_overlap_recovery_penalty_{1000000.0};
  std::vector<geometry_msgs::msg::Point> inset_core_footprint_;
  CertificationWorkspace certification_workspace_;
  CertificationWorkspace * shared_certification_workspace_{nullptr};
  bool certification_workspace_prepared_{false};
  bool detailed_failure_diagnostics_{true};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__HORIZON_OBSTACLE_FOOTPRINT_CRITIC_HPP_
