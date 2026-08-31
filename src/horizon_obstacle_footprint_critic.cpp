/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/trajectory_certifier.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void HorizonObstacleFootprintCritic::onInit()
{
  dwb_critics::ObstacleFootprintCritic::onInit();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".score_time_horizon",
    rclcpp::ParameterValue(1.25));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".score_time_horizon",
    score_time_horizon_);
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".maximum_swept_distance",
    rclcpp::ParameterValue(0.5 * costmap_->getResolution()));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".maximum_swept_distance",
    maximum_swept_distance_);
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".enable_initial_overlap_recovery",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".initial_overlap_footprint_inset",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".initial_overlap_recovery_penalty",
    rclcpp::ParameterValue(1000000.0));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".localization_uncertainty_footprint_inset",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".localization_uncertainty_recovery_penalty",
    rclcpp::ParameterValue(2000000.0));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".enable_transient_boundary_margin_recovery",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".transient_boundary_margin_require_clearance",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ +
    ".transient_boundary_margin_maximum_overlap_fraction",
    rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ +
    ".transient_boundary_margin_minimum_clear_suffix_fraction",
    rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + ".transient_boundary_margin_recovery_penalty",
    rclcpp::ParameterValue(10.0));
  node->get_parameter(
    dwb_plugin_name_ + ".enable_initial_overlap_recovery",
    enable_initial_overlap_recovery_);
  node->get_parameter(
    dwb_plugin_name_ + ".initial_overlap_footprint_inset",
    initial_overlap_footprint_inset_);
  node->get_parameter(
    dwb_plugin_name_ + ".initial_overlap_recovery_penalty",
    initial_overlap_recovery_penalty_);
  node->get_parameter(
    dwb_plugin_name_ + ".localization_uncertainty_footprint_inset",
    localization_uncertainty_footprint_inset_);
  node->get_parameter(
    dwb_plugin_name_ + ".localization_uncertainty_recovery_penalty",
    localization_uncertainty_recovery_penalty_);
  node->get_parameter(
    dwb_plugin_name_ + ".enable_transient_boundary_margin_recovery",
    enable_transient_boundary_margin_recovery_);
  node->get_parameter(
    dwb_plugin_name_ + ".transient_boundary_margin_require_clearance",
    transient_boundary_margin_require_clearance_);
  node->get_parameter(
    dwb_plugin_name_ +
    ".transient_boundary_margin_maximum_overlap_fraction",
    transient_boundary_margin_maximum_overlap_fraction_);
  node->get_parameter(
    dwb_plugin_name_ +
    ".transient_boundary_margin_minimum_clear_suffix_fraction",
    transient_boundary_margin_minimum_clear_suffix_fraction_);
  node->get_parameter(
    dwb_plugin_name_ + ".transient_boundary_margin_recovery_penalty",
    transient_boundary_margin_recovery_penalty_);
  if (!std::isfinite(score_time_horizon_) || score_time_horizon_ < 0.0 ||
    !std::isfinite(maximum_swept_distance_) || maximum_swept_distance_ <= 0.0 ||
    !std::isfinite(initial_overlap_footprint_inset_) ||
    initial_overlap_footprint_inset_ <= 0.0 ||
    !std::isfinite(initial_overlap_recovery_penalty_) ||
    initial_overlap_recovery_penalty_ <= 0.0 ||
    !std::isfinite(localization_uncertainty_footprint_inset_) ||
    localization_uncertainty_footprint_inset_ < 0.0 ||
    !std::isfinite(localization_uncertainty_recovery_penalty_) ||
    localization_uncertainty_recovery_penalty_ <= 0.0 ||
    !std::isfinite(
      transient_boundary_margin_maximum_overlap_fraction_) ||
    transient_boundary_margin_maximum_overlap_fraction_ < 0.0 ||
    transient_boundary_margin_maximum_overlap_fraction_ > 1.0 ||
    !std::isfinite(
      transient_boundary_margin_minimum_clear_suffix_fraction_) ||
    transient_boundary_margin_minimum_clear_suffix_fraction_ < 0.0 ||
    transient_boundary_margin_minimum_clear_suffix_fraction_ > 1.0 ||
    !std::isfinite(transient_boundary_margin_recovery_penalty_) ||
    transient_boundary_margin_recovery_penalty_ <= 0.0)
  {
    throw std::runtime_error{
            "HorizonObstacleFootprintCritic parameters must be finite"};
  }
}

bool HorizonObstacleFootprintCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  certification_workspace_prepared_ = false;
  invalidate_observation_layer_certification_workspace(
    observation_layer_certification_workspace_);
  if (!dwb_critics::ObstacleFootprintCritic::prepare(
      pose, velocity, goal, global_plan))
  {
    return false;
  }
  footprint_radius_ = 0.0;
  for (const auto & point : footprint_spec_) {
    footprint_radius_ = std::max(
      footprint_radius_, std::hypot(point.x, point.y));
  }
  inset_core_footprint_ = footprint_spec_;
  nav2_costmap_2d::padFootprint(
    inset_core_footprint_, -initial_overlap_footprint_inset_);
  localization_core_footprint_ = inset_core_footprint_;
  if (localization_uncertainty_footprint_inset_ > 0.0) {
    nav2_costmap_2d::padFootprint(
      localization_core_footprint_,
      -localization_uncertainty_footprint_inset_);
  }
  const auto valid_footprint = [](const auto & footprint) {
      double twice_area = 0.0;
      for (std::size_t index = 0u; index < footprint.size(); ++index) {
        const auto & first = footprint[index];
        const auto & second = footprint[(index + 1u) % footprint.size()];
        if (!std::isfinite(first.x) || !std::isfinite(first.y)) {
          return false;
        }
        twice_area += first.x * second.y - second.x * first.y;
      }
      return footprint.size() >= 3u && std::abs(twice_area) > 1.0e-9;
    };
  if ((enable_initial_overlap_recovery_ ||
    enable_transient_boundary_margin_recovery_) &&
    !valid_footprint(inset_core_footprint_))
  {
    throw std::runtime_error{
            "Initial-overlap inset collapses the planning footprint"};
  }
  if (localization_uncertainty_footprint_inset_ > 0.0 &&
    !valid_footprint(localization_core_footprint_))
  {
    throw std::runtime_error{
            "Localization-uncertainty inset collapses the physical footprint"};
  }
  return std::isfinite(footprint_radius_);
}

bool HorizonObstacleFootprintCritic::prepareCertificationBroadphaseIfNeeded()
{
  if (!certification_workspace_prepared_ && costmap_) {
    certification_workspace_prepared_ = prepare_certification_broadphase(
      *costmap_, certification_workspace_);
  }
  return certification_workspace_prepared_;
}

void HorizonObstacleFootprintCritic::setDetailedFailureDiagnostics(
  const bool enabled) noexcept
{
  detailed_failure_diagnostics_ = enabled;
}

void HorizonObstacleFootprintCritic::setSharedCertificationWorkspace(
  CertificationWorkspace * const workspace) noexcept
{
  shared_certification_workspace_ = workspace;
}

double HorizonObstacleFootprintCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "Trajectory has no poses.");
  }
  const auto append_certification_diagnostics =
    [this](
    std::ostringstream & detail,
    const CertificationResult & result,
    const char * const prefix)
    {
      detail << ';' << prefix << "failure=" <<
        certification_failure_name(result.failure);
      if (!result.has_failure_cell) {
        return;
      }
      detail <<
        ';' << prefix << "cell_x=" << result.failure_cell_x <<
        ';' << prefix << "cell_y=" << result.failure_cell_y <<
        ';' << prefix << "cell_world_x=" << result.failure_cell_world_x <<
        ';' << prefix << "cell_world_y=" << result.failure_cell_world_y <<
        ';' << prefix << "cell_cost=" <<
        static_cast<unsigned int>(result.failure_cell_cost);

      if (!costmap_ros_ || !costmap_ros_->getLayeredCostmap()) {
        return;
      }
      const auto plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
      std::ostringstream layer_costs;
      std::ostringstream hazard_layers;
      bool first_layer = true;
      bool first_hazard_layer = true;
      for (const auto & plugin : *plugins) {
        const auto layer =
          std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(plugin);
        if (!layer) {
          continue;
        }
        unsigned int layer_x = 0u;
        unsigned int layer_y = 0u;
        if (!layer->worldToMap(
            result.failure_cell_world_x,
            result.failure_cell_world_y, layer_x, layer_y))
        {
          continue;
        }
        const unsigned char layer_cost = layer->getCost(layer_x, layer_y);
        layer_costs << (first_layer ? "" : ",") << plugin->getName() << ':' <<
          static_cast<unsigned int>(layer_cost);
        first_layer = false;
        if (layer_cost == nav2_costmap_2d::NO_INFORMATION ||
          layer_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          hazard_layers << (first_hazard_layer ? "" : ",") <<
            plugin->getName() << ':' <<
            static_cast<unsigned int>(layer_cost);
          first_hazard_layer = false;
        }
      }
      if (!first_layer) {
        detail << ';' << prefix << "layer_costs=" << layer_costs.str();
      }
      if (!first_hazard_layer) {
        detail << ';' << prefix << "hazard_layers=" << hazard_layers.str();
      }
    };
  const auto score_pose_with_diagnostics =
    [this, &append_certification_diagnostics](
    const geometry_msgs::msg::Pose2D & checked_pose,
    const std::size_t pose_index,
    const std::size_t subdivision_index)
    {
      // The planner prepared this broadphase under the same locked Costmap
      // snapshot. Empty footprint bounds are a complete safety proof for this
      // pose; an inconclusive query falls through to the unchanged Nav2 exact
      // check, preserving legality and diagnostics at obstacle boundaries.
      if (shared_certification_workspace_ && costmap_ &&
        certification_footprint_bounds_are_hazard_free(
          *costmap_, footprint_spec_, checked_pose,
          *shared_certification_workspace_))
      {
        return;
      }
      try {
        static_cast<void>(scorePose(checked_pose));
      } catch (const dwb_core::IllegalTrajectoryException & exception) {
        std::ostringstream detail;
        detail << std::setprecision(17) << exception.what() <<
          ";pose_index=" << pose_index <<
          ";subdivision=" << subdivision_index;
        if (!detailed_failure_diagnostics_) {
          throw dwb_core::IllegalTrajectoryException(name_, detail.str());
        }
        detail <<
          ";pose_x=" << checked_pose.x <<
          ";pose_y=" << checked_pose.y <<
          ";pose_yaw=" << checked_pose.theta;

        CertificationResult result;
        if (costmap_ && footprint_spec_.size() >= 3u) {
          static_cast<void>(prepareCertificationBroadphaseIfNeeded());
          const std::vector<geometry_msgs::msg::Pose2D> single_pose{
            checked_pose};
          result = certify_pose_sequence(
            *costmap_, footprint_spec_, single_pose,
            maximum_swept_distance_, &certification_workspace_);
        }
        append_certification_diagnostics(detail, result, "");
        throw dwb_core::IllegalTrajectoryException(name_, detail.str());
      }
    };

  try {
    for (std::size_t index = 0u; index < trajectory.poses.size(); ++index) {
      const auto & pose = trajectory.poses[index];
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta))
      {
        throw dwb_core::IllegalTrajectoryException(
                name_, "Trajectory contains a non-finite pose.");
      }

      // Numeric legal-cell cost is intentionally ignored. This critic defines
      // only the physical hard gate; the common soft critics rank legal poses.
      score_pose_with_diagnostics(pose, index, 0u);

      if (index == 0u) {
        continue;
      }
      const auto & previous = trajectory.poses[index - 1u];
      const double delta_x = pose.x - previous.x;
      const double delta_y = pose.y - previous.y;
      const double delta_yaw = std::remainder(
        pose.theta - previous.theta, 2.0 * M_PI);
      const double corner_sweep =
        std::hypot(delta_x, delta_y) +
        footprint_radius_ * std::abs(delta_yaw);
      if (!std::isfinite(corner_sweep)) {
        throw dwb_core::IllegalTrajectoryException(
                name_, "Trajectory sweep is non-finite.");
      }
      const std::size_t subdivisions = std::max<std::size_t>(
        1u, static_cast<std::size_t>(
          std::ceil(corner_sweep / maximum_swept_distance_)));
      constexpr std::size_t kMaximumSubdivisions = 10000u;
      if (subdivisions > kMaximumSubdivisions) {
        throw dwb_core::IllegalTrajectoryException(
                name_, "Trajectory sweep requires too many samples.");
      }
      for (std::size_t subdivision = 1u;
        subdivision < subdivisions; ++subdivision)
      {
        const double ratio = static_cast<double>(subdivision) /
          static_cast<double>(subdivisions);
        geometry_msgs::msg::Pose2D intermediate;
        intermediate.x = previous.x + ratio * delta_x;
        intermediate.y = previous.y + ratio * delta_y;
        intermediate.theta = previous.theta + ratio * delta_yaw;
        // Ignore the legal-cell numeric cost: this sample exists solely to
        // prevent a physical footprint from crossing a lethal/unknown cell
        // between two 50 ms rollout poses.
        score_pose_with_diagnostics(intermediate, index, subdivision);
      }
    }
  } catch (const dwb_core::IllegalTrajectoryException & exception) {
    const std::string_view failure_detail{exception.what()};
    const bool failure_is_at_initial_pose =
      failure_detail.find(";pose_index=0;subdivision=0") !=
      std::string_view::npos;
    if ((enable_initial_overlap_recovery_ ||
      enable_transient_boundary_margin_recovery_ ||
      localization_uncertainty_footprint_inset_ > 0.0) && costmap_)
    {
      static_cast<void>(prepareCertificationBroadphaseIfNeeded());
      double overlap_fraction = 0.0;
      CertificationResult initial_overlap_physical_certificate;
      const bool initial_overlap_certificate_attempted =
        enable_initial_overlap_recovery_ && failure_is_at_initial_pose;
      const bool initial_overlap_recovered =
        initial_overlap_certificate_attempted &&
        certify_initial_overlap_margin_sequence(
          *costmap_, footprint_spec_, inset_core_footprint_, trajectory.poses,
          maximum_swept_distance_, &overlap_fraction,
          &certification_workspace_, false, false, 1.0, 0.0, false,
          &initial_overlap_physical_certificate);
      if (initial_overlap_recovered) {
        return initial_overlap_recovery_penalty_ *
               (1.0 + overlap_fraction);
      }
      overlap_fraction = 0.0;
      ObservationLayerCertificationResult observation_certificate;
      if (localization_uncertainty_footprint_inset_ > 0.0 &&
        failure_is_at_initial_pose && costmap_ros_ &&
        costmap_ros_->getLayeredCostmap())
      {
        observation_certificate = certify_observation_layer_sequence(
          *costmap_ros_->getLayeredCostmap(), localization_core_footprint_,
          trajectory.poses, maximum_swept_distance_,
          &observation_layer_certification_workspace_);
      }
      const bool localization_overlap_recovered =
        localization_uncertainty_footprint_inset_ > 0.0 &&
        failure_is_at_initial_pose &&
        observation_certificate.safe &&
        certify_initial_overlap_margin_sequence(
        *costmap_, inset_core_footprint_, localization_core_footprint_,
        trajectory.poses, maximum_swept_distance_, &overlap_fraction,
        &certification_workspace_, false, true, 1.0, 0.0, true);
      if (localization_overlap_recovered) {
        // The inward localization core stays hard against live observations;
        // the measured body may only escape a non-growing AMCL-error-sized
        // boundary overlap. A core collision and any later re-entry stay hard.
        return localization_uncertainty_recovery_penalty_ *
               (1.0 + overlap_fraction);
      }
      overlap_fraction = 0.0;
      const bool transient_margin_recovered =
        enable_transient_boundary_margin_recovery_ &&
        !failure_is_at_initial_pose &&
        certify_initial_overlap_margin_sequence(
          *costmap_, footprint_spec_, inset_core_footprint_, trajectory.poses,
          maximum_swept_distance_, &overlap_fraction,
          &certification_workspace_, true,
          transient_boundary_margin_require_clearance_,
          transient_boundary_margin_maximum_overlap_fraction_,
          transient_boundary_margin_minimum_clear_suffix_fraction_);
      if (transient_margin_recovered) {
        return transient_boundary_margin_recovery_penalty_ *
               (1.0 + overlap_fraction);
      }
      if (detailed_failure_diagnostics_ &&
        initial_overlap_certificate_attempted)
      {
        std::ostringstream detail;
        detail << exception.what();
        if (initial_overlap_physical_certificate.safe) {
          detail << ";initial_overlap_recovery=planning_margin_policy_failed";
        } else {
          if (initial_overlap_physical_certificate.failure !=
            CertificationFailure::kInvalidInput)
          {
            detail <<
              ";initial_overlap_recovery=physical_core_failed" <<
              ";physical_core_pose_index=" <<
              initial_overlap_physical_certificate.failure_source_pose_index <<
              ";physical_core_subdivision=" <<
              initial_overlap_physical_certificate.failure_interpolation_index;
            if (initial_overlap_physical_certificate.has_failure_pose) {
              detail << std::setprecision(17) <<
                ";physical_core_pose_x=" <<
                initial_overlap_physical_certificate.failure_pose.x <<
                ";physical_core_pose_y=" <<
                initial_overlap_physical_certificate.failure_pose.y <<
                ";physical_core_pose_yaw=" <<
                initial_overlap_physical_certificate.failure_pose.theta;
            }
            append_certification_diagnostics(
              detail, initial_overlap_physical_certificate, "physical_core_");
          } else {
            detail << ";initial_overlap_recovery=physical_core_not_checked";
          }
        }
        if (localization_uncertainty_footprint_inset_ > 0.0 &&
          failure_is_at_initial_pose)
        {
          detail <<
            ";localization_observation_layer_available=" <<
            (observation_certificate.layer_available ? "true" : "false") <<
            ";localization_observation_layers_current=" <<
            (observation_certificate.layers_current ? "true" : "false") <<
            ";localization_observation_layers_safe=" <<
            (observation_certificate.safe ? "true" : "false");
          if (!observation_certificate.failure_layer_name.empty()) {
            detail << ";localization_observation_failure_layer=" <<
              observation_certificate.failure_layer_name;
          }
          if (observation_certificate.failure.failure !=
            CertificationFailure::kInvalidInput)
          {
            detail <<
              ";localization_observation_failure_pose_index=" <<
              observation_certificate.failure.failure_source_pose_index <<
              ";localization_observation_failure_subdivision=" <<
              observation_certificate.failure.failure_interpolation_index;
            if (observation_certificate.failure.has_failure_pose) {
              detail << std::setprecision(17) <<
                ";localization_observation_failure_pose_x=" <<
                observation_certificate.failure.failure_pose.x <<
                ";localization_observation_failure_pose_y=" <<
                observation_certificate.failure.failure_pose.y <<
                ";localization_observation_failure_pose_yaw=" <<
                observation_certificate.failure.failure_pose.theta;
            }
            append_certification_diagnostics(
              detail, observation_certificate.failure,
              "localization_observation_");
          }
        }
        throw dwb_core::IllegalTrajectoryException(name_, detail.str());
      }
    }
    throw;
  }
  return 0.0;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::HorizonObstacleFootprintCritic,
  dwb_core::TrajectoryCritic)
