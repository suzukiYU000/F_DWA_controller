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

#ifndef F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_
#define F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace f_dwa_controller
{

enum class CertificationFailure
{
  kNone,
  kInvalidInput,
  kOffCostmap,
  kLethalObstacle,
  kUnknownSpace
};

struct CertificationResult
{
  bool safe{false};
  CertificationFailure failure{CertificationFailure::kInvalidInput};
  std::size_t checked_pose_count{0};
  bool has_failure_pose{false};
  std::size_t failure_source_pose_index{0};
  std::size_t failure_interpolation_index{0};
  geometry_msgs::msg::Pose2D failure_pose;
  bool has_failure_cell{false};
  unsigned int failure_cell_x{0};
  unsigned int failure_cell_y{0};
  unsigned char failure_cell_cost{0};
  double failure_cell_world_x{0.0};
  double failure_cell_world_y{0.0};
};

struct ObservationLayerCertificationResult
{
  // A localization-error recovery is fail-closed unless at least one enabled
  // ObstacleLayer or VoxelLayer supplied a current, collision-free view.
  bool layer_available{false};
  bool layers_current{false};
  bool safe{false};
  std::string failure_layer_name;
  CertificationResult failure;
};

constexpr std::size_t kMaximumCachedFootprintVertices = 16u;

struct CachedFootprintPoint
{
  double x{0.0};
  double y{0.0};
};

struct PreparedFootprintAxis
{
  double x{0.0};
  double y{0.0};
  double length{0.0};
  double projection_minimum{0.0};
  double projection_maximum{0.0};
};

struct PoseCheckCacheEntry
{
  std::array<
    CachedFootprintPoint,
    kMaximumCachedFootprintVertices> world_footprint{};
  std::size_t vertex_count{0u};
  bool allow_lethal{false};
  bool allow_unknown_space{false};
  bool lethal_overlap{false};
  CertificationResult result;
};

struct CertificationWorkspace
{
  std::vector<geometry_msgs::msg::Point> world_footprint;
  std::vector<PreparedFootprintAxis> footprint_axes;
  std::vector<nav2_costmap_2d::MapLocation> map_footprint;
  std::vector<nav2_costmap_2d::MapLocation> footprint_cells;
  std::vector<std::size_t> hazard_prefix_sum;
  // Lethal and unknown cells in row-major order. Row offsets let the exact
  // continuous-footprint test visit only hazards inside its map AABB instead
  // of rescanning every free cell for every rollout pose.
  std::vector<nav2_costmap_2d::MapLocation> hazard_cells;
  std::vector<std::size_t> hazard_row_offsets;
  unsigned int hazard_size_x{0};
  unsigned int hazard_size_y{0};
  double hazard_origin_x{0.0};
  double hazard_origin_y{0.0};
  double hazard_resolution{0.0};
  bool hazard_unknown_space_is_hazard{true};
  bool hazard_prefix_valid{false};
  // F-DWA's long filtered stop tails often revisit an exact continuous
  // footprint. Cache that result without merging different sub-cell poses,
  // because occupied cells are checked as complete squares.
  std::vector<PoseCheckCacheEntry> pose_check_cache;
  std::unordered_multimap<std::size_t, std::size_t> pose_check_cache_index;
};

struct ObservationLayerCertificationWorkspaceEntry
{
  nav2_costmap_2d::ObstacleLayer * layer{nullptr};
  std::string layer_name;
  bool broadphase_prepared{false};
  CertificationWorkspace certification;
};

struct ObservationLayerCertificationWorkspace
{
  // Valid only while the caller keeps the layered Costmap snapshot locked.
  // One entry is reused by every candidate in that control cycle.
  bool valid{false};
  std::vector<ObservationLayerCertificationWorkspaceEntry> layers;
};

// The prefix is valid only while the caller keeps the costmap snapshot
// unchanged. The planner prepares it under DWB's costmap mutex once per
// control cycle and invalidates it before the next snapshot.
bool prepare_certification_broadphase(
  const nav2_costmap_2d::Costmap2D & costmap,
  CertificationWorkspace & workspace,
  bool unknown_space_is_hazard = true);

void invalidate_certification_broadphase(
  CertificationWorkspace & workspace);

void invalidate_observation_layer_certification_workspace(
  ObservationLayerCertificationWorkspace & workspace);

// Return true only when the transformed footprint's complete axis-aligned
// bounds contain no lethal or unknown Costmap cell. False is inconclusive and
// requires the caller's exact footprint check.
bool certification_footprint_bounds_are_hazard_free(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::Pose2D & pose,
  const CertificationWorkspace & workspace);

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  CertificationWorkspace * workspace = nullptr,
  bool allow_unknown_space = false);

// Certify the caller's hard footprint against every enabled live sensor
// obstacle layer, independently of the fused master and StaticLayer. For a
// bounded localization-error recovery the caller supplies its inward core;
// the outer error band is then governed separately by non-growing overlap and
// clearance requirements on the fused map.
// NO_INFORMATION means the sensor layer has not marked that cell and is not a
// sensor-observed obstacle; stale, off-map, and lethal cells remain fail-closed.
ObservationLayerCertificationResult certify_observation_layer_sequence(
  nav2_costmap_2d::LayeredCostmap & layered_costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  ObservationLayerCertificationWorkspace * workspace = nullptr);

// A recovery sequence may start inside only the additional certificate
// reserve. The planning footprint must remain safe, the trajectory must reach
// a pose that clears the reserve, and hysteresis optionally requires the
// complete remaining suffix to stay clear. Recovery is evaluated over the
// complete swept footprint rather than by monotonic pose-to-goal distance so
// a hard-safe rectangular tail swing can be retained.
bool certify_reserve_recovery_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & certified_footprint,
  const std::vector<geometry_msgs::msg::Point> & planning_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  bool require_clear_suffix,
  CertificationWorkspace * workspace = nullptr);

// A bounded initial-overlap recovery may encounter a lethal cell in the
// physical-footprint boundary strip before first_required_clear_pose. The
// inset core must remain hard-safe for the complete sweep, and the physical
// footprint must be continuously clear from that pose onward. Unknown and
// off-costmap overlap is never recoverable.
bool certify_initial_overlap_recovery_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Point> & inset_core_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  std::size_t first_required_clear_pose,
  CertificationWorkspace * workspace = nullptr);

// A boundary-margin sequence is normally available only when the padded
// planning footprint is on a lethal cell at the current pose or enters it during the
// first response segment. A caller may explicitly permit one later transient
// entry. Lethal cells may remain only in the added planning strip; the inset
// physical footprint must remain safe over the complete swept sequence. Re-entry
// after clearing is rejected by default; an explicitly bounded localization-error
// prefix may permit it while the inward physical core remains continuously safe.
// Unknown space and off-costmap overlap are never accepted.
// Optional overlap and clear-suffix bounds let nominal rollout scoring retain
// only a short, self-clearing quantization/noise overlap. A localization-error
// recovery can additionally require that lethal-cell penetration never grows
// beyond its value at first contact. The less restrictive defaults preserve
// the committed-prefix and initial-overlap semantics.
bool certify_initial_overlap_margin_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & planning_footprint,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  double * overlap_fraction = nullptr,
  CertificationWorkspace * workspace = nullptr,
  bool allow_later_entry = false,
  bool require_planning_clearance = false,
  double maximum_overlap_fraction = 1.0,
  double minimum_clear_suffix_fraction = 0.0,
  bool require_nonincreasing_overlap_depth = false,
  CertificationResult * physical_certificate = nullptr,
  bool allow_reentry_after_clearance = false);

const char * certification_failure_name(CertificationFailure failure);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_
