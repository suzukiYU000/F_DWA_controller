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

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

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

struct CertificationWorkspace
{
  std::vector<nav2_costmap_2d::MapLocation> map_footprint;
  std::vector<nav2_costmap_2d::MapLocation> footprint_cells;
  std::vector<std::size_t> hazard_prefix_sum;
  unsigned int hazard_size_x{0};
  unsigned int hazard_size_y{0};
  double hazard_origin_x{0.0};
  double hazard_origin_y{0.0};
  double hazard_resolution{0.0};
  bool hazard_prefix_valid{false};
};

// The prefix is valid only while the caller keeps the costmap snapshot
// unchanged. The planner prepares it under DWB's costmap mutex once per
// control cycle and invalidates it before the next snapshot.
bool prepare_certification_broadphase(
  const nav2_costmap_2d::Costmap2D & costmap,
  CertificationWorkspace & workspace);

void invalidate_certification_broadphase(
  CertificationWorkspace & workspace);

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  CertificationWorkspace * workspace = nullptr);

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

// A boundary-margin sequence is available only when the physical footprint is
// on a lethal cell at the current pose or enters it during the first response
// segment. A caller checking an already-issued, no-longer-changeable command
// prefix may opt into later entry within that prefix. Lethal cells may remain
// in the physical footprint's outer strip,
// but the inset core must remain safe over the complete swept sequence. Later
// entry, re-entry after clearing, unknown space, and off-costmap overlap are
// never accepted. overlap_fraction reports
// how much of the swept sequence used the margin so callers can prefer prompt
// clearance without imposing a deadline that can deadlock beside a wall.
bool certify_initial_overlap_margin_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Point> & inset_core_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance,
  double * overlap_fraction = nullptr,
  CertificationWorkspace * workspace = nullptr,
  bool allow_committed_prefix_entry = false);

const char * certification_failure_name(CertificationFailure failure);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_
