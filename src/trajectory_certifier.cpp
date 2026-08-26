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

#include "f_dwa_controller/trajectory_certifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace f_dwa_controller
{

namespace
{

bool query_hazard_count(
  const nav2_costmap_2d::Costmap2D & costmap,
  const CertificationWorkspace & workspace,
  const unsigned int minimum_x,
  const unsigned int minimum_y,
  const unsigned int maximum_x,
  const unsigned int maximum_y,
  std::size_t & hazard_count)
{
  const std::size_t size_x = costmap.getSizeInCellsX();
  const std::size_t size_y = costmap.getSizeInCellsY();
  if (size_x == 0u || size_y == 0u ||
    minimum_x > maximum_x || minimum_y > maximum_y ||
    maximum_x >= size_x || maximum_y >= size_y ||
    size_x == std::numeric_limits<std::size_t>::max())
  {
    return false;
  }
  const std::size_t stride = size_x + 1u;
  if (size_y + 1u > std::numeric_limits<std::size_t>::max() / stride) {
    return false;
  }
  const std::size_t expected_prefix_size = stride * (size_y + 1u);
  const bool matching_broadphase =
    workspace.hazard_prefix_valid &&
    workspace.hazard_size_x == costmap.getSizeInCellsX() &&
    workspace.hazard_size_y == costmap.getSizeInCellsY() &&
    workspace.hazard_origin_x == costmap.getOriginX() &&
    workspace.hazard_origin_y == costmap.getOriginY() &&
    workspace.hazard_resolution == costmap.getResolution() &&
    workspace.hazard_prefix_sum.size() == expected_prefix_size;
  if (!matching_broadphase) {
    return false;
  }

  const auto prefix_at =
    [&workspace, stride](const std::size_t x, const std::size_t y)
    {
      return workspace.hazard_prefix_sum[y * stride + x];
    };
  const std::size_t lower_x = minimum_x;
  const std::size_t lower_y = minimum_y;
  const std::size_t upper_x = static_cast<std::size_t>(maximum_x) + 1u;
  const std::size_t upper_y = static_cast<std::size_t>(maximum_y) + 1u;
  hazard_count =
    prefix_at(upper_x, upper_y) -
    prefix_at(lower_x, upper_y) -
    prefix_at(upper_x, lower_y) +
    prefix_at(lower_x, lower_y);
  return true;
}

CertificationResult check_pose(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::Pose2D & pose,
  std::vector<nav2_costmap_2d::MapLocation> & map_footprint,
  std::vector<nav2_costmap_2d::MapLocation> & footprint_cells,
  const CertificationWorkspace & workspace,
  const bool allow_lethal = false,
  bool * lethal_overlap = nullptr)
{
  CertificationResult result;
  if (lethal_overlap) {
    *lethal_overlap = false;
  }
  result.checked_pose_count = 1;
  map_footprint.clear();
  map_footprint.reserve(footprint.size());
  unsigned int minimum_x = std::numeric_limits<unsigned int>::max();
  unsigned int minimum_y = std::numeric_limits<unsigned int>::max();
  unsigned int maximum_x = 0;
  unsigned int maximum_y = 0;
  const double cosine = std::cos(pose.theta);
  const double sine = std::sin(pose.theta);
  for (const geometry_msgs::msg::Point & point : footprint) {
    const double world_x =
      pose.x + point.x * cosine - point.y * sine;
    const double world_y =
      pose.y + point.x * sine + point.y * cosine;
    nav2_costmap_2d::MapLocation map_point;
    if (!costmap.worldToMap(
        world_x, world_y, map_point.x, map_point.y))
    {
      result.failure = CertificationFailure::kOffCostmap;
      return result;
    }
    map_footprint.push_back(map_point);
    minimum_x = std::min(minimum_x, map_point.x);
    minimum_y = std::min(minimum_y, map_point.y);
    maximum_x = std::max(maximum_x, map_point.x);
    maximum_y = std::max(maximum_y, map_point.y);
  }

  std::size_t hazard_count = 0u;
  if (query_hazard_count(
      costmap, workspace, minimum_x, minimum_y, maximum_x, maximum_y,
      hazard_count) &&
    hazard_count == 0u)
  {
    result.safe = true;
    result.failure = CertificationFailure::kNone;
    return result;
  }

  footprint_cells.clear();
  costmap.convexFillCells(map_footprint, footprint_cells);
  for (const nav2_costmap_2d::MapLocation & cell : footprint_cells) {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      result.failure = CertificationFailure::kUnknownSpace;
      result.has_failure_cell = true;
      result.failure_cell_x = cell.x;
      result.failure_cell_y = cell.y;
      result.failure_cell_cost = cost;
      costmap.mapToWorld(
        cell.x, cell.y, result.failure_cell_world_x,
        result.failure_cell_world_y);
      return result;
    }
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      if (lethal_overlap) {
        *lethal_overlap = true;
      }
      if (allow_lethal) {
        continue;
      }
      result.failure = CertificationFailure::kLethalObstacle;
      result.has_failure_cell = true;
      result.failure_cell_x = cell.x;
      result.failure_cell_y = cell.y;
      result.failure_cell_cost = cost;
      costmap.mapToWorld(
        cell.x, cell.y, result.failure_cell_world_x,
        result.failure_cell_world_y);
      return result;
    }
  }
  result.safe = true;
  result.failure = CertificationFailure::kNone;
  return result;
}

double normalized_angle_difference(
  const double from,
  const double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

bool certify_hazard_free_sequence_bounds(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_footprint_radius,
  const double maximum_swept_distance,
  const CertificationWorkspace & workspace,
  std::size_t & checked_pose_count)
{
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const geometry_msgs::msg::Pose2D & pose : poses) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta))
    {
      return false;
    }
    minimum_x = std::min(minimum_x, pose.x);
    minimum_y = std::min(minimum_y, pose.y);
    maximum_x = std::max(maximum_x, pose.x);
    maximum_y = std::max(maximum_y, pose.y);
  }

  // Every translated or rotated footprint point lies inside this circle-based
  // world AABB, including all interpolated poses between trajectory samples.
  // An empty hazard query therefore certifies the complete swept trajectory.
  unsigned int minimum_map_x = 0u;
  unsigned int minimum_map_y = 0u;
  unsigned int maximum_map_x = 0u;
  unsigned int maximum_map_y = 0u;
  const double coordinate_scale =
    std::max(
    {1.0, std::abs(minimum_x), std::abs(minimum_y),
      std::abs(maximum_x), std::abs(maximum_y),
      maximum_footprint_radius});
  const double outward_rounding_margin = 1.0e-12 * coordinate_scale;
  if (!costmap.worldToMap(
      minimum_x - maximum_footprint_radius - outward_rounding_margin,
      minimum_y - maximum_footprint_radius - outward_rounding_margin,
      minimum_map_x, minimum_map_y) ||
    !costmap.worldToMap(
      maximum_x + maximum_footprint_radius + outward_rounding_margin,
      maximum_y + maximum_footprint_radius + outward_rounding_margin,
      maximum_map_x, maximum_map_y))
  {
    return false;
  }

  std::size_t hazard_count = 0u;
  if (!query_hazard_count(
      costmap, workspace, minimum_map_x, minimum_map_y,
      maximum_map_x, maximum_map_y, hazard_count) ||
    hazard_count != 0u)
  {
    return false;
  }

  checked_pose_count = 1u;
  for (std::size_t pose_index = 1u;
    pose_index < poses.size(); ++pose_index)
  {
    const geometry_msgs::msg::Pose2D & previous = poses[pose_index - 1u];
    const geometry_msgs::msg::Pose2D & next = poses[pose_index];
    const double angle_difference =
      normalized_angle_difference(previous.theta, next.theta);
    const double swept_distance =
      std::hypot(next.x - previous.x, next.y - previous.y) +
      maximum_footprint_radius * std::abs(angle_difference);
    checked_pose_count += static_cast<std::size_t>(
      std::max(
        1, static_cast<int>(
          std::ceil(swept_distance / maximum_swept_distance))));
  }
  return true;
}

}  // namespace

bool prepare_certification_broadphase(
  const nav2_costmap_2d::Costmap2D & costmap,
  CertificationWorkspace & workspace)
{
  invalidate_certification_broadphase(workspace);
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  if (size_x == 0u || size_y == 0u) {
    return false;
  }
  const std::size_t stride = static_cast<std::size_t>(size_x) + 1u;
  if (static_cast<std::size_t>(size_y) + 1u >
    std::numeric_limits<std::size_t>::max() / stride)
  {
    return false;
  }

  const std::size_t prefix_size =
    stride * (static_cast<std::size_t>(size_y) + 1u);
  if (workspace.hazard_prefix_sum.size() != prefix_size) {
    workspace.hazard_prefix_sum.assign(prefix_size, 0u);
  } else {
    // Every interior entry is overwritten below. Only the integral-image
    // border must be cleared when reusing the allocation.
    std::fill(
      workspace.hazard_prefix_sum.begin(),
      workspace.hazard_prefix_sum.begin() +
      static_cast<std::ptrdiff_t>(stride), 0u);
    for (std::size_t y = 1u;
      y <= static_cast<std::size_t>(size_y); ++y)
    {
      workspace.hazard_prefix_sum[y * stride] = 0u;
    }
  }
  for (unsigned int y = 0; y < size_y; ++y) {
    std::size_t row_hazard_count = 0u;
    for (unsigned int x = 0; x < size_x; ++x) {
      const unsigned char cost = costmap.getCost(x, y);
      if (cost == nav2_costmap_2d::NO_INFORMATION ||
        cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        ++row_hazard_count;
      }
      const std::size_t index =
        (static_cast<std::size_t>(y) + 1u) * stride +
        (static_cast<std::size_t>(x) + 1u);
      workspace.hazard_prefix_sum[index] =
        workspace.hazard_prefix_sum[index - stride] +
        row_hazard_count;
    }
  }
  workspace.hazard_size_x = size_x;
  workspace.hazard_size_y = size_y;
  workspace.hazard_origin_x = costmap.getOriginX();
  workspace.hazard_origin_y = costmap.getOriginY();
  workspace.hazard_resolution = costmap.getResolution();
  workspace.hazard_prefix_valid = true;
  return true;
}

void invalidate_certification_broadphase(
  CertificationWorkspace & workspace)
{
  workspace.hazard_size_x = 0u;
  workspace.hazard_size_y = 0u;
  workspace.hazard_origin_x = 0.0;
  workspace.hazard_origin_y = 0.0;
  workspace.hazard_resolution = 0.0;
  workspace.hazard_prefix_valid = false;
}

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  CertificationWorkspace * workspace)
{
  CertificationResult result;
  if (footprint.size() < 3u || poses.empty() ||
    !std::isfinite(maximum_swept_distance) ||
    maximum_swept_distance <= 0.0)
  {
    return result;
  }

  double maximum_footprint_radius = 0.0;
  for (const geometry_msgs::msg::Point & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return result;
    }
    maximum_footprint_radius =
      std::max(maximum_footprint_radius, std::hypot(point.x, point.y));
  }

  CertificationWorkspace local_workspace;
  CertificationWorkspace & active_workspace =
    workspace ? *workspace : local_workspace;
  if (certify_hazard_free_sequence_bounds(
      costmap, poses, maximum_footprint_radius,
      maximum_swept_distance, active_workspace,
      result.checked_pose_count))
  {
    result.safe = true;
    result.failure = CertificationFailure::kNone;
    return result;
  }
  CertificationResult pose_result =
    check_pose(
    costmap, footprint, poses.front(),
    active_workspace.map_footprint,
    active_workspace.footprint_cells, active_workspace);
  result.checked_pose_count += pose_result.checked_pose_count;
  if (!pose_result.safe) {
    result.failure = pose_result.failure;
    result.has_failure_pose = true;
    result.failure_pose = poses.front();
    result.has_failure_cell = pose_result.has_failure_cell;
    result.failure_cell_x = pose_result.failure_cell_x;
    result.failure_cell_y = pose_result.failure_cell_y;
    result.failure_cell_cost = pose_result.failure_cell_cost;
    result.failure_cell_world_x = pose_result.failure_cell_world_x;
    result.failure_cell_world_y = pose_result.failure_cell_world_y;
    return result;
  }

  for (std::size_t pose_index = 1u;
    pose_index < poses.size(); ++pose_index)
  {
    const geometry_msgs::msg::Pose2D & previous = poses[pose_index - 1u];
    const geometry_msgs::msg::Pose2D & next = poses[pose_index];
    if (!std::isfinite(previous.x) || !std::isfinite(previous.y) ||
      !std::isfinite(previous.theta) || !std::isfinite(next.x) ||
      !std::isfinite(next.y) || !std::isfinite(next.theta))
    {
      result.failure = CertificationFailure::kInvalidInput;
      return result;
    }
    const double angle_difference =
      normalized_angle_difference(previous.theta, next.theta);
    const double swept_distance =
      std::hypot(next.x - previous.x, next.y - previous.y) +
      maximum_footprint_radius * std::abs(angle_difference);
    const int interpolation_count =
      std::max(
      1, static_cast<int>(
        std::ceil(swept_distance / maximum_swept_distance)));
    for (int interpolation_index = 1;
      interpolation_index <= interpolation_count; ++interpolation_index)
    {
      const double ratio =
        static_cast<double>(interpolation_index) /
        static_cast<double>(interpolation_count);
      geometry_msgs::msg::Pose2D interpolated;
      interpolated.x = previous.x + ratio * (next.x - previous.x);
      interpolated.y = previous.y + ratio * (next.y - previous.y);
      interpolated.theta = previous.theta + ratio * angle_difference;
      pose_result = check_pose(
        costmap, footprint, interpolated,
        active_workspace.map_footprint,
        active_workspace.footprint_cells, active_workspace);
      result.checked_pose_count += pose_result.checked_pose_count;
      if (!pose_result.safe) {
        result.failure = pose_result.failure;
        result.has_failure_pose = true;
        result.failure_source_pose_index = pose_index;
        result.failure_interpolation_index =
          static_cast<std::size_t>(interpolation_index);
        result.failure_pose = interpolated;
        result.has_failure_cell = pose_result.has_failure_cell;
        result.failure_cell_x = pose_result.failure_cell_x;
        result.failure_cell_y = pose_result.failure_cell_y;
        result.failure_cell_cost = pose_result.failure_cell_cost;
        result.failure_cell_world_x = pose_result.failure_cell_world_x;
        result.failure_cell_world_y = pose_result.failure_cell_world_y;
        return result;
      }
    }
  }

  result.safe = true;
  result.failure = CertificationFailure::kNone;
  return result;
}

bool certify_reserve_recovery_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & certified_footprint,
  const std::vector<geometry_msgs::msg::Point> & planning_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  const bool require_clear_suffix,
  CertificationWorkspace * workspace)
{
  if (poses.size() < 2u) {
    return false;
  }
  const CertificationResult planning_result =
    certify_pose_sequence(
    costmap, planning_footprint, poses, maximum_swept_distance,
    workspace);
  if (!planning_result.safe) {
    return false;
  }

  const std::vector<geometry_msgs::msg::Pose2D> initial_pose{
    poses.front()};
  const CertificationResult initial_result =
    certify_pose_sequence(
    costmap, certified_footprint, initial_pose,
    maximum_swept_distance, workspace);
  if (initial_result.safe ||
    initial_result.failure != CertificationFailure::kLethalObstacle)
  {
    return false;
  }

  std::size_t clearance_index = poses.size();
  for (std::size_t reverse_index = poses.size();
    reverse_index > 1u; --reverse_index)
  {
    const std::size_t pose_index = reverse_index - 1u;
    const std::vector<geometry_msgs::msg::Pose2D> single_pose{
      poses[pose_index]};
    const CertificationResult pose_result =
      certify_pose_sequence(
      costmap, certified_footprint, single_pose,
      maximum_swept_distance, workspace);
    if (!pose_result.safe) {
      break;
    }
    clearance_index = pose_index;
    if (!require_clear_suffix) {
      break;
    }
  }
  if (clearance_index >= poses.size()) {
    return false;
  }

  if (require_clear_suffix) {
    const std::vector<geometry_msgs::msg::Pose2D> clear_suffix(
      poses.begin() + static_cast<std::ptrdiff_t>(clearance_index),
      poses.end());
    if (!certify_pose_sequence(
        costmap, certified_footprint, clear_suffix,
        maximum_swept_distance, workspace).safe)
    {
      return false;
    }
  }

  return true;
}

bool certify_initial_overlap_recovery_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Point> & inset_core_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  const std::size_t first_required_clear_pose,
  CertificationWorkspace * workspace)
{
  if (poses.size() < 2u || first_required_clear_pose == 0u ||
    first_required_clear_pose >= poses.size())
  {
    return false;
  }

  const std::vector<geometry_msgs::msg::Pose2D> overlap_prefix(
    poses.begin(),
    poses.begin() + static_cast<std::ptrdiff_t>(first_required_clear_pose));
  const CertificationResult overlap_result =
    certify_pose_sequence(
    costmap, physical_footprint, overlap_prefix,
    maximum_swept_distance, workspace);
  if (overlap_result.safe ||
    overlap_result.failure != CertificationFailure::kLethalObstacle)
  {
    return false;
  }

  if (!certify_pose_sequence(
      costmap, inset_core_footprint, poses,
      maximum_swept_distance, workspace).safe)
  {
    return false;
  }

  const std::vector<geometry_msgs::msg::Pose2D> clear_suffix(
    poses.begin() + static_cast<std::ptrdiff_t>(first_required_clear_pose),
    poses.end());
  return certify_pose_sequence(
    costmap, physical_footprint, clear_suffix,
    maximum_swept_distance, workspace).safe;
}

bool certify_initial_overlap_margin_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Point> & inset_core_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  double * overlap_fraction,
  CertificationWorkspace * workspace,
  const bool allow_committed_prefix_entry)
{
  if (overlap_fraction) {
    *overlap_fraction = 0.0;
  }
  if (physical_footprint.size() < 3u || poses.empty() ||
    !std::isfinite(maximum_swept_distance) ||
    maximum_swept_distance <= 0.0)
  {
    return false;
  }

  double maximum_footprint_radius = 0.0;
  for (const geometry_msgs::msg::Point & point : physical_footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    maximum_footprint_radius =
      std::max(maximum_footprint_radius, std::hypot(point.x, point.y));
  }

  CertificationWorkspace local_workspace;
  CertificationWorkspace & active_workspace =
    workspace ? *workspace : local_workspace;
  bool initial_lethal_overlap = false;
  const CertificationResult initial_result = check_pose(
    costmap, physical_footprint, poses.front(),
    active_workspace.map_footprint, active_workspace.footprint_cells,
    active_workspace, true, &initial_lethal_overlap);
  if (!initial_result.safe) {
    return false;
  }

  // This is the hard body certificate. Densification accounts for the rear
  // corner arc as well as translation, so turning away beside a wall cannot
  // sweep the tail through an obstacle.
  if (!certify_pose_sequence(
      costmap, inset_core_footprint, poses,
      maximum_swept_distance, &active_workspace).safe)
  {
    return false;
  }

  std::size_t checked_pose_count = 1u;
  std::size_t overlap_pose_count = initial_lethal_overlap ? 1u : 0u;
  bool margin_started = initial_lethal_overlap;
  bool physical_clear_observed = false;
  for (std::size_t pose_index = 1u; pose_index < poses.size(); ++pose_index) {
    const geometry_msgs::msg::Pose2D & previous = poses[pose_index - 1u];
    const geometry_msgs::msg::Pose2D & next = poses[pose_index];
    if (!std::isfinite(previous.x) || !std::isfinite(previous.y) ||
      !std::isfinite(previous.theta) || !std::isfinite(next.x) ||
      !std::isfinite(next.y) || !std::isfinite(next.theta))
    {
      return false;
    }
    const double angle_difference =
      normalized_angle_difference(previous.theta, next.theta);
    const double swept_distance =
      std::hypot(next.x - previous.x, next.y - previous.y) +
      maximum_footprint_radius * std::abs(angle_difference);
    const int interpolation_count = std::max(
      1, static_cast<int>(
        std::ceil(swept_distance / maximum_swept_distance)));
    for (int interpolation_index = 1;
      interpolation_index <= interpolation_count; ++interpolation_index)
    {
      const double ratio =
        static_cast<double>(interpolation_index) /
        static_cast<double>(interpolation_count);
      geometry_msgs::msg::Pose2D interpolated;
      interpolated.x = previous.x + ratio * (next.x - previous.x);
      interpolated.y = previous.y + ratio * (next.y - previous.y);
      interpolated.theta = previous.theta + ratio * angle_difference;
      bool lethal_overlap = false;
      const CertificationResult pose_result = check_pose(
        costmap, physical_footprint, interpolated,
        active_workspace.map_footprint, active_workspace.footprint_cells,
        active_workspace, true, &lethal_overlap);
      if (!pose_result.safe) {
        return false;
      }
      ++checked_pose_count;
      if (lethal_overlap) {
        if (!margin_started) {
          // A newly commanded candidate may use only its common first 50 ms
          // response. An already-issued delay prefix is no longer alterable,
          // so let its boundary strip reach the predicted activation pose and
          // require the candidate beginning there to clear it.
          if (pose_index != 1u && !allow_committed_prefix_entry) {
            return false;
          }
          margin_started = true;
        }
        if (physical_clear_observed) {
          return false;
        }
        ++overlap_pose_count;
      } else if (margin_started) {
        physical_clear_observed = true;
      }
    }
  }

  if (!margin_started) {
    return false;
  }
  if (overlap_fraction) {
    *overlap_fraction = static_cast<double>(overlap_pose_count) /
      static_cast<double>(checked_pose_count);
  }
  return true;
}

const char * certification_failure_name(
  const CertificationFailure failure)
{
  switch (failure) {
    case CertificationFailure::kNone:
      return "none";
    case CertificationFailure::kInvalidInput:
      return "invalid_input";
    case CertificationFailure::kOffCostmap:
      return "off_costmap";
    case CertificationFailure::kLethalObstacle:
      return "lethal_obstacle";
    case CertificationFailure::kUnknownSpace:
      return "unknown_space";
  }
  return "invalid_input";
}

}  // namespace f_dwa_controller
