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
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace f_dwa_controller
{

namespace
{

bool broadphase_matches_costmap(
  const nav2_costmap_2d::Costmap2D & costmap,
  const CertificationWorkspace & workspace)
{
  const std::size_t size_x = costmap.getSizeInCellsX();
  const std::size_t size_y = costmap.getSizeInCellsY();
  if (size_x == 0u || size_y == 0u ||
    size_x == std::numeric_limits<std::size_t>::max())
  {
    return false;
  }
  const std::size_t stride = size_x + 1u;
  if (size_y + 1u > std::numeric_limits<std::size_t>::max() / stride) {
    return false;
  }
  return workspace.hazard_prefix_valid &&
         workspace.hazard_size_x == costmap.getSizeInCellsX() &&
         workspace.hazard_size_y == costmap.getSizeInCellsY() &&
         workspace.hazard_origin_x == costmap.getOriginX() &&
         workspace.hazard_origin_y == costmap.getOriginY() &&
         workspace.hazard_resolution == costmap.getResolution() &&
         workspace.hazard_prefix_sum.size() == stride * (size_y + 1u) &&
         workspace.hazard_row_offsets.size() == size_y + 1u &&
         workspace.hazard_row_offsets.back() == workspace.hazard_cells.size();
}

bool broadphase_matches_policy(
  const nav2_costmap_2d::Costmap2D & costmap,
  const CertificationWorkspace & workspace,
  const bool allow_unknown_space)
{
  return broadphase_matches_costmap(costmap, workspace) &&
         workspace.hazard_unknown_space_is_hazard == !allow_unknown_space;
}

bool query_hazard_count(
  const nav2_costmap_2d::Costmap2D & costmap,
  const CertificationWorkspace & workspace,
  const unsigned int minimum_x,
  const unsigned int minimum_y,
  const unsigned int maximum_x,
  const unsigned int maximum_y,
  std::size_t & hazard_count,
  const bool allow_unknown_space = false)
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
    broadphase_matches_policy(
    costmap, workspace, allow_unknown_space) &&
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

std::size_t hash_world_footprint(
  const std::vector<geometry_msgs::msg::Point> & world_footprint,
  const bool allow_lethal,
  const bool allow_unknown_space)
{
  std::size_t hash = world_footprint.size();
  const auto combine = [&hash](const std::size_t value) {
      hash ^= value + static_cast<std::size_t>(0x9e3779b9u) +
        (hash << 6u) + (hash >> 2u);
    };
  combine(allow_lethal ? 1u : 0u);
  combine(allow_unknown_space ? 1u : 0u);
  const std::hash<double> hash_double;
  for (const auto & point : world_footprint) {
    combine(hash_double(point.x));
    combine(hash_double(point.y));
  }
  return hash;
}

bool cached_footprint_matches(
  const PoseCheckCacheEntry & entry,
  const std::vector<geometry_msgs::msg::Point> & world_footprint,
  const bool allow_lethal,
  const bool allow_unknown_space)
{
  if (entry.vertex_count != world_footprint.size() ||
    entry.allow_lethal != allow_lethal ||
    entry.allow_unknown_space != allow_unknown_space)
  {
    return false;
  }
  for (std::size_t index = 0u; index < world_footprint.size(); ++index) {
    if (entry.world_footprint[index].x != world_footprint[index].x ||
      entry.world_footprint[index].y != world_footprint[index].y)
    {
      return false;
    }
  }
  return true;
}

void expand_map_bounds_by_one_cell(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned int & minimum_x,
  unsigned int & minimum_y,
  unsigned int & maximum_x,
  unsigned int & maximum_y)
{
  minimum_x = minimum_x > 0u ? minimum_x - 1u : 0u;
  minimum_y = minimum_y > 0u ? minimum_y - 1u : 0u;
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  if (size_x > 0u && maximum_x < size_x - 1u) {
    ++maximum_x;
  }
  if (size_y > 0u && maximum_y < size_y - 1u) {
    ++maximum_y;
  }
}

bool convex_footprint_intersects_cell(
  const std::vector<PreparedFootprintAxis> & footprint_axes,
  const double footprint_minimum_x,
  const double footprint_minimum_y,
  const double footprint_maximum_x,
  const double footprint_maximum_y,
  const double cell_minimum_x,
  const double cell_minimum_y,
  const double cell_maximum_x,
  const double cell_maximum_y,
  double * overlap_depth)
{
  double minimum_overlap_depth = std::numeric_limits<double>::infinity();
  const auto intervals_overlap =
    [&minimum_overlap_depth](
    const double footprint_minimum,
    const double footprint_maximum,
    const double cell_minimum,
    const double cell_maximum,
    const double axis_length)
    {
      if (footprint_maximum < cell_minimum ||
        cell_maximum < footprint_minimum)
      {
        return false;
      }
      minimum_overlap_depth = std::min(
        minimum_overlap_depth,
        std::min(
          footprint_maximum - cell_minimum,
          cell_maximum - footprint_minimum) / axis_length);
      return true;
    };

  if (!intervals_overlap(
      footprint_minimum_x, footprint_maximum_x,
      cell_minimum_x, cell_maximum_x, 1.0) ||
    !intervals_overlap(
      footprint_minimum_y, footprint_maximum_y,
      cell_minimum_y, cell_maximum_y, 1.0))
  {
    return false;
  }

  const double cell_centre_x = 0.5 * (cell_minimum_x + cell_maximum_x);
  const double cell_centre_y = 0.5 * (cell_minimum_y + cell_maximum_y);
  const double cell_half_width = 0.5 * (cell_maximum_x - cell_minimum_x);
  const double cell_half_height = 0.5 * (cell_maximum_y - cell_minimum_y);
  for (const auto & axis : footprint_axes) {
    const double cell_projection_centre =
      axis.x * cell_centre_x + axis.y * cell_centre_y;
    const double cell_projection_radius =
      std::abs(axis.x) * cell_half_width +
      std::abs(axis.y) * cell_half_height;
    if (!intervals_overlap(
        axis.projection_minimum, axis.projection_maximum,
        cell_projection_centre - cell_projection_radius,
        cell_projection_centre + cell_projection_radius,
        axis.length))
    {
      return false;
    }
  }

  if (overlap_depth) {
    *overlap_depth = std::max(0.0, minimum_overlap_depth);
  }
  return true;
}

CertificationResult check_pose(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::Pose2D & pose,
  std::vector<nav2_costmap_2d::MapLocation> & map_footprint,
  std::vector<nav2_costmap_2d::MapLocation> & footprint_cells,
  CertificationWorkspace & workspace,
  const bool allow_lethal = false,
  bool * lethal_overlap = nullptr,
  double * lethal_overlap_depth = nullptr,
  const bool allow_unknown_space = false)
{
  CertificationResult result;
  if (lethal_overlap) {
    *lethal_overlap = false;
  }
  if (lethal_overlap_depth) {
    *lethal_overlap_depth = 0.0;
  }
  result.checked_pose_count = 1;
  std::vector<geometry_msgs::msg::Point> & world_footprint =
    workspace.world_footprint;
  world_footprint.clear();
  world_footprint.reserve(footprint.size());
  map_footprint.clear();
  map_footprint.reserve(footprint.size());
  unsigned int minimum_x = std::numeric_limits<unsigned int>::max();
  unsigned int minimum_y = std::numeric_limits<unsigned int>::max();
  unsigned int maximum_x = 0;
  unsigned int maximum_y = 0;
  double footprint_minimum_x = std::numeric_limits<double>::infinity();
  double footprint_minimum_y = std::numeric_limits<double>::infinity();
  double footprint_maximum_x = -std::numeric_limits<double>::infinity();
  double footprint_maximum_y = -std::numeric_limits<double>::infinity();
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
    geometry_msgs::msg::Point world_point;
    world_point.x = world_x;
    world_point.y = world_y;
    world_footprint.push_back(world_point);
    map_footprint.push_back(map_point);
    minimum_x = std::min(minimum_x, map_point.x);
    minimum_y = std::min(minimum_y, map_point.y);
    maximum_x = std::max(maximum_x, map_point.x);
    maximum_y = std::max(maximum_y, map_point.y);
    footprint_minimum_x = std::min(footprint_minimum_x, world_x);
    footprint_minimum_y = std::min(footprint_minimum_y, world_y);
    footprint_maximum_x = std::max(footprint_maximum_x, world_x);
    footprint_maximum_y = std::max(footprint_maximum_y, world_y);
  }

  expand_map_bounds_by_one_cell(
    costmap, minimum_x, minimum_y, maximum_x, maximum_y);

  std::size_t hazard_count = 0u;
  if (query_hazard_count(
      costmap, workspace, minimum_x, minimum_y, maximum_x, maximum_y,
      hazard_count, allow_unknown_space) &&
    hazard_count == 0u)
  {
    result.safe = true;
    result.failure = CertificationFailure::kNone;
    return result;
  }

  const bool cacheable =
    world_footprint.size() <= kMaximumCachedFootprintVertices &&
    workspace.hazard_prefix_valid && !lethal_overlap_depth;
  std::size_t cache_hash = 0u;
  if (cacheable) {
    cache_hash = hash_world_footprint(
      world_footprint, allow_lethal, allow_unknown_space);
    const auto cached_range =
      workspace.pose_check_cache_index.equal_range(cache_hash);
    for (auto cached = cached_range.first;
      cached != cached_range.second; ++cached)
    {
      if (cached->second >= workspace.pose_check_cache.size()) {
        continue;
      }
      const PoseCheckCacheEntry & entry =
        workspace.pose_check_cache[cached->second];
      if (!cached_footprint_matches(
          entry, world_footprint, allow_lethal, allow_unknown_space))
      {
        continue;
      }
      if (lethal_overlap) {
        *lethal_overlap = entry.lethal_overlap;
      }
      return entry.result;
    }
  }
  const auto cache_result =
    [&workspace, &world_footprint, cacheable, cache_hash, allow_lethal,
      allow_unknown_space](
    const CertificationResult & completed,
    const bool completed_lethal_overlap)
    {
      if (cacheable) {
        PoseCheckCacheEntry entry;
        entry.vertex_count = world_footprint.size();
        entry.allow_lethal = allow_lethal;
        entry.allow_unknown_space = allow_unknown_space;
        entry.lethal_overlap = completed_lethal_overlap;
        entry.result = completed;
        for (std::size_t index = 0u;
          index < world_footprint.size(); ++index)
        {
          entry.world_footprint[index].x = world_footprint[index].x;
          entry.world_footprint[index].y = world_footprint[index].y;
        }
        const std::size_t index = workspace.pose_check_cache.size();
        workspace.pose_check_cache.push_back(std::move(entry));
        workspace.pose_check_cache_index.emplace(cache_hash, index);
      }
      return completed;
    };

  std::vector<PreparedFootprintAxis> & footprint_axes =
    workspace.footprint_axes;
  footprint_axes.clear();
  footprint_axes.reserve(world_footprint.size());
  for (std::size_t index = 0u; index < world_footprint.size(); ++index) {
    const auto & first = world_footprint[index];
    const auto & second =
      world_footprint[(index + 1u) % world_footprint.size()];
    PreparedFootprintAxis axis;
    axis.x = first.y - second.y;
    axis.y = second.x - first.x;
    axis.length = std::hypot(axis.x, axis.y);
    if (axis.length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    axis.projection_minimum =
      axis.x * world_footprint.front().x +
      axis.y * world_footprint.front().y;
    axis.projection_maximum = axis.projection_minimum;
    for (std::size_t point_index = 1u;
      point_index < world_footprint.size(); ++point_index)
    {
      const double projection =
        axis.x * world_footprint[point_index].x +
        axis.y * world_footprint[point_index].y;
      axis.projection_minimum =
        std::min(axis.projection_minimum, projection);
      axis.projection_maximum =
        std::max(axis.projection_maximum, projection);
    }
    footprint_axes.push_back(axis);
  }

  footprint_cells.clear();
  bool observed_lethal_overlap = false;
  const double resolution = costmap.getResolution();
  const double origin_x = costmap.getOriginX();
  const double origin_y = costmap.getOriginY();
  const auto check_hazard_cell =
    [&](const unsigned int cell_x, const unsigned int cell_y)
    {
      const unsigned char cost = costmap.getCost(cell_x, cell_y);
      const double cell_minimum_x =
        origin_x + static_cast<double>(cell_x) * resolution;
      const double cell_minimum_y =
        origin_y + static_cast<double>(cell_y) * resolution;
      double cell_overlap_depth = 0.0;
      if (!convex_footprint_intersects_cell(
          footprint_axes,
          footprint_minimum_x, footprint_minimum_y,
          footprint_maximum_x, footprint_maximum_y,
          cell_minimum_x, cell_minimum_y,
          cell_minimum_x + resolution, cell_minimum_y + resolution,
          lethal_overlap_depth ? &cell_overlap_depth : nullptr))
      {
        return false;
      }
      nav2_costmap_2d::MapLocation cell;
      cell.x = cell_x;
      cell.y = cell_y;
      footprint_cells.push_back(cell);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        if (allow_unknown_space) {
          return false;
        }
        result.failure = CertificationFailure::kUnknownSpace;
        result.has_failure_cell = true;
        result.failure_cell_x = cell.x;
        result.failure_cell_y = cell.y;
        result.failure_cell_cost = cost;
        costmap.mapToWorld(
          cell.x, cell.y, result.failure_cell_world_x,
          result.failure_cell_world_y);
        return true;
      }
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        observed_lethal_overlap = true;
        if (lethal_overlap) {
          *lethal_overlap = true;
        }
        if (lethal_overlap_depth) {
          *lethal_overlap_depth = std::max(
            *lethal_overlap_depth, cell_overlap_depth);
        }
        if (allow_lethal) {
          return false;
        }
        result.failure = CertificationFailure::kLethalObstacle;
        result.has_failure_cell = true;
        result.failure_cell_x = cell.x;
        result.failure_cell_y = cell.y;
        result.failure_cell_cost = cost;
        costmap.mapToWorld(
          cell.x, cell.y, result.failure_cell_world_x,
          result.failure_cell_world_y);
        return true;
      }
      return false;
    };

  if (broadphase_matches_policy(
      costmap, workspace, allow_unknown_space))
  {
    for (unsigned int cell_y = minimum_y; cell_y <= maximum_y; ++cell_y) {
      const std::size_t row_begin = workspace.hazard_row_offsets[cell_y];
      const std::size_t row_end = workspace.hazard_row_offsets[cell_y + 1u];
      const auto begin = workspace.hazard_cells.begin() +
        static_cast<std::ptrdiff_t>(row_begin);
      const auto end = workspace.hazard_cells.begin() +
        static_cast<std::ptrdiff_t>(row_end);
      const auto first = std::lower_bound(
        begin, end, minimum_x,
        [](const nav2_costmap_2d::MapLocation & cell, const unsigned int x) {
          return cell.x < x;
        });
      for (auto cell = first; cell != end && cell->x <= maximum_x; ++cell) {
        if (check_hazard_cell(cell->x, cell->y)) {
          return cache_result(result, observed_lethal_overlap);
        }
      }
    }
  } else {
    for (unsigned int cell_y = minimum_y; cell_y <= maximum_y; ++cell_y) {
      for (unsigned int cell_x = minimum_x; cell_x <= maximum_x; ++cell_x) {
        const unsigned char cost = costmap.getCost(cell_x, cell_y);
        if ((cost == nav2_costmap_2d::NO_INFORMATION &&
          allow_unknown_space) ||
          (cost != nav2_costmap_2d::NO_INFORMATION &&
          cost < nav2_costmap_2d::LETHAL_OBSTACLE))
        {
          continue;
        }
        if (check_hazard_cell(cell_x, cell_y)) {
          return cache_result(result, observed_lethal_overlap);
        }
      }
    }
  }
  result.safe = true;
  result.failure = CertificationFailure::kNone;
  return cache_result(result, observed_lethal_overlap);
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
  std::size_t & checked_pose_count,
  const bool allow_unknown_space)
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
  expand_map_bounds_by_one_cell(
    costmap, minimum_map_x, minimum_map_y,
    maximum_map_x, maximum_map_y);

  std::size_t hazard_count = 0u;
  if (!query_hazard_count(
      costmap, workspace, minimum_map_x, minimum_map_y,
      maximum_map_x, maximum_map_y, hazard_count,
      allow_unknown_space) ||
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

bool certify_hazard_free_segment_bounds(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::Pose2D & first,
  const geometry_msgs::msg::Pose2D & second,
  const double maximum_footprint_radius,
  const CertificationWorkspace & workspace,
  const bool allow_unknown_space)
{
  if (!std::isfinite(first.x) || !std::isfinite(first.y) ||
    !std::isfinite(first.theta) || !std::isfinite(second.x) ||
    !std::isfinite(second.y) || !std::isfinite(second.theta) ||
    !std::isfinite(maximum_footprint_radius) ||
    maximum_footprint_radius < 0.0)
  {
    return false;
  }

  // During the linear pose interpolation below, every footprint point stays
  // inside a circle of maximum_footprint_radius around the interpolated robot
  // centre.  The centre remains on the segment joining first and second, so
  // this AABB contains the complete continuously swept footprint regardless
  // of its intermediate orientation.  An empty hazard-prefix query therefore
  // proves this segment without changing the exact fallback for nearby cells.
  const double minimum_x = std::min(first.x, second.x);
  const double minimum_y = std::min(first.y, second.y);
  const double maximum_x = std::max(first.x, second.x);
  const double maximum_y = std::max(first.y, second.y);
  const double coordinate_scale = std::max(
    {1.0, std::abs(minimum_x), std::abs(minimum_y),
      std::abs(maximum_x), std::abs(maximum_y),
      maximum_footprint_radius});
  const double outward_rounding_margin = 1.0e-12 * coordinate_scale;
  unsigned int minimum_map_x = 0u;
  unsigned int minimum_map_y = 0u;
  unsigned int maximum_map_x = 0u;
  unsigned int maximum_map_y = 0u;
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
  expand_map_bounds_by_one_cell(
    costmap, minimum_map_x, minimum_map_y,
    maximum_map_x, maximum_map_y);

  std::size_t hazard_count = 0u;
  return query_hazard_count(
    costmap, workspace, minimum_map_x, minimum_map_y,
    maximum_map_x, maximum_map_y, hazard_count,
    allow_unknown_space) && hazard_count == 0u;
}

}  // namespace

bool prepare_certification_broadphase(
  const nav2_costmap_2d::Costmap2D & costmap,
  CertificationWorkspace & workspace,
  const bool unknown_space_is_hazard)
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
  workspace.hazard_cells.clear();
  workspace.hazard_row_offsets.resize(
    static_cast<std::size_t>(size_y) + 1u);
  for (unsigned int y = 0; y < size_y; ++y) {
    workspace.hazard_row_offsets[y] = workspace.hazard_cells.size();
    std::size_t row_hazard_count = 0u;
    for (unsigned int x = 0; x < size_x; ++x) {
      const unsigned char cost = costmap.getCost(x, y);
      const bool cell_is_hazard =
        cost == nav2_costmap_2d::NO_INFORMATION ?
        unknown_space_is_hazard :
        cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
      if (cell_is_hazard) {
        ++row_hazard_count;
        nav2_costmap_2d::MapLocation cell;
        cell.x = x;
        cell.y = y;
        workspace.hazard_cells.push_back(cell);
      }
      const std::size_t index =
        (static_cast<std::size_t>(y) + 1u) * stride +
        (static_cast<std::size_t>(x) + 1u);
      workspace.hazard_prefix_sum[index] =
        workspace.hazard_prefix_sum[index - stride] +
        row_hazard_count;
    }
  }
  workspace.hazard_row_offsets[size_y] = workspace.hazard_cells.size();
  workspace.hazard_size_x = size_x;
  workspace.hazard_size_y = size_y;
  workspace.hazard_origin_x = costmap.getOriginX();
  workspace.hazard_origin_y = costmap.getOriginY();
  workspace.hazard_resolution = costmap.getResolution();
  workspace.hazard_unknown_space_is_hazard = unknown_space_is_hazard;
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
  workspace.hazard_unknown_space_is_hazard = true;
  workspace.hazard_prefix_valid = false;
  workspace.pose_check_cache.clear();
  workspace.pose_check_cache_index.clear();
}

void invalidate_observation_layer_certification_workspace(
  ObservationLayerCertificationWorkspace & workspace)
{
  workspace.valid = false;
}

bool certification_footprint_bounds_are_hazard_free(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::Pose2D & pose,
  const CertificationWorkspace & workspace)
{
  if (footprint.size() < 3u || !std::isfinite(pose.x) ||
    !std::isfinite(pose.y) || !std::isfinite(pose.theta))
  {
    return false;
  }
  unsigned int minimum_x = std::numeric_limits<unsigned int>::max();
  unsigned int minimum_y = std::numeric_limits<unsigned int>::max();
  unsigned int maximum_x = 0u;
  unsigned int maximum_y = 0u;
  const double cosine = std::cos(pose.theta);
  const double sine = std::sin(pose.theta);
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    const double world_x = pose.x + point.x * cosine - point.y * sine;
    const double world_y = pose.y + point.x * sine + point.y * cosine;
    unsigned int map_x = 0u;
    unsigned int map_y = 0u;
    if (!costmap.worldToMap(world_x, world_y, map_x, map_y)) {
      return false;
    }
    minimum_x = std::min(minimum_x, map_x);
    minimum_y = std::min(minimum_y, map_y);
    maximum_x = std::max(maximum_x, map_x);
    maximum_y = std::max(maximum_y, map_y);
  }
  expand_map_bounds_by_one_cell(
    costmap, minimum_x, minimum_y, maximum_x, maximum_y);
  std::size_t hazard_count = 0u;
  return query_hazard_count(
    costmap, workspace, minimum_x, minimum_y, maximum_x, maximum_y,
    hazard_count) && hazard_count == 0u;
}

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  CertificationWorkspace * workspace,
  const bool allow_unknown_space)
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
      result.checked_pose_count, allow_unknown_space))
  {
    result.safe = true;
    result.failure = CertificationFailure::kNone;
    return result;
  }
  CertificationResult pose_result =
    check_pose(
    costmap, footprint, poses.front(),
    active_workspace.map_footprint,
    active_workspace.footprint_cells, active_workspace,
    false, nullptr, nullptr, allow_unknown_space);
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
    if (certify_hazard_free_segment_bounds(
        costmap, previous, next, maximum_footprint_radius,
        active_workspace, allow_unknown_space))
    {
      result.checked_pose_count +=
        static_cast<std::size_t>(interpolation_count);
      continue;
    }
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
        active_workspace.footprint_cells, active_workspace,
        false, nullptr, nullptr, allow_unknown_space);
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

ObservationLayerCertificationResult certify_observation_layer_sequence(
  nav2_costmap_2d::LayeredCostmap & layered_costmap,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  ObservationLayerCertificationWorkspace * workspace)
{
  ObservationLayerCertificationResult result;
  const auto plugins = layered_costmap.getPlugins();
  if (!plugins) {
    return result;
  }

  ObservationLayerCertificationWorkspace local_workspace;
  ObservationLayerCertificationWorkspace & active_workspace =
    workspace ? *workspace : local_workspace;
  if (!active_workspace.valid) {
    std::size_t observation_index = 0u;
    for (const auto & plugin : *plugins) {
      const auto observation_layer =
        std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
      if (!observation_layer || !plugin->isEnabled()) {
        continue;
      }
      if (active_workspace.layers.size() <= observation_index) {
        active_workspace.layers.emplace_back();
      }
      auto & entry = active_workspace.layers[observation_index];
      if (entry.layer != observation_layer.get()) {
        entry = ObservationLayerCertificationWorkspaceEntry{};
      }
      entry.layer = observation_layer.get();
      entry.layer_name = plugin->getName();
      entry.broadphase_prepared = plugin->isCurrent() &&
        prepare_certification_broadphase(
        *observation_layer, entry.certification, false);
      ++observation_index;
    }
    active_workspace.layers.resize(observation_index);
    active_workspace.valid = true;
  }

  result.layers_current = true;
  for (auto & entry : active_workspace.layers) {
    result.layer_available = true;
    if (!entry.layer || !entry.layer->isCurrent()) {
      result.layers_current = false;
      result.failure_layer_name = entry.layer_name;
      return result;
    }

    if (!entry.broadphase_prepared) {
      result.failure_layer_name = entry.layer_name;
      result.failure.failure = CertificationFailure::kInvalidInput;
      return result;
    }
    const CertificationResult layer_result = certify_pose_sequence(
      *entry.layer, physical_footprint, poses,
      maximum_swept_distance, &entry.certification, true);
    if (!layer_result.safe) {
      result.failure_layer_name = entry.layer_name;
      result.failure = layer_result;
      return result;
    }
  }

  result.safe = result.layer_available && result.layers_current;
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
  const std::vector<geometry_msgs::msg::Point> & planning_footprint,
  const std::vector<geometry_msgs::msg::Point> & physical_footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance,
  double * overlap_fraction,
  CertificationWorkspace * workspace,
  const bool allow_later_entry,
  const bool require_planning_clearance,
  const double maximum_overlap_fraction,
  const double minimum_clear_suffix_fraction,
  const bool require_nonincreasing_overlap_depth,
  CertificationResult * const physical_certificate,
  const bool allow_reentry_after_clearance)
{
  if (overlap_fraction) {
    *overlap_fraction = 0.0;
  }
  if (physical_certificate) {
    const CertificationResult empty_certificate;
    *physical_certificate = empty_certificate;
  }
  if (planning_footprint.size() < 3u || physical_footprint.size() < 3u ||
    poses.empty() ||
    !std::isfinite(maximum_swept_distance) ||
    maximum_swept_distance <= 0.0 ||
    !std::isfinite(maximum_overlap_fraction) ||
    maximum_overlap_fraction < 0.0 || maximum_overlap_fraction > 1.0 ||
    !std::isfinite(minimum_clear_suffix_fraction) ||
    minimum_clear_suffix_fraction < 0.0 ||
    minimum_clear_suffix_fraction > 1.0)
  {
    return false;
  }

  double maximum_footprint_radius = 0.0;
  for (const geometry_msgs::msg::Point & point : planning_footprint) {
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
  double initial_overlap_depth = 0.0;
  const CertificationResult initial_result = check_pose(
    costmap, planning_footprint, poses.front(),
    active_workspace.map_footprint, active_workspace.footprint_cells,
    active_workspace, true, &initial_lethal_overlap,
    require_nonincreasing_overlap_depth ? &initial_overlap_depth : nullptr);
  if (!initial_result.safe) {
    return false;
  }

  // This is the hard body certificate. Densification accounts for the rear
  // corner arc as well as translation, so turning away beside a wall cannot
  // sweep the tail through an obstacle.
  const CertificationResult physical_result = certify_pose_sequence(
    costmap, physical_footprint, poses,
    maximum_swept_distance, &active_workspace);
  if (physical_certificate) {
    *physical_certificate = physical_result;
  }
  if (!physical_result.safe) {
    return false;
  }

  std::size_t checked_pose_count = 1u;
  std::size_t overlap_pose_count = initial_lethal_overlap ? 1u : 0u;
  bool margin_started = initial_lethal_overlap;
  bool planning_clear_observed = false;
  std::size_t clear_suffix_pose_count = 0u;
  double maximum_permitted_overlap_depth = initial_overlap_depth;
  const double overlap_depth_tolerance = std::max(
    1.0e-6, 0.05 * costmap.getResolution());
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
      double lethal_overlap_depth = 0.0;
      const CertificationResult pose_result = check_pose(
        costmap, planning_footprint, interpolated,
        active_workspace.map_footprint, active_workspace.footprint_cells,
        active_workspace, true, &lethal_overlap,
        require_nonincreasing_overlap_depth ?
        &lethal_overlap_depth : nullptr);
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
          if (pose_index != 1u && !allow_later_entry) {
            return false;
          }
          margin_started = true;
          maximum_permitted_overlap_depth = lethal_overlap_depth;
        }
        if (planning_clear_observed && !allow_reentry_after_clearance) {
          return false;
        }
        if (planning_clear_observed) {
          // Only bounded receding-horizon localization recovery enables this.
          // Its inward body core and every current observation layer are
          // certified independently by the caller for the complete prefix.
          planning_clear_observed = false;
          clear_suffix_pose_count = 0u;
        }
        if (require_nonincreasing_overlap_depth &&
          lethal_overlap_depth >
          maximum_permitted_overlap_depth + overlap_depth_tolerance)
        {
          return false;
        }
        ++overlap_pose_count;
      } else if (margin_started) {
        planning_clear_observed = true;
        ++clear_suffix_pose_count;
      }
    }
  }

  if (!margin_started) {
    return false;
  }
  if (require_planning_clearance && !planning_clear_observed) {
    return false;
  }
  const double measured_overlap_fraction =
    static_cast<double>(overlap_pose_count) /
    static_cast<double>(checked_pose_count);
  const double measured_clear_suffix_fraction =
    static_cast<double>(clear_suffix_pose_count) /
    static_cast<double>(checked_pose_count);
  if (measured_overlap_fraction > maximum_overlap_fraction ||
    measured_clear_suffix_fraction < minimum_clear_suffix_fraction)
  {
    return false;
  }
  if (overlap_fraction) {
    *overlap_fraction = measured_overlap_fraction;
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
