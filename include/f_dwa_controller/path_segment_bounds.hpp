// Copyright 2026 YT Lab
// SPDX-License-Identifier: BSD-3-Clause
#ifndef F_DWA_CONTROLLER__PATH_SEGMENT_BOUNDS_HPP_
#define F_DWA_CONTROLLER__PATH_SEGMENT_BOUNDS_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace f_dwa_controller
{
constexpr std::size_t kPathSegmentBlockSize = 16u;

struct PathSegmentBounds
{
  std::size_t end;
  double minimum_x;
  double maximum_x;
  double minimum_y;
  double maximum_y;

  double squaredDistance(double x, double y) const
  {
    const double dx = std::max({minimum_x - x, 0.0, x - maximum_x});
    const double dy = std::max({minimum_y - y, 0.0, y - maximum_y});
    return dx * dx + dy * dy;
  }
};

// A block contains the exact boxes already used for individual segments.
// Skipping a block cannot skip a segment that the original scan would retain.
template<class Segment>
void prepare_path_segment_bounds(
  const std::vector<Segment> & segments, std::vector<PathSegmentBounds> & bounds)
{
  bounds.clear();
  bounds.reserve((segments.size() + kPathSegmentBlockSize - 1u) / kPathSegmentBlockSize);
  for (std::size_t first = 0u; first < segments.size(); first += kPathSegmentBlockSize) {
    const auto & segment = segments[first];
    PathSegmentBounds block{
      std::min(first + kPathSegmentBlockSize, segments.size()),
      segment.minimum_x, segment.maximum_x, segment.minimum_y, segment.maximum_y};
    for (std::size_t index = first + 1u; index < block.end; ++index) {
      block.minimum_x = std::min(block.minimum_x, segments[index].minimum_x);
      block.maximum_x = std::max(block.maximum_x, segments[index].maximum_x);
      block.minimum_y = std::min(block.minimum_y, segments[index].minimum_y);
      block.maximum_y = std::max(block.maximum_y, segments[index].maximum_y);
    }
    bounds.push_back(block);
  }
}
}  // namespace f_dwa_controller
#endif  // F_DWA_CONTROLLER__PATH_SEGMENT_BOUNDS_HPP_
