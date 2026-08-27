/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/footprint_clearance_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/fixed_distance_risk_path.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

namespace
{

bool costmapGridsAreAligned(
  const nav2_costmap_2d::Costmap2D * first,
  const nav2_costmap_2d::Costmap2D * second)
{
  if (!first || !second) {
    return false;
  }
  const double resolution = first->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
    std::abs(second->getResolution() - resolution) > 1.0e-9)
  {
    return false;
  }
  const auto is_integer_cell_offset = [resolution](const double offset) {
      const double cells = offset / resolution;
      return std::abs(cells - std::round(cells)) <= 1.0e-6;
    };
  return is_integer_cell_offset(first->getOriginX() - second->getOriginX()) &&
         is_integer_cell_offset(first->getOriginY() - second->getOriginY());
}

// Felzenszwalb and Huttenlocher's one-dimensional squared Euclidean distance
// transform.  Running it across rows and then columns builds an exact
// cell-centre distance field in linear time.  The finite sentinel avoids the
// inf - inf indeterminacy in rows which contain no obstacle.
void squaredDistanceTransform1D(
  const std::vector<double> & input,
  std::vector<double> & output,
  std::vector<int> & sites,
  std::vector<double> & boundaries)
{
  const int count = static_cast<int>(input.size());
  if (count <= 0) {
    output.clear();
    return;
  }
  output.resize(input.size());
  sites.resize(input.size());
  boundaries.resize(input.size() + 1u);
  int envelope = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int query = 1; query < count; ++query) {
    double intersection = 0.0;
    do {
      const int site = sites[envelope];
      intersection =
        ((input[query] + static_cast<double>(query) * query) -
        (input[site] + static_cast<double>(site) * site)) /
        (2.0 * static_cast<double>(query - site));
      if (intersection > boundaries[envelope]) {
        break;
      }
      --envelope;
    } while (envelope >= 0);
    if (envelope < 0) {
      envelope = 0;
    } else {
      ++envelope;
    }
    sites[envelope] = query;
    boundaries[envelope] = intersection;
    boundaries[envelope + 1] = std::numeric_limits<double>::infinity();
  }
  envelope = 0;
  for (int query = 0; query < count; ++query) {
    while (boundaries[envelope + 1] < static_cast<double>(query)) {
      ++envelope;
    }
    const double offset = static_cast<double>(query - sites[envelope]);
    output[query] = offset * offset + input[sites[envelope]];
  }
}

}  // namespace

void FootprintClearanceCritic::onInit()
{
  auto node = node_.lock();
  if (!node || !costmap_ros_) {
    throw std::runtime_error{"FootprintClearanceCritic initialization failed"};
  }
  costmap_ = costmap_ros_->getCostmap();
  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "clearance_margin", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "clearance_bands", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "risk_distance", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "risk_seed_time", rclcpp::ParameterValue(1.4));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "heading_relaxation_distance",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "sample_resolution", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "penalty_power", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "peak_weight", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "source_layer", rclcpp::ParameterValue(std::string{}));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exclude_layer", rclcpp::ParameterValue(std::string{}));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exclude_layer_tolerance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "apply_exclude_tolerance_on_aligned_grids",
    rclcpp::ParameterValue(false));
  node->get_parameter(prefix + "clearance_margin", clearance_margin_);
  node->get_parameter(prefix + "clearance_bands", clearance_bands_);
  node->get_parameter(prefix + "risk_distance", risk_distance_);
  node->get_parameter(prefix + "risk_seed_time", risk_seed_time_);
  node->get_parameter(
    prefix + "heading_relaxation_distance",
    heading_relaxation_distance_);
  node->get_parameter(prefix + "sample_resolution", sample_resolution_);
  node->get_parameter(prefix + "penalty_power", penalty_power_);
  node->get_parameter(prefix + "peak_weight", peak_weight_);
  node->get_parameter(prefix + "source_layer", source_layer_);
  node->get_parameter(prefix + "exclude_layer", exclude_layer_);
  node->get_parameter(
    prefix + "exclude_layer_tolerance", exclude_layer_tolerance_);
  node->get_parameter(
    prefix + "apply_exclude_tolerance_on_aligned_grids",
    apply_exclude_tolerance_on_aligned_grids_);
  if (!std::isfinite(clearance_margin_) || clearance_margin_ <= 0.0 ||
    clearance_bands_ <= 0 ||
    !std::isfinite(risk_distance_) || risk_distance_ <= 1.0e-9 ||
    !std::isfinite(risk_seed_time_) || risk_seed_time_ <= 0.0 ||
    !std::isfinite(heading_relaxation_distance_) ||
    heading_relaxation_distance_ <= 0.0 ||
    !std::isfinite(sample_resolution_) || sample_resolution_ <= 0.0 ||
    !std::isfinite(penalty_power_) || penalty_power_ <= 0.0 ||
    !std::isfinite(peak_weight_) || peak_weight_ < 0.0 || peak_weight_ > 1.0 ||
    !std::isfinite(exclude_layer_tolerance_) ||
    exclude_layer_tolerance_ < 0.0)
  {
    throw std::runtime_error{
            "FootprintClearanceCritic parameters must be finite and positive"};
  }
  const auto plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
  const auto find_costmap_layer =
    [plugins](const std::string & requested_name) ->
    nav2_costmap_2d::CostmapLayer *
    {
      for (const auto & plugin : *plugins) {
        const std::string & plugin_name = plugin->getName();
        const bool name_matches = plugin_name == requested_name ||
          (plugin_name.size() > requested_name.size() &&
          plugin_name.compare(
            plugin_name.size() - requested_name.size(),
            requested_name.size(), requested_name) == 0);
        if (!name_matches) {
          continue;
        }
        const auto layer =
          std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(plugin);
        if (layer) {
          return layer.get();
        }
      }
      return nullptr;
    };
  if (!source_layer_.empty()) {
    costmap_ = find_costmap_layer(source_layer_);
    if (!costmap_) {
      throw std::runtime_error{
              "FootprintClearanceCritic source_layer was not found: " +
              source_layer_};
    }
  }
  if (!exclude_layer_.empty()) {
    exclusion_costmap_layer_ = find_costmap_layer(exclude_layer_);
    if (!exclusion_costmap_layer_) {
      throw std::runtime_error{
              "FootprintClearanceCritic exclude_layer was not found: " +
              exclude_layer_};
    }
    exclusion_costmap_ = exclusion_costmap_layer_;
    // A rolling master Costmap is expressed in its global frame, while a
    // StaticLayer retains the original map grid and projects it only from
    // updateCosts(). Querying that retained grid with rolling-frame
    // coordinates silently misses every duplicate on a real AMCL stack.
    // Reuse the layer's own projection path into a master-aligned scratch map.
    project_exclusion_costmap_ =
      costmap_ == costmap_ros_->getCostmap() &&
      costmap_ros_->getLayeredCostmap()->isRolling();
  }
}

void FootprintClearanceCritic::refreshProjectedExclusionCostmap()
{
  if (!project_exclusion_costmap_ || !exclusion_costmap_layer_ ||
    !costmap_)
  {
    return;
  }
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const double resolution = costmap_->getResolution();
  if (size_x == 0u || size_y == 0u ||
    !std::isfinite(resolution) || resolution <= 0.0)
  {
    return;
  }
  const double origin_x = costmap_->getOriginX();
  const double origin_y = costmap_->getOriginY();
  const bool geometry_changed =
    projected_exclusion_costmap_.getSizeInCellsX() != size_x ||
    projected_exclusion_costmap_.getSizeInCellsY() != size_y ||
    projected_exclusion_costmap_.getResolution() != resolution;
  if (geometry_changed) {
    projected_exclusion_costmap_.resizeMap(
      size_x, size_y, resolution, origin_x, origin_y);
  } else {
    projected_exclusion_costmap_.updateOrigin(origin_x, origin_y);
  }
  projected_exclusion_costmap_.resetMapToValue(
    0u, 0u, size_x, size_y, nav2_costmap_2d::FREE_SPACE);
  exclusion_costmap_layer_->updateCosts(
    projected_exclusion_costmap_, 0, 0,
    static_cast<int>(size_x), static_cast<int>(size_y));
  exclusion_costmap_ = &projected_exclusion_costmap_;
}

bool FootprintClearanceCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  prepared_ = false;
  prepared_pose_penalty_valid_ = false;
  prepared_plan_geometry_ = PreparedPlanGeometry{};
  global_plan_.poses.clear();
  if (!costmap_ || !costmap_ros_ ||
    !std::isfinite(risk_distance_) || risk_distance_ <= 1.0e-9 ||
    global_plan.poses.empty())
  {
    return false;
  }
  try {
    prepared_plan_geometry_ =
      prepare_plan_continuation_geometry(global_plan);
  } catch (const std::invalid_argument &) {
    return false;
  }
  global_plan_ = global_plan;
  physical_footprint_ = costmap_ros_->getRobotFootprint();
  if (physical_footprint_.size() < 3u) {
    return false;
  }
  if (!refreshFootprintBoundarySamples()) {
    return false;
  }
  refreshProjectedExclusionCostmap();
  expanded_footprints_.clear();
  expanded_footprints_.reserve(static_cast<std::size_t>(clearance_bands_));
  for (int band = 1; band <= clearance_bands_; ++band) {
    auto expanded = physical_footprint_;
    const double padding = clearance_margin_ *
      static_cast<double>(band) / static_cast<double>(clearance_bands_);
    nav2_costmap_2d::padFootprint(expanded, padding);
    expanded_footprints_.push_back(std::move(expanded));
  }
  refreshPenalizedCellMask();
  prepared_ = obstacle_distance_field_.size() ==
    static_cast<std::size_t>(costmap_->getSizeInCellsX()) *
    static_cast<std::size_t>(costmap_->getSizeInCellsY());
  if (prepared_ && std::isfinite(pose.x) && std::isfinite(pose.y) &&
    std::isfinite(pose.theta))
  {
    prepared_pose_ = pose;
    prepared_pose_penalty_ = scorePoseClearance(pose);
    prepared_pose_penalty_valid_ =
      std::isfinite(prepared_pose_penalty_);
  }
  return prepared_;
}

bool FootprintClearanceCritic::excludedByStaticLayer(
  const double world_x, const double world_y) const
{
  if (!exclusion_costmap_) {
    return false;
  }
  unsigned int exclusion_x = 0u;
  unsigned int exclusion_y = 0u;
  if (!exclusion_costmap_->worldToMap(
      world_x, world_y, exclusion_x, exclusion_y))
  {
    return false;
  }
  // Exact overlap is always a duplicate. By default, neighbouring observations
  // on an aligned grid remain real protrusions/narrowing; simulation may opt in
  // to the bounded tolerance below to absorb known map/mesh rasterization error.
  if (exclusion_costmap_->getCost(exclusion_x, exclusion_y) ==
    nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    return true;
  }
  if (!exclusion_tolerance_active_) {
    return false;
  }

  // Differently registered grids use the tolerance fallback automatically.
  // Aligned grids reach it only through the explicit simulation opt-in above.
  for (const auto & [dx, dy] : exclusion_tolerance_offsets_) {
    const int map_x = static_cast<int>(exclusion_x) + dx;
    const int map_y = static_cast<int>(exclusion_y) + dy;
    if (map_x < 0 || map_y < 0 ||
      map_x >= static_cast<int>(exclusion_costmap_->getSizeInCellsX()) ||
      map_y >= static_cast<int>(exclusion_costmap_->getSizeInCellsY()))
    {
      continue;
    }
    if (exclusion_costmap_->getCost(
        static_cast<unsigned int>(map_x),
        static_cast<unsigned int>(map_y)) ==
      nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      return true;
    }
  }
  return false;
}

bool FootprintClearanceCritic::refreshPenalizedCellMask()
{
  const auto invalidate_distance_field = [this]() {
      penalized_cell_mask_.clear();
      penalized_cell_mask_scratch_.clear();
      obstacle_distance_field_.clear();
      distance_transform_input_scratch_.clear();
      distance_transform_output_scratch_.clear();
      distance_transform_sites_scratch_.clear();
      distance_transform_boundaries_scratch_.clear();
      row_distance_scratch_.clear();
      distance_field_size_x_ = 0u;
      distance_field_size_y_ = 0u;
      distance_field_resolution_ =
        std::numeric_limits<double>::quiet_NaN();
    };
  if (!costmap_) {
    invalidate_distance_field();
    return false;
  }
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  if (size_x == 0u || size_y == 0u) {
    invalidate_distance_field();
    return false;
  }
  const std::size_t cell_count =
    static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  exclusion_tolerance_active_ = exclusion_costmap_ &&
    exclude_layer_tolerance_ > 0.0 &&
    (!costmapGridsAreAligned(costmap_, exclusion_costmap_) ||
    apply_exclude_tolerance_on_aligned_grids_);
  if (exclusion_tolerance_active_) {
    const double exclusion_resolution = exclusion_costmap_->getResolution();
    if (exclusion_tolerance_offsets_.empty() ||
      exclusion_offsets_resolution_ != exclusion_resolution ||
      exclusion_offsets_tolerance_ != exclude_layer_tolerance_)
    {
      exclusion_tolerance_offsets_.clear();
      exclusion_offsets_resolution_ = exclusion_resolution;
      exclusion_offsets_tolerance_ = exclude_layer_tolerance_;
      const int cell_radius = static_cast<int>(std::ceil(
          exclude_layer_tolerance_ / exclusion_resolution));
      const double inclusive_radius =
        exclude_layer_tolerance_ + 0.5 * exclusion_resolution;
      const double inclusive_radius_squared =
        inclusive_radius * inclusive_radius;
      exclusion_tolerance_offsets_.reserve(
        static_cast<std::size_t>(2 * cell_radius + 1) *
        static_cast<std::size_t>(2 * cell_radius + 1));
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const double offset_x = dx * exclusion_resolution;
          const double offset_y = dy * exclusion_resolution;
          if (offset_x * offset_x + offset_y * offset_y <=
            inclusive_radius_squared)
          {
            exclusion_tolerance_offsets_.emplace_back(dx, dy);
          }
        }
      }
    }
  }
  penalized_cell_mask_scratch_.resize(cell_count);
  bool has_penalized_cell = false;
  if (!exclusion_costmap_) {
    // This is the primary research configuration.  With no exclusion layer,
    // the exact source mask is a direct byte-wise predicate over the master
    // Costmap; converting every lethal cell to world coordinates cannot
    // change the result and only adds work.
    const unsigned char * const source = costmap_->getCharMap();
    if (!source) {
      invalidate_distance_field();
      return false;
    }
    for (std::size_t index = 0u; index < cell_count; ++index) {
      const bool penalized =
        source[index] == nav2_costmap_2d::LETHAL_OBSTACLE;
      penalized_cell_mask_scratch_[index] = penalized ? 1u : 0u;
      has_penalized_cell = has_penalized_cell || penalized;
    }
  } else {
    // Preserve the coordinate-aware compatibility path exactly when a layer
    // must be excluded, including differently registered Costmap grids.
    std::fill(
      penalized_cell_mask_scratch_.begin(),
      penalized_cell_mask_scratch_.end(), 0u);
    for (unsigned int map_y = 0u; map_y < size_y; ++map_y) {
      for (unsigned int map_x = 0u; map_x < size_x; ++map_x) {
        if (costmap_->getCost(map_x, map_y) !=
          nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          continue;
        }
        double world_x = 0.0;
        double world_y = 0.0;
        costmap_->mapToWorld(map_x, map_y, world_x, world_y);
        if (!excludedByStaticLayer(world_x, world_y)) {
          penalized_cell_mask_scratch_[
            static_cast<std::size_t>(map_y) * size_x + map_x] = 1u;
          has_penalized_cell = true;
        }
      }
    }
  }

  const double resolution = costmap_->getResolution();
  // Exclusion is intentionally re-evaluated above on every prepare().  Only
  // the exact grid-index mask can be cached: changes in either source layer,
  // exclusion layer registration/content, grid shape, or resolution rebuild
  // the field.  Origin alone is not part of the EDT because worldToMap applies
  // the current origin before querying this grid-index field.
  if (distance_field_size_x_ == size_x &&
    distance_field_size_y_ == size_y &&
    distance_field_resolution_ == resolution &&
    obstacle_distance_field_.size() == cell_count &&
    penalized_cell_mask_ == penalized_cell_mask_scratch_)
  {
    return false;
  }

  penalized_cell_mask_.swap(penalized_cell_mask_scratch_);
  distance_field_size_x_ = size_x;
  distance_field_size_y_ = size_y;
  distance_field_resolution_ = resolution;
  obstacle_distance_field_.assign(
    cell_count, std::numeric_limits<float>::infinity());
  if (!has_penalized_cell) {
    return true;
  }

  const double maximum_squared_distance =
    4.0 * (static_cast<double>(size_x) * size_x +
    static_cast<double>(size_y) * size_y) + 1.0;
  // Every element of these buffers is overwritten before it is read. Keeping
  // their capacity across Costmap updates removes allocator traffic without
  // retaining any prior distance-transform value.
  row_distance_scratch_.resize(cell_count);
  const std::size_t maximum_dimension = std::max(size_x, size_y);
  distance_transform_input_scratch_.reserve(maximum_dimension);
  distance_transform_output_scratch_.reserve(maximum_dimension);
  distance_transform_sites_scratch_.reserve(maximum_dimension);
  distance_transform_boundaries_scratch_.reserve(maximum_dimension + 1u);
  for (unsigned int map_y = 0u; map_y < size_y; ++map_y) {
    const std::size_t row_offset = static_cast<std::size_t>(map_y) * size_x;
    int nearest_penalized_x = -1;
    for (unsigned int map_x = 0u; map_x < size_x; ++map_x) {
      if (penalized_cell_mask_[row_offset + map_x] != 0u) {
        nearest_penalized_x = static_cast<int>(map_x);
        row_distance_scratch_[row_offset + map_x] = 0.0;
      } else if (nearest_penalized_x >= 0) {
        const double distance = static_cast<double>(
          static_cast<int>(map_x) - nearest_penalized_x);
        row_distance_scratch_[row_offset + map_x] = distance * distance;
      } else {
        row_distance_scratch_[row_offset + map_x] =
          maximum_squared_distance;
      }
    }
    nearest_penalized_x = -1;
    for (unsigned int reverse_x = size_x; reverse_x > 0u; --reverse_x) {
      const unsigned int map_x = reverse_x - 1u;
      if (penalized_cell_mask_[row_offset + map_x] != 0u) {
        nearest_penalized_x = static_cast<int>(map_x);
      } else if (nearest_penalized_x >= 0) {
        const double distance = static_cast<double>(
          nearest_penalized_x - static_cast<int>(map_x));
        row_distance_scratch_[row_offset + map_x] = std::min(
          row_distance_scratch_[row_offset + map_x], distance * distance);
      }
    }
  }

  distance_transform_input_scratch_.resize(size_y);
  for (unsigned int map_x = 0u; map_x < size_x; ++map_x) {
    for (unsigned int map_y = 0u; map_y < size_y; ++map_y) {
      distance_transform_input_scratch_[map_y] = row_distance_scratch_[
        static_cast<std::size_t>(map_y) * size_x + map_x];
    }
    squaredDistanceTransform1D(
      distance_transform_input_scratch_,
      distance_transform_output_scratch_,
      distance_transform_sites_scratch_,
      distance_transform_boundaries_scratch_);
    for (unsigned int map_y = 0u; map_y < size_y; ++map_y) {
      obstacle_distance_field_[
        static_cast<std::size_t>(map_y) * size_x + map_x] =
        static_cast<float>(
        std::sqrt(distance_transform_output_scratch_[map_y]) *
        resolution);
    }
  }
  return true;
}

bool FootprintClearanceCritic::refreshFootprintBoundarySamples()
{
  footprint_boundary_samples_.clear();
  maximum_footprint_probe_gap_ = 0.0;
  if (!costmap_ || physical_footprint_.size() < 3u) {
    return false;
  }
  // This is a ranking margin, not the independent hard-footprint gate.  One
  // probe per two soft-clearance bands is sufficient because the clearance
  // query subtracts half of the measured maximum probe gap below.  The
  // distance-to-obstacle function is 1-Lipschitz, so this remains a lower
  // bound for the continuously swept footprint between adjacent probes while
  // halving the soft-ranking work on the 0.025 m experiment Costmap.  The
  // independent physical-footprint critic keeps its 0.0125 m hard sweep.
  const double band_width = clearance_margin_ /
    static_cast<double>(std::max(1, clearance_bands_));
  const double probe_spacing = std::max(
    costmap_->getResolution(),
    std::min(sample_resolution_, 2.0 * band_width));
  if (!std::isfinite(probe_spacing) || probe_spacing <= 0.0) {
    return false;
  }
  for (std::size_t index = 0u; index < physical_footprint_.size(); ++index) {
    const auto & first = physical_footprint_[index];
    const auto & second =
      physical_footprint_[(index + 1u) % physical_footprint_.size()];
    if (!std::isfinite(first.x) || !std::isfinite(first.y) ||
      !std::isfinite(second.x) || !std::isfinite(second.y))
    {
      footprint_boundary_samples_.clear();
      return false;
    }
    const double edge_x = second.x - first.x;
    const double edge_y = second.y - first.y;
    const double edge_length = std::hypot(edge_x, edge_y);
    if (!std::isfinite(edge_length) || edge_length <= 1.0e-12) {
      continue;
    }
    const std::size_t intervals = std::max<std::size_t>(
      1u, static_cast<std::size_t>(std::ceil(edge_length / probe_spacing)));
    const double interval_length = edge_length / static_cast<double>(intervals);
    maximum_footprint_probe_gap_ = std::max(
      maximum_footprint_probe_gap_, interval_length);
    for (std::size_t sample = 0u; sample < intervals; ++sample) {
      const double ratio =
        static_cast<double>(sample) / static_cast<double>(intervals);
      auto point = first;
      point.x = first.x + ratio * edge_x;
      point.y = first.y + ratio * edge_y;
      footprint_boundary_samples_.push_back(point);
    }
  }
  return footprint_boundary_samples_.size() >= 3u &&
         maximum_footprint_probe_gap_ > 0.0;
}

double FootprintClearanceCritic::minimumFootprintClearance(
  const geometry_msgs::msg::Pose2D & pose) const
{
  if (!costmap_ || footprint_boundary_samples_.empty() ||
    obstacle_distance_field_.size() !=
    static_cast<std::size_t>(costmap_->getSizeInCellsX()) *
    static_cast<std::size_t>(costmap_->getSizeInCellsY()) ||
    !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.theta))
  {
    return 0.0;
  }
  const double resolution = costmap_->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 0.0;
  }
  const double obstacle_cell_radius = std::sqrt(0.5) * resolution;
  const double probe_gap_radius = 0.5 * maximum_footprint_probe_gap_;
  const double origin_x = costmap_->getOriginX();
  const double origin_y = costmap_->getOriginY();
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const double maximum_x = origin_x +
    static_cast<double>(size_x) * resolution;
  const double maximum_y = origin_y +
    static_cast<double>(costmap_->getSizeInCellsY()) * resolution;
  const double cosine = std::cos(pose.theta);
  const double sine = std::sin(pose.theta);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (const auto & point : footprint_boundary_samples_) {
    const double world_x = pose.x + point.x * cosine - point.y * sine;
    const double world_y = pose.y + point.x * sine + point.y * cosine;
    minimum_clearance = std::min(
      minimum_clearance,
      std::min({
        world_x - origin_x, maximum_x - world_x,
        world_y - origin_y, maximum_y - world_y}));
    if (minimum_clearance <= 0.0) {
      return 0.0;
    }
    unsigned int cell_x = 0u;
    unsigned int cell_y = 0u;
    if (!costmap_->worldToMap(world_x, world_y, cell_x, cell_y)) {
      return 0.0;
    }
    const std::size_t field_index =
      static_cast<std::size_t>(cell_y) * size_x + cell_x;
    const double cell_centre_distance = obstacle_distance_field_[field_index];
    if (!std::isfinite(cell_centre_distance)) {
      continue;
    }
    const double cell_centre_x =
      origin_x + (static_cast<double>(cell_x) + 0.5) * resolution;
    const double cell_centre_y =
      origin_y + (static_cast<double>(cell_y) + 0.5) * resolution;
    // The cell-centre distance transform is 1-Lipschitz.  Subtracting the
    // query-to-cell-centre offset gives a lower bound at the exact probe.
    // The other two terms conservatively cover the occupied cell's square
    // area and the unsampled interval between adjacent footprint probes.
    const double centre_dx = world_x - cell_centre_x;
    const double centre_dy = world_y - cell_centre_y;
    const double conservative_clearance = cell_centre_distance -
      std::sqrt(centre_dx * centre_dx + centre_dy * centre_dy) -
      obstacle_cell_radius - probe_gap_radius;
    minimum_clearance = std::min(minimum_clearance, conservative_clearance);
    if (minimum_clearance <= 0.0) {
      return 0.0;
    }
  }
  return std::max(0.0, minimum_clearance);
}

bool FootprintClearanceCritic::expandedFootprintHitsLethal(
  const geometry_msgs::msg::Pose2D & pose,
  const std::size_t band_index) const
{
  if (!costmap_ || band_index >= expanded_footprints_.size()) {
    return true;
  }
  const double clearance = minimumFootprintClearance(pose);
  const double padding = clearance_margin_ *
    static_cast<double>(band_index + 1u) /
    static_cast<double>(expanded_footprints_.size());
  if (std::isinf(clearance) && clearance > 0.0) {
    return false;
  }
  return !std::isfinite(clearance) || clearance <= padding;
}

double FootprintClearanceCritic::scorePoseClearance(
  const geometry_msgs::msg::Pose2D & pose) const
{
  if (expanded_footprints_.empty() ||
    !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.theta))
  {
    return 1.0;
  }
  const double clearance = minimumFootprintClearance(pose);
  if (std::isinf(clearance) && clearance > 0.0) {
    return 0.0;
  }
  if (!std::isfinite(clearance) || clearance <= 0.0) {
    return 1.0;
  }
  if (clearance > clearance_margin_) {
    return 0.0;
  }
  const double remaining_fraction = std::clamp(
    1.0 - clearance / clearance_margin_, 0.0, 1.0);
  return std::pow(remaining_fraction, penalty_power_);
}

double FootprintClearanceCritic::scorePoseClearanceWithPreparedPoseCache(
  const geometry_msgs::msg::Pose2D & pose) const
{
  if (prepared_pose_penalty_valid_ &&
    pose.x == prepared_pose_.x && pose.y == prepared_pose_.y &&
    pose.theta == prepared_pose_.theta)
  {
    return prepared_pose_penalty_;
  }
  return scorePoseClearance(pose);
}

double FootprintClearanceCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  return scoreTrajectoryWithApproachRisk(trajectory, nullptr);
}

double FootprintClearanceCritic::scoreTrajectoryWithApproachRisk(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  double * approach_risk)
{
  if (approach_risk) {
    *approach_risk = 0.0;
  }
  if (!prepared_) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "clearance field is not prepared");
  }
  if (!std::isfinite(risk_distance_) || risk_distance_ <= 1.0e-9) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "risk_distance must be finite and greater than 1e-9");
  }
  // A pure maximum over the complete rollout makes the common first pose dominate
  // as soon as the robot is inside the soft band: stopping, turning away and
  // continuing toward the obstacle all receive the same score.  The physical
  // footprint critic independently rejects actual collisions, so use the
  // distance-normalized exposure integral to preserve that escape gradient.
  // Retain a configurable peak component so a short future encounter is not
  // diluted by the otherwise-clear soft horizon.
  double maximum_penalty = 0.0;
  // This is a soft margin, not the physical collision gate.  Sampling at the
  // 0.025 m Costmap cell size made a 2.5 m probe evaluate 101 poses for every
  // candidate.  Bound the spatial gap by half the configured clearance band
  // instead: an obstacle entering the 0.25 m soft margin is then at most one
  // half-gap from a probe, while HorizonObstacleFootprintCritic continues its
  // independent 0.0125 m hard swept-footprint check.
  const double effective_resolution = std::min(
    sample_resolution_, 0.5 * clearance_margin_);
  const auto & risk_path = [this, &trajectory, effective_resolution]()
    -> const std::vector<RiskPathSample> &
    {
      try {
        if (!prepared_plan_geometry_.empty()) {
          return build_plan_continued_risk_path(
            trajectory, prepared_plan_geometry_, risk_distance_,
            effective_resolution, risk_seed_time_,
            heading_relaxation_distance_, risk_path_workspace_);
        }
        // Compatibility for derived tests which directly seed global_plan_.
        // Production prepare() always builds the cache before scoring.
        return build_plan_continued_risk_path(
          trajectory, global_plan_, risk_distance_, effective_resolution,
          risk_seed_time_, heading_relaxation_distance_,
          risk_path_workspace_);
      } catch (const std::invalid_argument & exception) {
        throw dwb_core::IllegalTrajectoryException(name_, exception.what());
      }
    }();
  if (risk_path.size() < 2u) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "fixed-distance clearance path is incomplete");
  }

  double previous_penalty =
    scorePoseClearanceWithPreparedPoseCache(risk_path.front().pose);
  maximum_penalty = previous_penalty;
  double exposure_integral = 0.0;
  for (std::size_t index = 1u; index < risk_path.size(); ++index) {
    const double penalty = scorePoseClearanceWithPreparedPoseCache(
      risk_path[index].pose);
    maximum_penalty = std::max(maximum_penalty, penalty);
    const double interval =
      risk_path[index].arc_length - risk_path[index - 1u].arc_length;
    if (!std::isfinite(interval) || interval <= 0.0) {
      throw dwb_core::IllegalTrajectoryException(
              name_, "fixed-distance clearance path is not strictly ordered");
    }
    exposure_integral += 0.5 * interval *
      (previous_penalty + penalty);
    previous_penalty = penalty;
  }
  const double mean_penalty = std::clamp(
    exposure_integral / risk_distance_, 0.0, 1.0);
  if (approach_risk) {
    // Activation must describe the candidate the Controller can actually
    // dispatch. The fixed-distance score above deliberately continues beyond
    // the executable endpoint along the Path; using that continuation here
    // made every candidate appear to approach the same future wall and kept
    // the avoidance hierarchy active over the entire route.
    constexpr double kTimeTolerance = 1.0e-9;
    constexpr double kAngularSampleResolution = 0.10;
    double minimum_candidate_penalty =
      scorePoseClearanceWithPreparedPoseCache(trajectory.poses.front());
    double maximum_approach_risk = 0.0;
    geometry_msgs::msg::Pose2D previous_sample = trajectory.poses.front();
    const auto update_approach =
      [this, &minimum_candidate_penalty, &maximum_approach_risk,
        &previous_sample](const geometry_msgs::msg::Pose2D & pose)
      {
        const double penalty = scorePoseClearanceWithPreparedPoseCache(pose);
        if (penalty > minimum_candidate_penalty) {
          const double available_increase = std::max(
            1.0 - minimum_candidate_penalty, 1.0e-9);
          maximum_approach_risk = std::max(
            maximum_approach_risk,
            std::clamp(
              (penalty - minimum_candidate_penalty) /
              available_increase,
              0.0, 1.0));
        }
        minimum_candidate_penalty = std::min(
          minimum_candidate_penalty, penalty);
        previous_sample = pose;
      };
    const std::size_t timed_pose_count = trajectory.time_offsets.size();
    for (std::size_t index = 1u; index < timed_pose_count; ++index) {
      const auto duration_seconds = [](const auto & duration) {
          return static_cast<double>(duration.sec) +
                 1.0e-9 * static_cast<double>(duration.nanosec);
        };
      const double pose_time = duration_seconds(
        trajectory.time_offsets[index]);
      geometry_msgs::msg::Pose2D candidate_pose = trajectory.poses[index];
      bool reached_seed =
        pose_time >= risk_seed_time_ - kTimeTolerance;
      if (pose_time > risk_seed_time_ + kTimeTolerance) {
        const double previous_time = duration_seconds(
          trajectory.time_offsets[index - 1u]);
        const double ratio = std::clamp(
          (risk_seed_time_ - previous_time) /
          (pose_time - previous_time), 0.0, 1.0);
        const auto & first = trajectory.poses[index - 1u];
        const auto & second = trajectory.poses[index];
        candidate_pose.x = first.x + ratio * (second.x - first.x);
        candidate_pose.y = first.y + ratio * (second.y - first.y);
        candidate_pose.theta = first.theta + ratio * std::remainder(
          second.theta - first.theta, 2.0 * M_PI);
        reached_seed = true;
      }
      const double translation = std::hypot(
        candidate_pose.x - previous_sample.x,
        candidate_pose.y - previous_sample.y);
      const double rotation = std::abs(std::remainder(
        candidate_pose.theta - previous_sample.theta, 2.0 * M_PI));
      const bool final_timed_pose = index + 1u == timed_pose_count;
      if (translation >= effective_resolution - 1.0e-12 ||
        rotation >= kAngularSampleResolution - 1.0e-12 ||
        reached_seed || final_timed_pose)
      {
        update_approach(candidate_pose);
      }
      if (reached_seed) {
        break;
      }
    }
    *approach_risk = maximum_approach_risk;
  }
  return peak_weight_ * maximum_penalty +
         (1.0 - peak_weight_) * mean_penalty;
}

double FootprintClearanceCritic::scoreUniformPoseSequenceWithApproachRisk(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double * approach_risk)
{
  if (approach_risk) {
    *approach_risk = 0.0;
  }
  if (!prepared_) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "clearance field is not prepared");
  }
  if (poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(
            name_, "executable pose sequence is empty");
  }

  double previous_penalty =
    scorePoseClearanceWithPreparedPoseCache(poses.front());
  double minimum_penalty = previous_penalty;
  double maximum_penalty = previous_penalty;
  double maximum_approach_risk = 0.0;
  double exposure_sum = 0.0;
  for (std::size_t index = 1u; index < poses.size(); ++index) {
    const bool repeats_previous_pose =
      poses[index].x == poses[index - 1u].x &&
      poses[index].y == poses[index - 1u].y &&
      poses[index].theta == poses[index - 1u].theta;
    const double penalty = repeats_previous_pose ? previous_penalty :
      scorePoseClearanceWithPreparedPoseCache(poses[index]);
    maximum_penalty = std::max(maximum_penalty, penalty);
    exposure_sum += 0.5 * (previous_penalty + penalty);
    if (penalty > minimum_penalty) {
      const double available_increase = std::max(
        1.0 - minimum_penalty, 1.0e-9);
      maximum_approach_risk = std::max(
        maximum_approach_risk,
        std::clamp(
          (penalty - minimum_penalty) / available_increase,
          0.0, 1.0));
    }
    minimum_penalty = std::min(minimum_penalty, penalty);
    previous_penalty = penalty;
  }
  if (approach_risk) {
    *approach_risk = maximum_approach_risk;
  }
  const double mean_penalty = poses.size() == 1u ?
    maximum_penalty :
    std::clamp(
    exposure_sum / static_cast<double>(poses.size() - 1u), 0.0, 1.0);
  return peak_weight_ * maximum_penalty +
         (1.0 - peak_weight_) * mean_penalty;
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::FootprintClearanceCritic,
  dwb_core::TrajectoryCritic)
