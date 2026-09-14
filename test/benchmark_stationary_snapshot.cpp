// Copyright 2026 YT Lab
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "f_dwa_controller/certified_dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;

template<class Message>
Message read_cdr(const std::string & filename)
{
  std::ifstream input(filename, std::ios::binary | std::ios::ate);
  if (!input || input.tellg() <= 0) {
    throw std::runtime_error("Missing snapshot: " + filename);
  }
  const auto size = static_cast<size_t>(input.tellg());
  input.seekg(0);
  rclcpp::SerializedMessage serialized(size);
  auto & buffer = serialized.get_rcl_serialized_message();
  input.read(reinterpret_cast<char *>(buffer.buffer), size);
  if (!input) {
    throw std::runtime_error("Incomplete snapshot: " + filename);
  }
  buffer.buffer_length = size;
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

template<class Message>
void write_cdr(const std::string & filename, const Message & message)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<Message>().serialize_message(&message, &serialized);
  const auto & buffer = serialized.get_rcl_serialized_message();
  std::ofstream output(filename, std::ios::binary);
  output.write(reinterpret_cast<const char *>(buffer.buffer), buffer.buffer_length);
  if (!output) {
    throw std::runtime_error("Cannot write benchmark result: " + filename);
  }
}

class SnapshotPlanner : public f_dwa_controller::CertifiedDWBLocalPlanner
{
public:
  dwb_msgs::msg::TrajectoryScore best;
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> evaluation;
  double core_ms{0.0};

protected:
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override
  {
    const auto start = SteadyClock::now();
    auto selected = CertifiedDWBLocalPlanner::coreScoringAlgorithm(pose, velocity, results);
    core_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
    best = selected;
    evaluation = results;
    return selected;
  }
};

struct SamplingCase
{
  std::string name;
  int vx;
  int omega;
  std::vector<double> durations;
  bool independent;
};

// Use Nav2's actual projection implementation, with immutable saved layer cells.
// No sensor subscription or map-update thread is started by this fixture.
class FrozenStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  void onInitialize() override {}

  void load(
    const nav2_msgs::msg::Costmap & saved, const std::string & global_frame,
    const nav_msgs::msg::OccupancyGrid & map)
  {
    const auto & metadata = saved.metadata;
    if (saved.data.size() != static_cast<size_t>(metadata.size_x) * metadata.size_y) {
      throw std::runtime_error("Invalid static-layer snapshot");
    }
    if (metadata.size_x != map.info.width || metadata.size_y != map.info.height ||
      metadata.resolution != map.info.resolution ||
      metadata.origin.position.x != map.info.origin.position.x ||
      metadata.origin.position.y != map.info.origin.position.y)
    {
      throw std::runtime_error("Static layer does not match the saved map geometry");
    }
    resizeMap(metadata.size_x, metadata.size_y, metadata.resolution,
      metadata.origin.position.x, metadata.origin.position.y);
    std::memcpy(getCharMap(), saved.data.data(), saved.data.size());
    global_frame_ = global_frame;
    // GetCostmap labels a retained StaticLayer with the master frame. Its cells
    // still use /map geometry; verify geometry above and use the map's frame.
    map_frame_ = map.header.frame_id;
    enabled_ = true;
    map_received_ = true;
    map_received_in_update_bounds_ = true;
    footprint_clearing_enabled_ = false;
    use_maximum_ = false;
    transform_tolerance_ = tf2::durationFromSec(0.0);
  }
};

void spin_for(rclcpp::executors::SingleThreadedExecutor & executor, double seconds)
{
  const auto end = SteadyClock::now() + std::chrono::duration<double>(seconds);
  do {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (SteadyClock::now() < end);
}

void benchmark_footprint(
  const std::string & directory, const std::string & output,
  const std::string & evaluation_file,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
{
  auto evaluation = read_cdr<dwb_msgs::msg::LocalPlanEvaluation>(evaluation_file);
  if (evaluation.twists.empty()) {throw std::runtime_error("Empty candidate snapshot");}
  rclcpp::NodeOptions options;
  options.use_global_arguments(false).automatically_declare_parameters_from_overrides(true);
  options.arguments({"--ros-args", "--params-file", directory + "/controller.yaml"});
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("footprint_benchmark", options);
  f_dwa_controller::HorizonObstacleFootprintCritic critic;
  critic.initialize(parent, "ObstacleFootprint", "f_dwa", costmap);
  f_dwa_controller::CertificationWorkspace workspace;
  critic.setSharedCertificationWorkspace(&workspace);
  const auto & first = evaluation.twists.front().traj.poses.front();
  const nav_2d_msgs::msg::Twist2D velocity;
  const nav_2d_msgs::msg::Path2D plan;
  const int cycles = std::getenv("FOOTPRINT_BENCH_CYCLES") ?
    std::stoi(std::getenv("FOOTPRINT_BENCH_CYCLES")) : 30;
  if (cycles < 1 || cycles > 1000) {throw std::runtime_error("Invalid cycle count");}
  std::ofstream timings(output + "/footprint_timings.csv");
  if (!timings) {throw std::runtime_error("Output directory must exist");}
  timings << "iteration,prepare_ms,score_ms,valid_count\n" << std::setprecision(12);
  for (int cycle = -3; cycle < cycles; ++cycle) {
    const auto start = SteadyClock::now();
    if (!f_dwa_controller::prepare_certification_broadphase(*costmap->getCostmap(), workspace) ||
      !critic.prepare(first, velocity, first, plan))
    {
      throw std::runtime_error("Footprint prepare failed");
    }
    const auto prepared = SteadyClock::now();
    std::size_t valid_count = 0;
    for (auto & candidate : evaluation.twists) {
      candidate.scores.clear();
      try {
        candidate.total = critic.scoreTrajectory(candidate.traj);
        ++valid_count;
      } catch (const dwb_core::IllegalTrajectoryException & error) {
        candidate.total = -1.0;
        dwb_msgs::msg::CriticScore reason;
        reason.name = error.what();
        candidate.scores.push_back(std::move(reason));
      }
    }
    const auto finished = SteadyClock::now();
    if (cycle >= 0) {
      timings << cycle << ',' <<
        std::chrono::duration<double, std::milli>(prepared - start).count() << ',' <<
        std::chrono::duration<double, std::milli>(finished - prepared).count() << ',' <<
        valid_count << '\n';
    }
  }
  write_cdr(output + "/footprint_evaluation.cdr", evaluation);
}
}  // namespace

int main(int argc, char ** argv)
{
  // No Twist publisher, controller server, actuator connection or FollowPath action
  // exists in this executable. Even its synthetic ledger stays in a separate domain.
  if ((argc != 3 && argc != 4) || !std::getenv("ROS_DOMAIN_ID") ||
    std::string(std::getenv("ROS_DOMAIN_ID")) != "94")
  {
    std::cerr <<
      "Usage: ROS_DOMAIN_ID=94 benchmark_stationary_snapshot SNAPSHOT OUTPUT [CANDIDATE_CDR]\n";
    return 2;
  }
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(94);
  rclcpp::init(0, nullptr, init_options);
  try {
    const std::string directory = argv[1];
    const std::string output = argv[2];
    const auto raw = read_cdr<nav2_msgs::msg::Costmap>(directory + "/costmap.cdr");
    const auto static_raw = read_cdr<nav2_msgs::msg::Costmap>(directory + "/static_layer.cdr");
    const auto map = read_cdr<nav_msgs::msg::OccupancyGrid>(directory + "/map.cdr");
    auto pose = read_cdr<geometry_msgs::msg::PoseStamped>(directory + "/pose.cdr");
    auto path = read_cdr<nav_msgs::msg::Path>(directory + "/path.cdr");
    const auto path_tf = read_cdr<geometry_msgs::msg::TransformStamped>(
      directory + "/path_transform.cdr");
    const auto odometry = read_cdr<nav_msgs::msg::Odometry>(directory + "/odometry.cdr");
    if (std::abs(odometry.twist.twist.linear.x) > 0.01 ||
      std::abs(odometry.twist.twist.angular.z) > 0.02)
    {
      throw std::runtime_error("Snapshot is not stationary");
    }
    rclcpp::NodeOptions costmap_options;
    costmap_options.use_global_arguments(false).arguments(
      {"--ros-args", "--params-file", directory + "/costmap.yaml"});
    costmap_options.parameter_overrides({
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}}});
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    costmap->declare_parameter("static_layer.footprint_clearing_enabled", false);
    if (costmap->on_configure(rclcpp_lifecycle::State()) != nav2_util::CallbackReturn::SUCCESS) {
      throw std::runtime_error("Frozen costmap configuration failed");
    }
    const auto & metadata = raw.metadata;
    if (raw.data.size() != static_cast<size_t>(metadata.size_x) * metadata.size_y) {
      throw std::runtime_error("Invalid costmap size");
    }
    costmap->getLayeredCostmap()->resizeMap(
      metadata.size_x, metadata.size_y, metadata.resolution,
      metadata.origin.position.x, metadata.origin.position.y);
    std::memcpy(costmap->getCostmap()->getCharMap(), raw.data.data(), raw.data.size());
    if (costmap->get_parameter("static_layer.footprint_clearing_enabled").as_bool() ||
      costmap->get_parameter("use_maximum").as_bool())
    {
      throw std::runtime_error("Snapshot fixture requires clearing=false and use_maximum=false");
    }
    auto layer_tf = costmap->getTfBuffer();
    if (path_tf.header.frame_id != path_tf.child_frame_id) {
      layer_tf->setTransform(path_tf, "snapshot", true);
    }
    auto static_layer = std::make_shared<FrozenStaticLayer>();
    auto group = costmap->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    static_layer->initialize(costmap->getLayeredCostmap(), "static_layer", layer_tf.get(),
      costmap, group);
    static_layer->load(static_raw, raw.header.frame_id, map);
    costmap->getLayeredCostmap()->addPlugin(static_layer);

    if (argc == 4) {
      benchmark_footprint(directory, output, argv[3], costmap);
      costmap->on_cleanup(rclcpp_lifecycle::State());
      rclcpp::shutdown();
      return 0;
    }

    std::vector<SamplingCase> cases{
      {"baseline165", 11, 15, {}, false},
      {"synchronized495", 11, 15, {0.2, 0.4}, false},
      {"independent1485", 11, 15, {0.2, 0.4}, true},
      {"independent2079", 11, 21, {0.2, 0.4}, true},
      {"amplitude1767", 31, 57, {}, false}};
    if (std::getenv("STATIONARY_BENCH_SAMPLING_SWEEP")) {
      cases = {
        {"baseline165", 11, 15, {}, false},
        {"amplitude315", 21, 15, {}, false},
        {"synchronized330", 11, 15, {0.2}, false},
        {"synchronized442", 13, 17, {0.2}, false},
        {"synchronized494", 13, 19, {0.2}, false},
        {"synchronized630", 21, 15, {0.2}, false},
        {"angular638", 11, 29, {0.2}, false},
        {"synchronized663", 13, 17, {0.2, 0.4}, false},
        {"independent660", 11, 15, {0.2}, true},
        {"independent884", 13, 17, {0.2}, true},
        {"synchronized1218", 21, 29, {0.2}, false}};
    }
    const bool configured_sampling = std::getenv("STATIONARY_BENCH_USE_CONFIG") != nullptr;
    if (configured_sampling) {cases = {{"configured", 0, 0, {}, false}};}
    if (const auto selected = std::getenv("STATIONARY_BENCH_CASE")) {
      cases.erase(std::remove_if(cases.begin(), cases.end(),
        [selected](const auto & sampling) {return sampling.name != selected;}), cases.end());
      if (cases.empty()) {throw std::runtime_error("Unknown sampling case");}
    }
    if (std::getenv("STATIONARY_BENCH_REVERSE")) {
      std::reverse(cases.begin(), cases.end());
    }
    const int cycles = std::getenv("STATIONARY_BENCH_CYCLES") ?
      std::stoi(std::getenv("STATIONARY_BENCH_CYCLES")) : 20;
    if (cycles < 1 || cycles > 1000) {throw std::runtime_error("Invalid cycle count");}
    std::ofstream timings(output + "/timings.csv");
    if (!timings) {
      throw std::runtime_error("Output directory must already exist");
    }
    timings << "case,iteration,full_ms,core_ms,v,omega,cost,candidate_count,valid_count\n";
    timings << std::setprecision(12);
    for (const auto & sampling : cases) {
      rclcpp::NodeOptions options;
      options.use_global_arguments(false).automatically_declare_parameters_from_overrides(true);
      options.arguments({"--ros-args", "--params-file", directory + "/controller.yaml"});
      std::vector<rclcpp::Parameter> overrides{
        {"f_dwa.command_dispatch_topic", "/stationary_benchmark/dispatch"},
        {"f_dwa.transport_valid_topic", "/stationary_benchmark/valid"},
        {"f_dwa.trial_reset_service_name", "/stationary_benchmark/reset"},
        {"f_dwa.transport_invalidation_service", "/stationary_benchmark/invalidate"}};
      if (!configured_sampling) {
        overrides.insert(overrides.end(), {
          {"f_dwa.vx_samples", sampling.vx}, {"f_dwa.vtheta_samples", sampling.omega},
          {"f_dwa.fir_prediction_pulse_durations", sampling.durations},
          {"f_dwa.fir_independent_pulse_durations", sampling.independent}});
      }
      options.parameter_overrides(overrides);
      auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "stationary_benchmark", options);
      auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
      if (path_tf.header.frame_id != path_tf.child_frame_id) {
        tf->setTransform(path_tf, "snapshot", true);
      }
      auto planner = std::make_unique<SnapshotPlanner>();
      planner->configure(parent, "f_dwa", tf, costmap);
      planner->activate();
      for (const auto & key : {"vx_samples", "vtheta_samples", "fir_prediction_pulse_duration",
          "fir_prediction_pulse_durations", "fir_independent_pulse_durations", "sim_time"})
      {
        const auto parameter = parent->get_parameter(std::string("f_dwa.") + key);
        std::cout << "READBACK " << parameter.get_name() << ": " <<
          parameter.value_to_string() << std::endl;
      }
      auto ledger = std::make_shared<rclcpp::Node>("stationary_ledger");
      auto dispatch = ledger->create_publisher<f_dwa_controller::msg::CommandDispatch>(
        "/stationary_benchmark/dispatch", rclcpp::QoS(64).reliable().transient_local());
      auto valid = ledger->create_publisher<std_msgs::msg::Bool>(
        "/stationary_benchmark/valid", rclcpp::QoS(1).reliable().transient_local());
      const auto reset_name = parent->get_parameter("f_dwa.trial_reset_service_name").as_string();
      auto reset = ledger->create_client<std_srvs::srv::Trigger>(reset_name);
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(parent->get_node_base_interface());
      executor.add_node(ledger);
      const auto discovery_deadline = SteadyClock::now() + std::chrono::seconds(10);
      while ((!reset->service_is_ready() || dispatch->get_subscription_count() != 1 ||
        valid->get_subscription_count() != 1) && SteadyClock::now() < discovery_deadline)
      {
        spin_for(executor, 0.05);
      }
      if (!reset->service_is_ready() || dispatch->get_subscription_count() != 1 ||
        valid->get_subscription_count() != 1)
      {
        throw std::runtime_error("Isolated ledger discovery failed: reset=" + reset_name +
                " ready=" + std::to_string(reset->service_is_ready()) +
                " dispatch=" + parent->get_parameter("f_dwa.command_dispatch_topic").as_string() +
                " count=" + std::to_string(dispatch->get_subscription_count()) +
                " valid_count=" + std::to_string(valid->get_subscription_count()));
      }
      for (int iteration = -3; iteration < cycles; ++iteration) {
        auto future =
          reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (executor.spin_until_future_complete(future, std::chrono::seconds(3)) !=
          rclcpp::FutureReturnCode::SUCCESS || !future.get()->success)
        {
          throw std::runtime_error("Isolated planner reset failed");
        }
        planner->setPlan(path);
        f_dwa_controller::msg::CommandDispatch boundary;
        boundary.header.stamp = parent->now();
        boundary.received_at = boundary.header.stamp;
        boundary.received_steady_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch()).count();
        dispatch->publish(boundary);
        std_msgs::msg::Bool valid_state;
        valid_state.data = true;
        valid->publish(valid_state);
        spin_for(executor, 0.02);
        pose.header.stamp = parent->now();
        const auto start = SteadyClock::now();
        const auto command = planner->computeVelocityCommands(pose, odometry.twist.twist, nullptr);
        const double full_ms = std::chrono::duration<double, std::milli>(
          SteadyClock::now() - start).count();
        // Deliberately discard the returned command; nothing sends it to ROS.
        const auto evaluation = planner->evaluation;
        size_t valid_count = 0;
        if (evaluation) {
          for (const auto & candidate : evaluation->twists) {
            if (candidate.total >= 0.0) {++valid_count;}
          }
        }
        if (iteration >= 0) {
          timings << sampling.name << ',' << iteration << ',' << full_ms << ',' <<
            planner->core_ms << ',' << command.twist.linear.x << ',' <<
            command.twist.angular.z << ',' << planner->best.total << ',' <<
            (evaluation ? evaluation->twists.size() : 0) << ',' << valid_count << '\n';
        }
        if (evaluation && !evaluation->twists.empty()) {
          write_cdr(output + "/" + sampling.name + "_evaluation.cdr", *evaluation);
        }
      }
      planner->deactivate();
      planner->cleanup();
      std::cout << "Completed " << sampling.name << std::endl;
    }
    costmap->on_cleanup(rclcpp_lifecycle::State());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Snapshot benchmark failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
