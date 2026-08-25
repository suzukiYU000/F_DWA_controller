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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "f_dwa_controller/command_delay_node.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace f_dwa_controller
{

class CommandDelayNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(CommandDelayNodeTest, TrialResetIsAppliedOnlyAtNextTimerBoundary)
{
  constexpr double kPublishFrequencyHz = 2.0;
  const std::string service_name =
    "/f_dwa_controller_test/reset_trial_state";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter(
        "input_topic", "/f_dwa_controller_test/input"),
      rclcpp::Parameter(
        "output_topic", "/f_dwa_controller_test/output"),
      rclcpp::Parameter(
        "applied_topic", "/f_dwa_controller_test/applied"),
      rclcpp::Parameter(
        "dispatch_topic", "/f_dwa_controller_test/dispatch"),
      rclcpp::Parameter(
        "transport_valid_topic", "/f_dwa_controller_test/valid"),
      rclcpp::Parameter(
        "transport_stopped_topic", "/f_dwa_controller_test/stopped"),
      rclcpp::Parameter(
        "diagnostics_topic", "/f_dwa_controller_test/diagnostics"),
      rclcpp::Parameter("reset_trial_service_name", service_name),
      rclcpp::Parameter("publish_frequency_hz", kPublishFrequencyHz),
      rclcpp::Parameter("random_seed", 42)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_reset_client_test");
  const auto client =
    client_node->create_client<std_srvs::srv::Trigger>(service_name);
  std::vector<geometry_msgs::msg::Twist> outputs;
  std::vector<geometry_msgs::msg::Twist> applied_commands;
  std::vector<f_dwa_controller::msg::CommandDispatch> dispatches;
  std::vector<bool> valid_states;
  std::vector<bool> stopped_states;
  auto output_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    "/f_dwa_controller_test/output",
    rclcpp::QoS(1).reliable(),
    [&outputs](const geometry_msgs::msg::Twist::SharedPtr message)
    {
      if (message) {
        outputs.push_back(*message);
      }
    });
  auto applied_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    "/f_dwa_controller_test/applied",
    rclcpp::QoS(1).reliable(),
    [&applied_commands](const geometry_msgs::msg::Twist::SharedPtr message)
    {
      if (message) {
        applied_commands.push_back(*message);
      }
    });
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    "/f_dwa_controller_test/dispatch",
    rclcpp::QoS(1).reliable().transient_local(),
    [&dispatches](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      if (message) {
        dispatches.push_back(*message);
      }
    });
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    "/f_dwa_controller_test/valid",
    rclcpp::QoS(1).reliable().transient_local(),
    [&valid_states](const std_msgs::msg::Bool::SharedPtr message)
    {
      if (message) {
        valid_states.push_back(message->data);
      }
    });
  auto stopped_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    "/f_dwa_controller_test/stopped",
    rclcpp::QoS(1).reliable().transient_local(),
    [&stopped_states](const std_msgs::msg::Bool::SharedPtr message)
    {
      if (message) {
        stopped_states.push_back(message->data);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  // Consume constructor/periodic state before marking the reset boundary.
  const auto initial_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
  while (std::chrono::steady_clock::now() < initial_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  outputs.clear();
  applied_commands.clear();
  dispatches.clear();
  valid_states.clear();
  stopped_states.clear();

  auto future =
    client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("seed 42"), std::string::npos);
  EXPECT_NE(response->message.find("scheduled"), std::string::npos);
  EXPECT_NE(response->message.find("next command Timer tick"), std::string::npos);

  // A successful service response only schedules reset. No robot-facing or
  // retained reset state may be emitted from the service callback itself.
  const auto before_timer_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
  while (std::chrono::steady_clock::now() < before_timer_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(outputs.empty());
  EXPECT_TRUE(applied_commands.empty());
  EXPECT_TRUE(dispatches.empty());
  EXPECT_TRUE(valid_states.empty());
  EXPECT_TRUE(stopped_states.empty());

  const auto dispatch_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((outputs.empty() || applied_commands.empty() ||
    dispatches.empty() || valid_states.empty() || stopped_states.empty()) &&
    std::chrono::steady_clock::now() < dispatch_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(outputs.size(), 1u);
  ASSERT_EQ(applied_commands.size(), 1u);
  ASSERT_EQ(dispatches.size(), 1u);
  ASSERT_EQ(valid_states.size(), 1u);
  ASSERT_EQ(stopped_states.size(), 1u);
  EXPECT_DOUBLE_EQ(outputs.front().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(outputs.front().angular.z, 0.0);
  EXPECT_DOUBLE_EQ(applied_commands.front().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(applied_commands.front().angular.z, 0.0);
  EXPECT_FALSE(dispatches.front().has_sequence);
  EXPECT_DOUBLE_EQ(dispatches.front().command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(dispatches.front().command.angular.z, 0.0);
  EXPECT_NE(dispatches.front().header.stamp.sec, 0);
  EXPECT_EQ(dispatches.front().received_at, dispatches.front().header.stamp);
  EXPECT_GT(dispatches.front().received_steady_time_ns, 0u);
  EXPECT_TRUE(valid_states.front());
  EXPECT_TRUE(stopped_states.front());

  const auto stable_state_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(120);
  while (std::chrono::steady_clock::now() < stable_state_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(stopped_states.size(), 1u);

  stopped_subscriber.reset();
  valid_subscriber.reset();
  dispatch_subscriber.reset();
  applied_subscriber.reset();
  output_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, LateSubscriberReceivesOnlyLatestDispatchState)
{
  const std::string prefix = "/f_dwa_controller_dispatch_history_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state")});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_dispatch_history_client_test");
  const auto client =
    client_node->create_client<std_srvs::srv::Trigger>(
    prefix + "/reset_trial_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  for (int reset_index = 0; reset_index < 3; ++reset_index) {
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(
      executor.spin_until_future_complete(future, std::chrono::seconds(1)),
      rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }

  std::vector<f_dwa_controller::msg::CommandDispatch> retained_dispatches;
  auto late_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch",
    rclcpp::QoS(10).reliable().transient_local(),
    [&retained_dispatches](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      if (message) {
        retained_dispatches.push_back(*message);
      }
    });

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (retained_dispatches.empty() &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_EQ(retained_dispatches.size(), 1u);
  EXPECT_FALSE(retained_dispatches.front().has_sequence);
  EXPECT_DOUBLE_EQ(retained_dispatches.front().command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(retained_dispatches.front().command.angular.z, 0.0);

  late_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, ExternalLedgerFailureInvalidatesTransport)
{
  const std::string prefix = "/f_dwa_controller_invalidation_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state"),
      rclcpp::Parameter(
        "invalidate_trial_service_name", prefix + "/invalidate_trial")});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>(
    "command_delay_invalidation_client_test");
  const auto client =
    client_node->create_client<std_srvs::srv::Trigger>(
    prefix + "/invalidate_trial");
  bool transport_valid = true;
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/valid",
    rclcpp::QoS(1).reliable().transient_local(),
    [&transport_valid](const std_msgs::msg::Bool::SharedPtr message)
    {
      transport_valid = message && message->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (transport_valid &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(transport_valid);

  valid_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, NonFiniteCommandInvalidatesTransport)
{
  const std::string prefix = "/f_dwa_controller_nonfinite_command_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state")});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>(
    "command_delay_nonfinite_command_client_test");
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", rclcpp::QoS(1).reliable());
  bool transport_valid = true;
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/valid",
    rclcpp::QoS(1).reliable().transient_local(),
    [&transport_valid](const std_msgs::msg::Bool::SharedPtr message)
    {
      transport_valid = message && message->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist invalid_command;
  invalid_command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command_publisher->publish(invalid_command);
  const auto invalid_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (transport_valid &&
    std::chrono::steady_clock::now() < invalid_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(transport_valid);

  valid_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, InputBurstBelowConfiguredIntervalInvalidatesTransport)
{
  const std::string prefix = "/f_dwa_controller_input_interval_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state"),
      rclcpp::Parameter("minimum_input_interval_ms", 30.0)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>(
    "command_delay_input_interval_client_test");
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", rclcpp::QoS(2).reliable());
  bool transport_valid = true;
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/valid",
    rclcpp::QoS(1).reliable().transient_local(),
    [&transport_valid](const std_msgs::msg::Bool::SharedPtr message)
    {
      transport_valid = message && message->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.1;
  command_publisher->publish(command);
  command.linear.x = 0.2;
  command_publisher->publish(command);
  const auto invalid_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (transport_valid &&
    std::chrono::steady_clock::now() < invalid_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(transport_valid);

  valid_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, NormalTwentyTicksMeetPacingRateAndMinimumInterval)
{
  constexpr std::size_t kDispatchCount = 20u;
  constexpr double kPeriodSeconds = 0.03;
  const std::string prefix = "/f_dwa_controller_normal_pacing_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state"),
      rclcpp::Parameter("publish_frequency_hz", 1.0 / kPeriodSeconds),
      rclcpp::Parameter("min_delay_ms", 0.0),
      rclcpp::Parameter("max_delay_ms", 0.0),
      rclcpp::Parameter("mean_delay_ms", 0.0),
      rclcpp::Parameter("delay_stddev_ms", 0.0),
      rclcpp::Parameter("max_queue_depth", 24)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_normal_pacing_client_test");
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", rclcpp::QoS(24).reliable());
  std::vector<rclcpp::Time> dispatch_stamps;
  std::vector<rclcpp::Time> received_stamps;
  std::vector<uint64_t> received_steady_stamps;
  std::vector<uint64_t> dispatch_sequences;
  std::promise<void> dispatch_count_reached;
  auto dispatch_future = dispatch_count_reached.get_future();
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch",
    rclcpp::QoS(24).reliable().transient_local(),
    [&dispatch_stamps, &received_stamps, &received_steady_stamps, &dispatch_sequences,
    &dispatch_count_reached](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      if (message && message->has_sequence) {
        dispatch_stamps.emplace_back(message->header.stamp);
        received_stamps.emplace_back(message->received_at);
        received_steady_stamps.emplace_back(
          message->received_steady_time_ns);
        dispatch_sequences.emplace_back(message->sequence_id);
        if (dispatch_stamps.size() == kDispatchCount) {
          dispatch_count_reached.set_value();
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist command;
  for (std::size_t index = 0; index < kDispatchCount; ++index) {
    command.linear.x = 0.01 * static_cast<double>(index + 1u);
    command_publisher->publish(command);
  }

  ASSERT_EQ(
    executor.spin_until_future_complete(
      dispatch_future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(dispatch_stamps.size(), kDispatchCount);
  ASSERT_EQ(received_stamps.size(), kDispatchCount);
  ASSERT_EQ(received_steady_stamps.size(), kDispatchCount);
  ASSERT_EQ(dispatch_sequences.size(), kDispatchCount);

  const int64_t minimum_interval_nanoseconds =
    rclcpp::Duration::from_seconds(kPeriodSeconds).nanoseconds();
  int64_t observed_minimum_interval_nanoseconds =
    std::numeric_limits<int64_t>::max();
  for (std::size_t index = 1; index < kDispatchCount; ++index) {
    EXPECT_EQ(dispatch_sequences[index], index);
    const int64_t interval_nanoseconds =
      (dispatch_stamps[index] - dispatch_stamps[index - 1u]).nanoseconds();
    EXPECT_GE(
      interval_nanoseconds,
      minimum_interval_nanoseconds);
    if (interval_nanoseconds < observed_minimum_interval_nanoseconds) {
      observed_minimum_interval_nanoseconds = interval_nanoseconds;
    }
  }
  for (std::size_t index = 0; index < kDispatchCount; ++index) {
    EXPECT_LE(received_stamps[index], dispatch_stamps[index]);
    EXPECT_GT(received_steady_stamps[index], 0u);
    if (index > 0u) {
      EXPECT_LT(
        received_steady_stamps[index - 1u],
        received_steady_stamps[index]);
    }
  }
  EXPECT_EQ(dispatch_sequences.front(), 0u);
  const double elapsed_seconds =
    (dispatch_stamps.back() - dispatch_stamps.front()).seconds();
  ASSERT_GT(elapsed_seconds, 0.0);
  const double observed_rate_hz =
    static_cast<double>(kDispatchCount - 1u) / elapsed_seconds;
  RecordProperty("observed_rate_hz", observed_rate_hz);
  RecordProperty(
    "observed_minimum_interval_nanoseconds",
    observed_minimum_interval_nanoseconds);
  EXPECT_GE(observed_rate_hz, 32.0);

  dispatch_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, VelocityResponseFiltersGazeboButPreservesTargetLedger)
{
  constexpr double kResponseGain = 0.5;
  const std::string prefix = "/f_dwa_controller_response_transport_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter("publish_frequency_hz", 20.0),
      rclcpp::Parameter("min_delay_ms", 0.0),
      rclcpp::Parameter("max_delay_ms", 0.0),
      rclcpp::Parameter("mean_delay_ms", 0.0),
      rclcpp::Parameter("delay_stddev_ms", 0.0),
      rclcpp::Parameter("enable_velocity_response_model", true),
      rclcpp::Parameter(
        "linear_velocity_response_dead_time_seconds", 0.0),
      rclcpp::Parameter(
        "linear_velocity_response_time_constant_seconds", 0.2),
      rclcpp::Parameter("linear_velocity_response_gain", kResponseGain),
      rclcpp::Parameter(
        "angular_velocity_response_dead_time_seconds", 0.0),
      rclcpp::Parameter(
        "angular_velocity_response_time_constant_seconds", 0.2),
      rclcpp::Parameter("angular_velocity_response_gain", kResponseGain)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("velocity_response_transport_client_test");
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", 10);
  std::vector<geometry_msgs::msg::Twist> outputs;
  std::vector<geometry_msgs::msg::Twist> applied_targets;
  std::vector<f_dwa_controller::msg::CommandDispatch> dispatches;
  auto output_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    prefix + "/output", 10,
    [&outputs](const geometry_msgs::msg::Twist::SharedPtr message) {
      if (message) {
        outputs.push_back(*message);
      }
    });
  auto applied_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    prefix + "/applied", 10,
    [&applied_targets](const geometry_msgs::msg::Twist::SharedPtr message) {
      if (message) {
        applied_targets.push_back(*message);
      }
    });
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch", rclcpp::QoS(10).reliable().transient_local(),
    [&dispatches](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message) {
      if (message && message->has_sequence) {
        dispatches.push_back(*message);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist target;
  target.linear.x = 1.0;
  target.angular.z = 1.0;
  command_publisher->publish(target);
  const auto response_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((dispatches.empty() || outputs.empty() || applied_targets.empty() ||
    outputs.back().linear.x <= 0.0) &&
    std::chrono::steady_clock::now() < response_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_FALSE(dispatches.empty());
  ASSERT_FALSE(outputs.empty());
  ASSERT_FALSE(applied_targets.empty());
  EXPECT_DOUBLE_EQ(dispatches.back().command.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(dispatches.back().command.angular.z, 1.0);
  EXPECT_DOUBLE_EQ(applied_targets.back().linear.x, 1.0);
  EXPECT_DOUBLE_EQ(applied_targets.back().angular.z, 1.0);
  EXPECT_GT(outputs.back().linear.x, 0.0);
  EXPECT_LT(outputs.back().linear.x, kResponseGain);
  EXPECT_GT(outputs.back().angular.z, 0.0);
  EXPECT_LT(outputs.back().angular.z, kResponseGain);

  dispatch_subscriber.reset();
  applied_subscriber.reset();
  output_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, LateTimerCallbackReanchorsAtRobotHandoff)
{
  constexpr double kPeriodSeconds = 0.03;
  const std::string prefix = "/f_dwa_controller_pacing_test";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter(
        "reset_trial_service_name", prefix + "/reset_trial_state"),
      rclcpp::Parameter("publish_frequency_hz", 1.0 / kPeriodSeconds),
      rclcpp::Parameter("min_delay_ms", 0.0),
      rclcpp::Parameter("max_delay_ms", 0.0),
      rclcpp::Parameter("mean_delay_ms", 0.0),
      rclcpp::Parameter("delay_stddev_ms", 0.0),
      rclcpp::Parameter("max_queue_depth", 5)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_pacing_client_test");
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", rclcpp::QoS(6).reliable());
  std::vector<rclcpp::Time> dispatch_stamps;
  std::vector<uint64_t> dispatch_sequences;
  bool transport_valid = true;
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch",
    rclcpp::QoS(6).reliable().transient_local(),
    [&dispatch_stamps, &dispatch_sequences](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      if (message && message->has_sequence) {
        dispatch_stamps.emplace_back(message->header.stamp);
        dispatch_sequences.emplace_back(message->sequence_id);
      }
    });
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/valid",
    rclcpp::QoS(1).reliable().transient_local(),
    [&transport_valid](const std_msgs::msg::Bool::SharedPtr message)
    {
      transport_valid = message && message->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.1;
  const auto publish_and_enqueue = [&]() {
      command_publisher->publish(command);
      const auto enqueue_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(8);
      while (std::chrono::steady_clock::now() < enqueue_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
  const auto wait_for_dispatch_count =
    [&executor, &dispatch_stamps](const std::size_t expected) {
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (dispatch_stamps.size() < expected &&
        std::chrono::steady_clock::now() < deadline)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return dispatch_stamps.size() >= expected;
    };

  publish_and_enqueue();
  ASSERT_TRUE(wait_for_dispatch_count(1u));
  publish_and_enqueue();

  // Make the next 30 ms Timer callback execute late. The Timer must re-anchor
  // at that robot-facing handoff instead of immediately catching up.
  std::this_thread::sleep_for(std::chrono::milliseconds(44));
  ASSERT_TRUE(wait_for_dispatch_count(2u));
  publish_and_enqueue();
  const auto catch_up_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
  while (std::chrono::steady_clock::now() < catch_up_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(dispatch_stamps.size(), 2u);
  ASSERT_TRUE(wait_for_dispatch_count(3u));

  const rclcpp::Duration interval_after_late_callback =
    dispatch_stamps[2] - dispatch_stamps[1];
  EXPECT_GE(
    interval_after_late_callback.nanoseconds(),
    rclcpp::Duration::from_seconds(kPeriodSeconds).nanoseconds());
  // Re-anchoring at the handoff adds one normal period, not an additional
  // full missed phase.
  EXPECT_LT(
    interval_after_late_callback.nanoseconds(),
    rclcpp::Duration::from_seconds(0.075).nanoseconds());
  EXPECT_EQ(dispatch_sequences, (std::vector<uint64_t>{0u, 1u, 2u}));
  EXPECT_TRUE(transport_valid);

  valid_subscriber.reset();
  dispatch_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

TEST_F(CommandDelayNodeTest, EmergencyStopImmediatelyDiscardsDelayedCommands)
{
  const std::string prefix = "/f_dwa_controller_emergency_stop_test";
  const std::string service_name = prefix + "/emergency_stop";
  const std::string reset_service_name = prefix + "/reset";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("input_topic", prefix + "/input"),
      rclcpp::Parameter("output_topic", prefix + "/output"),
      rclcpp::Parameter("applied_topic", prefix + "/applied"),
      rclcpp::Parameter("dispatch_topic", prefix + "/dispatch"),
      rclcpp::Parameter("transport_valid_topic", prefix + "/valid"),
      rclcpp::Parameter("transport_stopped_topic", prefix + "/stopped"),
      rclcpp::Parameter("diagnostics_topic", prefix + "/diagnostics"),
      rclcpp::Parameter("emergency_stop_service_name", service_name),
      rclcpp::Parameter("reset_trial_service_name", reset_service_name),
      rclcpp::Parameter("publish_frequency_hz", 2.0)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_emergency_stop_client_test");
  const auto client =
    client_node->create_client<std_srvs::srv::Trigger>(service_name);
  const auto reset_client =
    client_node->create_client<std_srvs::srv::Trigger>(reset_service_name);
  const auto command_publisher =
    client_node->create_publisher<geometry_msgs::msg::Twist>(
    prefix + "/input", rclcpp::QoS(2).reliable());
  std::vector<geometry_msgs::msg::Twist> outputs;
  std::vector<geometry_msgs::msg::Twist> applied_commands;
  std::vector<f_dwa_controller::msg::CommandDispatch> dispatches;
  std::vector<bool> valid_states;
  std::vector<bool> stopped_states;
  auto output_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    prefix + "/output", rclcpp::QoS(2).reliable(),
    [&outputs](const geometry_msgs::msg::Twist::SharedPtr message) {
      outputs.push_back(*message);
    });
  auto applied_subscriber =
    client_node->create_subscription<geometry_msgs::msg::Twist>(
    prefix + "/applied", rclcpp::QoS(2).reliable(),
    [&applied_commands](const geometry_msgs::msg::Twist::SharedPtr message) {
      applied_commands.push_back(*message);
    });
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch", rclcpp::QoS(2).reliable().transient_local(),
    [&dispatches](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message) {
      dispatches.push_back(*message);
    });
  auto valid_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/valid", rclcpp::QoS(2).reliable().transient_local(),
    [&valid_states](const std_msgs::msg::Bool::SharedPtr message) {
      valid_states.push_back(message->data);
    });
  auto stopped_subscriber =
    client_node->create_subscription<std_msgs::msg::Bool>(
    prefix + "/stopped", rclcpp::QoS(2).reliable().transient_local(),
    [&stopped_states](const std_msgs::msg::Bool::SharedPtr message) {
      stopped_states.push_back(message->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(1)));
  const auto match_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (command_publisher->get_subscription_count() == 0u &&
    std::chrono::steady_clock::now() < match_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(command_publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist moving_command;
  moving_command.linear.x = 0.4;
  command_publisher->publish(moving_command);
  const auto enqueue_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
  while (std::chrono::steady_clock::now() < enqueue_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  outputs.clear();
  applied_commands.clear();
  dispatches.clear();
  valid_states.clear();
  stopped_states.clear();

  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("discarded 1"), std::string::npos);

  const auto message_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((outputs.empty() || applied_commands.empty() || dispatches.empty() ||
    valid_states.empty() || stopped_states.empty()) &&
    std::chrono::steady_clock::now() < message_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(outputs.empty());
  ASSERT_FALSE(applied_commands.empty());
  ASSERT_FALSE(dispatches.empty());
  ASSERT_FALSE(valid_states.empty());
  ASSERT_FALSE(stopped_states.empty());
  EXPECT_DOUBLE_EQ(outputs.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(applied_commands.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(dispatches.back().command.linear.x, 0.0);
  EXPECT_FALSE(dispatches.back().has_sequence);
  EXPECT_GT(dispatches.back().received_steady_time_ns, 0u);
  EXPECT_FALSE(valid_states.back());
  EXPECT_TRUE(stopped_states.back());

  outputs.clear();
  command_publisher->publish(moving_command);
  const auto blocked_command_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < blocked_command_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(outputs.empty());
  EXPECT_TRUE(
    std::all_of(
      outputs.begin(), outputs.end(),
      [](const geometry_msgs::msg::Twist & command) {
        return command.linear.x == 0.0 && command.angular.z == 0.0;
      }));

  // An explicit trial reset is the only recovery from the emergency latch.
  // It must publish a new zero/valid/stopped boundary before accepting a new
  // delayed command; restarting the transport process is not required.
  valid_states.clear();
  stopped_states.clear();
  auto reset_future = reset_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(reset_future, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto reset_response = reset_future.get();
  ASSERT_NE(reset_response, nullptr);
  EXPECT_TRUE(reset_response->success);
  const auto reset_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((valid_states.empty() || !valid_states.back() ||
    stopped_states.empty() || !stopped_states.back()) &&
    std::chrono::steady_clock::now() < reset_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(valid_states.empty());
  ASSERT_FALSE(stopped_states.empty());
  EXPECT_TRUE(valid_states.back());
  EXPECT_TRUE(stopped_states.back());

  outputs.clear();
  command_publisher->publish(moving_command);
  const auto resumed_command_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::none_of(
      outputs.begin(), outputs.end(),
      [](const geometry_msgs::msg::Twist & command) {
        return command.linear.x > 0.0;
      }) && std::chrono::steady_clock::now() < resumed_command_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(
    std::any_of(
      outputs.begin(), outputs.end(),
      [](const geometry_msgs::msg::Twist & command) {
        return command.linear.x > 0.0;
      }));

  stopped_subscriber.reset();
  valid_subscriber.reset();
  dispatch_subscriber.reset();
  applied_subscriber.reset();
  output_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

}  // namespace f_dwa_controller
