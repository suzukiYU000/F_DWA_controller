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
  std::vector<uint64_t> dispatch_sequences;
  std::promise<void> dispatch_count_reached;
  auto dispatch_future = dispatch_count_reached.get_future();
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    prefix + "/dispatch",
    rclcpp::QoS(24).reliable().transient_local(),
    [&dispatch_stamps, &dispatch_sequences, &dispatch_count_reached](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      if (message && message->has_sequence) {
        dispatch_stamps.emplace_back(message->header.stamp);
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

}  // namespace f_dwa_controller
