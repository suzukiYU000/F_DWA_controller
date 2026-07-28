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
#include <memory>
#include <string>
#include <vector>

#include "f_dwa_controller/command_delay_node.hpp"
#include "f_dwa_controller/msg/command_dispatch.hpp"
#include "rclcpp/rclcpp.hpp"
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

TEST_F(CommandDelayNodeTest, TrialResetServiceRespondsOnIsolatedTopics)
{
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
        "diagnostics_topic", "/f_dwa_controller_test/diagnostics"),
      rclcpp::Parameter("reset_trial_service_name", service_name),
      rclcpp::Parameter("random_seed", 42)});

  const auto transport = std::make_shared<CommandDelayNode>(options);
  const auto client_node =
    std::make_shared<rclcpp::Node>("command_delay_reset_client_test");
  const auto client =
    client_node->create_client<std_srvs::srv::Trigger>(service_name);
  bool received_dispatch = false;
  auto dispatch_subscriber =
    client_node->create_subscription<f_dwa_controller::msg::CommandDispatch>(
    "/f_dwa_controller_test/dispatch",
    rclcpp::QoS(1).reliable().transient_local(),
    [&received_dispatch](
      const f_dwa_controller::msg::CommandDispatch::SharedPtr message)
    {
      received_dispatch =
      message &&
      message->command.linear.x == 0.0 &&
      message->command.angular.z == 0.0 &&
      !message->has_sequence &&
      message->header.stamp.sec != 0;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transport);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

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
  executor.spin_some();
  EXPECT_TRUE(received_dispatch);

  dispatch_subscriber.reset();
  executor.remove_node(client_node);
  executor.remove_node(transport);
}

}  // namespace f_dwa_controller
