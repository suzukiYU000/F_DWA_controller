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

#ifndef F_DWA_CONTROLLER__COMMAND_DELAY_QUEUE_HPP_
#define F_DWA_CONTROLLER__COMMAND_DELAY_QUEUE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <random>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"

namespace f_dwa_controller
{

struct DelayedCommand
{
  geometry_msgs::msg::Twist command;
  rclcpp::Time received_at;
  uint64_t received_steady_time_ns;
  rclcpp::Time eligible_at;
  uint64_t sequence;
  double sampled_delay_ms;
};

struct CommandDelayParameters
{
  double min_delay_ms{60.0};
  double max_delay_ms{80.0};
  double mean_delay_ms{70.0};
  double delay_stddev_ms{3.333333333333333};
  double zero_threshold{0.0};
  // Three bounded-delay entries, one normal callback-order entry, and one
  // delayed-timer scheduling slot. A further backlog is transport_invalid.
  std::size_t max_queue_depth{24};
  uint64_t random_seed{0};
};

class CommandDelayQueue
{
public:
  explicit CommandDelayQueue(const CommandDelayParameters & parameters);

  [[nodiscard]] bool enqueue(
    const geometry_msgs::msg::Twist & command,
    const rclcpp::Time & received_at,
    uint64_t received_steady_time_ns);

  [[nodiscard]] std::optional<DelayedCommand> pop_due(const rclcpp::Time & now);
  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] bool empty() const;
  [[nodiscard]] const DelayedCommand * front() const;
  [[nodiscard]] std::vector<DelayedCommand> snapshot() const;
  [[nodiscard]] uint64_t next_sequence() const;

  void clear();
  void reset(uint64_t random_seed);

  [[nodiscard]] static geometry_msgs::msg::Twist normalized_command(
    const geometry_msgs::msg::Twist & command,
    double zero_threshold);
  [[nodiscard]] static bool is_zero(
    const geometry_msgs::msg::Twist & command,
    double zero_threshold);

private:
  [[nodiscard]] double sample_delay_ms();

  CommandDelayParameters parameters_;
  std::deque<DelayedCommand> queue_;
  std::mt19937_64 random_engine_;
  std::normal_distribution<double> delay_distribution_;
  uint64_t next_sequence_{0};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__COMMAND_DELAY_QUEUE_HPP_
