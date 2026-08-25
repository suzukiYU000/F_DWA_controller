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

#include <cmath>
#include <limits>

#include "f_dwa_controller/velocity_response_model.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr AxisVelocityResponseModel kLinearModel{0.035, 0.02, 1.0};
constexpr AxisVelocityResponseModel kAngularModel{0.015, 0.085, 0.95};

}  // namespace

TEST(VelocityResponseModel, HoldsMeasuredVelocityUntilDeadTimeExpires)
{
  EXPECT_DOUBLE_EQ(
    predict_axis_velocity(0.0, 0.6, 0.01, 0.02, kLinearModel),
    0.0);
  EXPECT_NEAR(
    predict_axis_velocity(0.0, 0.6, 0.01, 0.12, kLinearModel),
    0.6 * (1.0 - std::exp(-0.095 / 0.02)), 1.0e-12);
}

TEST(VelocityResponseModel, DispatchAgeConsumesOnlyTheRemainingDeadTime)
{
  const double predicted = predict_axis_velocity(
    0.0, 0.6, 0.05, 0.02, kLinearModel);
  EXPECT_NEAR(predicted, 0.6 * (1.0 - std::exp(-1.0)), 1.0e-12);
}

TEST(VelocityResponseModel, DelayedOdomDoesNotReanchorCommandRiseAtZero)
{
  const double predicted = predict_axis_velocity(
    0.0, 0.20, 0.05, 0.12, kLinearModel);
  EXPECT_GT(predicted, 0.19);
  EXPECT_LT(predicted, 0.20);
}

TEST(VelocityResponseModel, DecelerationStartsFromMeasuredPhysicalVelocity)
{
  const double predicted = predict_axis_velocity(
    0.6, 0.0, 0.05, 0.02, kLinearModel);
  EXPECT_GT(predicted, 0.0);
  EXPECT_LT(predicted, 0.6);
  EXPECT_NEAR(predicted, 0.6 * std::exp(-1.0), 1.0e-12);
}

TEST(VelocityResponseModel, PredictsPoseAndVelocityWithOneCommonPlantModel)
{
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D odometry;
  nav_2d_msgs::msg::Twist2D command;
  command.x = 0.6;
  command.theta = 0.4;

  const VelocityResponsePrediction prediction = predict_velocity_response(
    pose, odometry, command, 0.05, 0.12, 0.01,
    kLinearModel, kAngularModel);

  ASSERT_TRUE(prediction.valid);
  ASSERT_EQ(prediction.trajectory.size(), 13u);
  EXPECT_GT(prediction.pose.x, 0.0);
  EXPECT_GT(prediction.pose.y, 0.0);
  EXPECT_GT(prediction.pose.theta, 0.0);
  EXPECT_GT(prediction.velocity.x, 0.59);
  EXPECT_GT(prediction.velocity.theta, 0.25);
  EXPECT_LT(prediction.velocity.theta, 0.38);
}

TEST(VelocityResponseModel, RejectsInvalidIdentifiedParameters)
{
  AxisVelocityResponseModel invalid = kLinearModel;
  invalid.time_constant_seconds = 0.0;
  EXPECT_FALSE(valid_velocity_response_model(invalid));
  EXPECT_TRUE(std::isnan(
      predict_axis_velocity(0.0, 0.6, 0.0, 0.12, invalid)));

  invalid = kLinearModel;
  invalid.dead_time_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_velocity_response_model(invalid));
}

}  // namespace f_dwa_controller
