#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "interactive_cleanup/pointing_alignment.hpp"

namespace
{

interactive_cleanup::PointingObservation makePointing(
  double yaw,
  double confidence = 1.0)
{
  interactive_cleanup::PointingObservation observation;
  observation.is_valid = true;
  observation.confidence = confidence;
  observation.direction.x = std::cos(yaw);
  observation.direction.y = std::sin(yaw);
  observation.direction.z = 0.0;
  return observation;
}

}  // namespace

TEST(PointingAlignmentTest, UsesAggregatedYawAndKeepsTurningForLargeErrors)
{
  const std::vector<interactive_cleanup::PointingObservation> pointings = {
    makePointing(-2.10),
    makePointing(-2.20),
    makePointing(-2.15),
  };

  const auto result = interactive_cleanup::evaluatePointingAlignment(
    pointings,
    0.0,
    0.18,
    0.20,
    1.6,
    0.45,
    3);

  EXPECT_TRUE(result.has_measurement);
  EXPECT_FALSE(result.aligned);
  EXPECT_EQ(result.sample_count, 3u);
  EXPECT_NEAR(result.desired_yaw, -2.15, 0.08);
  EXPECT_LT(result.yaw_error, -2.0);
  EXPECT_NEAR(result.command_angular, -0.45, 1e-6);
}

TEST(PointingAlignmentTest, DoesNotDeclareAlignedOnSingleFrameNoise)
{
  const std::vector<interactive_cleanup::PointingObservation> pointings = {
    makePointing(-0.12),
  };

  const auto result = interactive_cleanup::evaluatePointingAlignment(
    pointings,
    -0.20,
    0.18,
    0.20,
    1.6,
    0.45,
    3);

  EXPECT_TRUE(result.has_measurement);
  EXPECT_FALSE(result.aligned);
  EXPECT_EQ(result.sample_count, 1u);
  EXPECT_TRUE(result.within_angle_threshold);
  EXPECT_DOUBLE_EQ(result.command_angular, 0.0);
}

TEST(PointingAlignmentTest, RejectsUnstablePointingCluster)
{
  const std::vector<interactive_cleanup::PointingObservation> pointings = {
    makePointing(-0.10),
    makePointing(0.85),
    makePointing(-0.15),
  };

  const auto result = interactive_cleanup::evaluatePointingAlignment(
    pointings,
    -0.12,
    0.18,
    0.20,
    1.6,
    0.45,
    3);

  EXPECT_TRUE(result.has_measurement);
  EXPECT_FALSE(result.aligned);
  EXPECT_EQ(result.sample_count, 3u);
  EXPECT_GT(result.angular_stability, 0.20);
}

TEST(PointingAlignmentTest, IgnoresLowConfidenceOutliersWhenAggregatingYaw)
{
  const std::vector<interactive_cleanup::PointingObservation> pointings = {
    makePointing(0.05, 0.92),
    makePointing(0.00, 0.88),
    makePointing(1.60, 0.20),
  };

  const auto result = interactive_cleanup::evaluatePointingAlignment(
    pointings,
    0.02,
    0.18,
    0.20,
    1.6,
    0.45,
    2);

  EXPECT_TRUE(result.has_measurement);
  EXPECT_TRUE(result.aligned);
  EXPECT_EQ(result.sample_count, 2u);
  EXPECT_NEAR(result.desired_yaw, 0.025, 0.08);
  EXPECT_LT(result.angular_stability, 0.10);
}

TEST(PointingAlignmentTest, ComputesAdaptiveTimeoutFromInitialYawError)
{
  const double timeout_sec = interactive_cleanup::computePointingAlignmentTimeout(
    -2.15,
    0.45,
    2.0,
    1.0,
    8.0);

  EXPECT_GT(timeout_sec, 5.5);
  EXPECT_LT(timeout_sec, 6.0);
}
