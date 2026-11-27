#include <gtest/gtest.h>

#include "interactive_cleanup/observation_head_sweep.hpp"

TEST(ObservationHeadSweepTest, BuildsSymmetricSweepAroundCenter)
{
  const auto plan = interactive_cleanup::buildObservationHeadSweepPlan(
    -0.14, 0.18, 0.35, 0.40);

  ASSERT_EQ(plan.phases.size(), 5u);
  EXPECT_DOUBLE_EQ(plan.phases[0].pan, 0.0);
  EXPECT_DOUBLE_EQ(plan.phases[1].pan, 0.18);
  EXPECT_DOUBLE_EQ(plan.phases[2].pan, 0.0);
  EXPECT_DOUBLE_EQ(plan.phases[3].pan, -0.18);
  EXPECT_DOUBLE_EQ(plan.phases[4].pan, 0.0);
  EXPECT_DOUBLE_EQ(plan.phases[0].tilt, -0.14);
  EXPECT_GT(plan.total_duration_sec, 3.0);
}

TEST(ObservationHeadSweepTest, CollectsOnlyAfterHeadMotionSettles)
{
  const auto plan = interactive_cleanup::buildObservationHeadSweepPlan(
    -0.12, 0.16, 0.30, 0.40);

  const auto first_motion = interactive_cleanup::sampleObservationHeadSweep(plan, 0.10);
  EXPECT_EQ(first_motion.phase_index, 0u);
  EXPECT_FALSE(first_motion.collecting);
  EXPECT_FALSE(first_motion.complete);

  const auto first_hold = interactive_cleanup::sampleObservationHeadSweep(plan, 0.45);
  EXPECT_EQ(first_hold.phase_index, 0u);
  EXPECT_TRUE(first_hold.collecting);

  const auto second_motion = interactive_cleanup::sampleObservationHeadSweep(plan, 0.75);
  EXPECT_EQ(second_motion.phase_index, 1u);
  EXPECT_FALSE(second_motion.collecting);

  const auto second_hold = interactive_cleanup::sampleObservationHeadSweep(plan, 1.05);
  EXPECT_EQ(second_hold.phase_index, 1u);
  EXPECT_TRUE(second_hold.collecting);
}

TEST(ObservationHeadSweepTest, CompletesOnFinalCenteredObservation)
{
  const auto plan = interactive_cleanup::buildObservationHeadSweepPlan(
    -0.10, 0.12, 0.25, 0.35);

  const auto sample = interactive_cleanup::sampleObservationHeadSweep(
    plan, plan.total_duration_sec + 0.01);

  EXPECT_TRUE(sample.complete);
  EXPECT_EQ(sample.phase_index, 4u);
  EXPECT_DOUBLE_EQ(sample.pan, 0.0);
  EXPECT_DOUBLE_EQ(sample.tilt, -0.10);
  EXPECT_FALSE(sample.collecting);
}
