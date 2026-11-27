#include <gtest/gtest.h>

#include "interactive_cleanup/grasp_utils.hpp"

TEST(GraspUtilsTest, RequestsBaseCorrectionTowardCenteredFrontGrasp)
{
  const auto correction = interactive_cleanup::computePreGraspBaseCorrection(
    0.52, 0.06,
    0.40, 0.0,
    0.03, 0.02,
    0.05, 0.04);

  EXPECT_FALSE(correction.aligned);
  EXPECT_GT(correction.linear_x, 0.0);
  EXPECT_GT(correction.linear_y, 0.0);
}

TEST(GraspUtilsTest, MarksBaseAlignedInsidePreGraspWindow)
{
  const auto correction = interactive_cleanup::computePreGraspBaseCorrection(
    0.41, -0.01,
    0.40, 0.0,
    0.03, 0.02,
    0.05, 0.04);

  EXPECT_TRUE(correction.aligned);
  EXPECT_DOUBLE_EQ(correction.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(correction.linear_y, 0.0);
}

TEST(GraspUtilsTest, RaisesArmForTallerTargetDuringParameterizedGrasp)
{
  const auto low_plan = interactive_cleanup::computeParameterizedGraspArmPose(0.03);
  const auto high_plan = interactive_cleanup::computeParameterizedGraspArmPose(0.14);

  EXPECT_LT(low_plan[0], high_plan[0]);
  EXPECT_NEAR(low_plan[1], high_plan[1], 1e-6);
  EXPECT_NEAR(low_plan[3], high_plan[3], 1e-6);
}

TEST(GraspUtilsTest, DetectsObjectHeldWhenGripperCannotFullyClose)
{
  EXPECT_TRUE(interactive_cleanup::gripperLikelyHoldingObject(
    -0.05, -0.105, 0.02));
  EXPECT_FALSE(interactive_cleanup::gripperLikelyHoldingObject(
    -0.095, -0.105, 0.02));
}

TEST(GraspUtilsTest, RequiresEitherHoldingSignalOrTargetDisappearanceForSuccess)
{
  EXPECT_TRUE(interactive_cleanup::isGraspVerificationSuccessful(true, true));
  EXPECT_TRUE(interactive_cleanup::isGraspVerificationSuccessful(false, false));
  EXPECT_FALSE(interactive_cleanup::isGraspVerificationSuccessful(false, true));
}

TEST(GraspUtilsTest, AcceptsHandCameraEvidenceAsIndependentGraspConfirmation)
{
  EXPECT_TRUE(interactive_cleanup::isGraspVerificationSuccessful(
    false, true, true));
  EXPECT_FALSE(interactive_cleanup::isGraspVerificationSuccessful(
    false, true, false));
}
