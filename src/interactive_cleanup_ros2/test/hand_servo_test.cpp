#include <gtest/gtest.h>

#include "interactive_cleanup/hand_servo.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(HandServoTest, CommandsConservativeCorrectionsTowardCenteredTarget)
{
  HandServoInput input;
  input.target_found = true;
  input.pixel_error_x = 90.0;
  input.pixel_error_y = 50.0;
  input.bbox_area_ratio = 0.05;
  input.confidence = 0.9;

  const auto command = computeHandServoCommand(input, 0.05, 0.04, 0.03);

  EXPECT_FALSE(command.aligned);
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_LT(command.linear_y, 0.0);
  EXPECT_LT(command.lift_delta, 0.0);
}

TEST(HandServoTest, MarksAlignedWhenTargetIsInsideGraspWindow)
{
  HandServoInput input;
  input.target_found = true;
  input.in_grasp_window = true;
  input.confidence = 0.8;

  const auto command = computeHandServoCommand(input, 0.05, 0.04, 0.03);

  EXPECT_TRUE(command.aligned);
  EXPECT_TRUE(command.should_close);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command.lift_delta, 0.0);
}

TEST(HandServoTest, RequestsRetryOnlyAfterBoundedTargetLoss)
{
  EXPECT_FALSE(shouldRetryHandApproach(false, false, 3, 6, 2.0, 8.0));
  EXPECT_TRUE(shouldRetryHandApproach(false, false, 7, 6, 2.0, 8.0));
  EXPECT_TRUE(shouldRetryHandApproach(false, true, 0, 6, 9.0, 8.0));
}

}  // namespace
}  // namespace interactive_cleanup
