#include <gtest/gtest.h>

#include "interactive_cleanup/avatar_tracker.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(AvatarTrackerTest, ReturnsNoCommandInsideDeadband)
{
  const auto cmd = computeAvatarTrackCommand(0.03, 0.05, 1.5, 0.4);
  EXPECT_FALSE(cmd.tracking);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST(AvatarTrackerTest, TurnsTowardAvatarAndClampsMagnitude)
{
  const auto cmd = computeAvatarTrackCommand(0.30, 0.05, 1.5, 0.4);
  EXPECT_TRUE(cmd.tracking);
  EXPECT_DOUBLE_EQ(cmd.angular_z, -0.4);
}

}  // namespace
}  // namespace interactive_cleanup
