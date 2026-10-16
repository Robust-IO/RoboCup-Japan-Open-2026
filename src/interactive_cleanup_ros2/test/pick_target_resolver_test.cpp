#include <gtest/gtest.h>

#include "interactive_cleanup/pick_target_resolver.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(PickTargetResolverTest, ClassifiesLowObjectsAsFloorPick)
{
  EXPECT_EQ("floor_pick", classifyGraspMode(0.03));
}

TEST(PickTargetResolverTest, PrefersPointingConsistentCandidate)
{
  PickCandidate aligned;
  aligned.class_name = "soysauce";
  aligned.confidence = 0.55;
  aligned.bbox_cx = 320.0;
  aligned.bbox_cy = 240.0;
  aligned.position.x = 1.0;
  aligned.position.y = 0.0;
  aligned.position.z = 0.04;
  aligned.has_3d_position = true;

  PickCandidate off_axis;
  off_axis.class_name = "soysauce";
  off_axis.confidence = 0.95;
  off_axis.bbox_cx = 500.0;
  off_axis.bbox_cy = 240.0;
  off_axis.position.x = 1.0;
  off_axis.position.y = 0.8;
  off_axis.position.z = 0.04;
  off_axis.has_3d_position = true;

  PointingObservation pointing;
  pointing.is_valid = true;
  pointing.confidence = 0.90;
  pointing.origin.z = 0.9;
  pointing.direction.x = 1.0;
  pointing.direction.y = 0.0;
  pointing.direction.z = 0.0;
  pointing.wrist_pixel_x = 320.0;
  pointing.wrist_pixel_y = 240.0;
  pointing.point_pixel_x = 420.0;
  pointing.point_pixel_y = 240.0;

  const auto result = resolvePickTarget({aligned, off_axis}, {pointing});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ("soysauce", result.class_name);
  EXPECT_NEAR(result.position.x, aligned.position.x, 1e-6);
  EXPECT_NEAR(result.position.y, aligned.position.y, 1e-6);
  EXPECT_EQ("floor_pick", result.grasp_mode);
}

TEST(PickTargetResolverTest, IgnoresLowConfidenceMisleadingPointingSamples)
{
  PickCandidate aligned;
  aligned.class_name = "soysauce";
  aligned.confidence = 0.55;
  aligned.bbox_cx = 320.0;
  aligned.bbox_cy = 240.0;
  aligned.position.x = 1.0;
  aligned.position.y = 0.0;
  aligned.position.z = 0.04;
  aligned.has_3d_position = true;

  PickCandidate off_axis;
  off_axis.class_name = "soysauce";
  off_axis.confidence = 0.95;
  off_axis.bbox_cx = 500.0;
  off_axis.bbox_cy = 240.0;
  off_axis.position.x = 1.0;
  off_axis.position.y = 0.8;
  off_axis.position.z = 0.04;
  off_axis.has_3d_position = true;

  PointingObservation high_conf_pointing;
  high_conf_pointing.is_valid = true;
  high_conf_pointing.confidence = 0.92;
  high_conf_pointing.origin.z = 0.9;
  high_conf_pointing.direction.x = 1.0;
  high_conf_pointing.direction.y = 0.0;
  high_conf_pointing.direction.z = 0.0;
  high_conf_pointing.wrist_pixel_x = 320.0;
  high_conf_pointing.wrist_pixel_y = 240.0;
  high_conf_pointing.point_pixel_x = 420.0;
  high_conf_pointing.point_pixel_y = 240.0;

  PointingObservation low_conf_outlier = high_conf_pointing;
  low_conf_outlier.confidence = 0.18;
  low_conf_outlier.direction.x = 0.78;
  low_conf_outlier.direction.y = 0.62;
  low_conf_outlier.wrist_pixel_x = 400.0;
  low_conf_outlier.point_pixel_x = 500.0;
  low_conf_outlier.point_pixel_y = 320.0;

  const auto result = resolvePickTarget(
    {aligned, off_axis},
    {high_conf_pointing, low_conf_outlier});

  ASSERT_TRUE(result.valid);
  EXPECT_EQ("soysauce", result.class_name);
  EXPECT_NEAR(result.position.x, aligned.position.x, 1e-6);
  EXPECT_NEAR(result.position.y, aligned.position.y, 1e-6);
}

}  // namespace
}  // namespace interactive_cleanup
