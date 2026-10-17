#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "interactive_cleanup/place_destination_resolver.hpp"

namespace interactive_cleanup
{
namespace
{

PointingObservation makePointing(
  double ox, double oy, double oz,
  double dx, double dy, double dz,
  double confidence = 0.90)
{
  PointingObservation pointing;
  pointing.is_valid = true;
  pointing.confidence = confidence;
  pointing.origin.x = ox;
  pointing.origin.y = oy;
  pointing.origin.z = oz;
  pointing.direction.x = dx;
  pointing.direction.y = dy;
  pointing.direction.z = dz;
  return pointing;
}

TEST(PlaceDestinationResolverTest, InfersLegacyRegionMetadataFromDestinationName)
{
  const auto region = inferDestinationRegion("white_side_table_2", -0.74, 1.416);

  EXPECT_EQ("table", region.type);
  EXPECT_NEAR(region.center.x, -0.74, 1e-6);
  EXPECT_NEAR(region.center.y, 1.416, 1e-6);
  EXPECT_NEAR(region.allowed_radius, 0.50, 1e-6);
  EXPECT_EQ("surface_place", region.placement_mode);
}

TEST(PlaceDestinationResolverTest, LoadsSeededDestinationRegionsFromConfig)
{
  const auto config_path = std::filesystem::path(__FILE__).parent_path().parent_path() /
    "config" / "destination_regions.yaml";

  std::string error;
  const auto regions = loadDestinationRegions(config_path.string(), &error);

  ASSERT_TRUE(error.empty()) << error;
  ASSERT_EQ(10u, regions.size());

  const auto it = std::find_if(
    regions.begin(), regions.end(),
    [](const DestinationRegion &region) {
      return region.name == "white_side_table_2";
    });

  ASSERT_NE(it, regions.end());
  EXPECT_EQ("table", it->type);
  EXPECT_EQ("surface_place", it->placement_mode);
}

TEST(PlaceDestinationResolverTest, PrefersPointingAlignedRegionOverUnalignedRegion)
{
  const DestinationRegion aligned = inferDestinationRegion("white_side_table_1", 1.5, 0.1);
  const DestinationRegion off_axis = inferDestinationRegion("wagon_1", 0.6, 1.2);

  PointingObservation pointing_a;
  pointing_a.is_valid = true;
  pointing_a.confidence = 0.90;
  pointing_a.origin.x = 0.0;
  pointing_a.origin.y = 0.0;
  pointing_a.direction.x = 1.0;
  pointing_a.direction.y = 0.0;

  PointingObservation pointing_b = pointing_a;
  pointing_b.direction.x = 0.98;
  pointing_b.direction.y = 0.05;
  PointingObservation pointing_c = pointing_a;
  pointing_c.direction.x = 0.99;
  pointing_c.direction.y = -0.04;

  const auto result = resolvePlaceDestination(
    {aligned, off_axis}, {pointing_a, pointing_b, pointing_c}, "soysauce");

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(aligned.name, result.name);
  EXPECT_NEAR(aligned.center.x, result.position.x, 1e-6);
  EXPECT_NEAR(aligned.center.y, result.position.y, 1e-6);
  EXPECT_EQ("surface_place", result.placement_mode);
}

TEST(PlaceDestinationResolverTest, ReturnsInvalidWhenNoUsablePointingSamplesExist)
{
  const auto result = resolvePlaceDestination(
    {inferDestinationRegion("white_side_table_1", 1.1, -2.28)},
    {},
    "soysauce");

  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, IgnoresLowConfidenceOffAxisPointingOutliers)
{
  const DestinationRegion aligned = inferDestinationRegion("white_side_table_1", 1.5, 0.1);
  const DestinationRegion off_axis = inferDestinationRegion("wagon_1", 0.6, 1.2);

  PointingObservation high_conf_a;
  high_conf_a.is_valid = true;
  high_conf_a.confidence = 0.91;
  high_conf_a.origin.x = 0.0;
  high_conf_a.origin.y = 0.0;
  high_conf_a.direction.x = 1.0;
  high_conf_a.direction.y = 0.0;

  PointingObservation high_conf_b = high_conf_a;
  high_conf_b.confidence = 0.87;
  high_conf_b.direction.x = 0.998;
  high_conf_b.direction.y = 0.05;
  PointingObservation high_conf_c = high_conf_a;
  high_conf_c.confidence = 0.89;
  high_conf_c.direction.x = 0.999;
  high_conf_c.direction.y = -0.03;

  PointingObservation low_conf_outlier = high_conf_a;
  low_conf_outlier.confidence = 0.15;
  low_conf_outlier.direction.x = 0.31;
  low_conf_outlier.direction.y = 0.95;

  const auto result = resolvePlaceDestination(
    {aligned, off_axis},
    {high_conf_a, high_conf_b, high_conf_c, low_conf_outlier},
    "soysauce");

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(aligned.name, result.name);
}

TEST(PlaceDestinationResolverTest, AcceptsOneClearPointingSample)
{
  const auto destination = inferDestinationRegion("white_side_table_1", 2.0, 0.0);
  const auto ray = makePointing(0.0, 0.0, 1.2, 1.0, 0.0, -0.3);

  const auto result = resolvePlaceDestination(
    {destination}, {ray}, "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(destination.name, result.name);
}

TEST(PlaceDestinationResolverTest, SurfaceIntersectionSeparatesCollinearRegions)
{
  DestinationRegion near_table = inferDestinationRegion(
    "white_side_table_1", 1.2, 0.0);
  near_table.center.z = 0.66;
  DestinationRegion far_box = inferDestinationRegion(
    "cardboard_box", 3.2, 0.0);
  far_box.center.z = 0.32;

  // Both destinations have zero planar-ray error.  The downward arm
  // elevation lands on the table height at x=1.2, so the infinite 2-D ray
  // must not incorrectly select the farther box.
  const auto ray = makePointing(0.0, 0.0, 1.20, 1.2, 0.0, -0.54);
  const auto result = resolvePlaceDestination(
    {near_table, far_box}, {ray}, "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(near_table.name, result.name);
  EXPECT_LT(result.surface_intersection_distance, 0.05);
}

TEST(PlaceDestinationResolverTest, ImageGestureDisambiguatesCollinearDepthError)
{
  DestinationRegion intended = inferDestinationRegion(
    "white_side_table_1", 1.2, 0.0);
  intended.center.z = 0.66;
  intended.image_projection_valid = true;
  intended.image_ray_distance_px = 8.0;
  intended.image_supporting_samples = 10;

  DestinationRegion depth_decoy = inferDestinationRegion(
    "cardboard_box", 3.2, 0.0);
  depth_decoy.center.z = 0.32;
  depth_decoy.image_projection_valid = true;
  depth_decoy.image_ray_distance_px = 90.0;
  depth_decoy.image_supporting_samples = 10;

  // Simulate contaminated landmark depth: the world ray is biased toward the
  // farther cardboard box, while the visible arm line in the held camera
  // view passes through the intended side table.
  const auto noisy_depth_ray = makePointing(
    0.0, 0.0, 1.20, 3.2, 0.0, -0.88);
  const auto result = resolvePlaceDestination(
    {intended, depth_decoy},
    {noisy_depth_ray, noisy_depth_ray, noisy_depth_ray},
    "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(intended.name, result.name);
  EXPECT_NEAR(8.0, result.image_ray_distance_px, 1e-6);
  EXPECT_TRUE(result.image_evidence_used);
}

TEST(PlaceDestinationResolverTest, IgnoresImageLineFarFromEveryDestination)
{
  DestinationRegion world_aligned = inferDestinationRegion(
    "white_side_table_1", 2.0, 0.0);
  world_aligned.center.z = 0.66;
  world_aligned.image_projection_valid = true;
  world_aligned.image_ray_distance_px = 250.0;

  DestinationRegion image_decoy = inferDestinationRegion(
    "white_side_table_2", 0.0, 2.0);
  image_decoy.center.z = 0.66;
  image_decoy.image_projection_valid = true;
  image_decoy.image_ray_distance_px = 110.0;

  const auto pose_world_ray = makePointing(
    0.0, 0.0, 1.20, 2.0, 0.0, -0.54);
  const auto result = resolvePlaceDestination(
    {world_aligned, image_decoy},
    {pose_world_ray, pose_world_ray, pose_world_ray},
    "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(world_aligned.name, result.name);
  EXPECT_FALSE(result.image_evidence_used);
}

TEST(PlaceDestinationResolverTest, UsesTriggerHeightInsteadOfGroundProjection)
{
  DestinationRegion intended = inferDestinationRegion("white_side_table_1", 3.0, 0.0);
  intended.center.z = 0.60;
  DestinationRegion floor_decoy = inferDestinationRegion("cardboard_box", 3.0, 0.0);
  floor_decoy.center.z = 0.10;

  const auto ray_a = makePointing(0.0, 0.0, 1.20, 3.0, 0.0, -0.60);
  const auto ray_b = makePointing(0.0, 0.02, 1.21, 3.0, -0.02, -0.61);
  const auto ray_c = makePointing(0.0, -0.02, 1.19, 3.0, 0.02, -0.59);

  const auto result = resolvePlaceDestination(
    {intended, floor_decoy}, {ray_a, ray_b, ray_c}, "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(intended.name, result.name);
  EXPECT_EQ(3, result.supporting_samples);
  EXPECT_LT(result.ray_distance, 0.05);
}

TEST(PlaceDestinationResolverTest, KeepsHorizontalDestinationWhenElevationIsNoisy)
{
  DestinationRegion intended = inferDestinationRegion("white_side_table_1", 3.0, 0.10);
  intended.center.z = 0.66;
  DestinationRegion off_axis = inferDestinationRegion("cardboard_box", 2.5, 1.20);
  off_axis.center.z = 0.32;

  // The arm bearing is consistently toward +X, but the elevation points too
  // steeply down and would miss every trigger in a strict 3-D-only gate.
  const auto ray_a = makePointing(0.0, 0.0, 1.20, 1.0, 0.02, -0.85);
  const auto ray_b = makePointing(0.0, 0.02, 1.22, 1.0, 0.00, -0.75);
  const auto ray_c = makePointing(0.0, -0.02, 1.18, 1.0, 0.04, -0.90);

  const auto result = resolvePlaceDestination(
    {intended, off_axis}, {ray_a, ray_b, ray_c}, "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(intended.name, result.name);
  EXPECT_LT(result.ray_distance, 0.20);
  EXPECT_GT(result.three_d_ray_distance, result.ray_distance);
}

TEST(PlaceDestinationResolverTest, RejectsGeometricallyAmbiguousRegions)
{
  DestinationRegion upper = inferDestinationRegion("white_side_table_1", 3.0, 0.05);
  upper.center.z = 0.60;
  DestinationRegion lower = inferDestinationRegion("white_side_table_2", 3.0, -0.05);
  lower.center.z = 0.60;
  const auto ray = makePointing(0.0, 0.0, 1.20, 3.0, 0.0, -0.60);

  const auto result = resolvePlaceDestination(
    {upper, lower}, {ray, ray, ray}, "bear_doll");

  EXPECT_FALSE(result.valid);
  EXPECT_NE(std::string::npos, result.rejection_reason.find("ambiguous"));
}

TEST(PlaceDestinationResolverTest, ResolvesDepthPolarityOnceForWholeGesture)
{
  DestinationRegion destination = inferDestinationRegion("square_low_table", 2.0, 0.0);
  destination.center.z = 0.60;
  const auto reversed_ray = makePointing(0.0, 0.0, 1.20, -2.0, 0.0, 0.60);

  const auto result = resolvePlaceDestination(
    {destination}, {reversed_ray, reversed_ray, reversed_ray}, "spray_bottle");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(destination.name, result.name);
  EXPECT_TRUE(result.reversed);
}

}  // namespace
}  // namespace interactive_cleanup
