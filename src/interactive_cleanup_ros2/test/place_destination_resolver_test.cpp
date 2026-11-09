#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "interactive_cleanup/place_destination_resolver.hpp"
#include "interactive_cleanup/pointing_geometry.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(GestureFramingTest, RecordedClippedHandMovesViewTowardHand)
{
  // Actual final destination frame: torso x=133.49, arm start=71.07,
  // inferred fingertip=-7.86. The view must include the hand as well.
  EXPECT_NEAR(gestureFramingCenter(133.49, 71.07, -7.86), 62.815, 1e-6);
  EXPECT_DOUBLE_EQ(gestureFramingCenter(320, 350, 680), 500);
  EXPECT_DOUBLE_EQ(gestureFramingCenter(320, 320, 320), 320);
  // Once reframed, the same configuration stays centred (no edge-toggle).
  EXPECT_NEAR(gestureFramingCenter(390.675, 328.255, 249.325), 320, 1e-6);
}

PointingObservation makePointing(
  double ox, double oy, double oz,
  double dx, double dy, double dz,
  double confidence = 0.90,
  bool measured_depth = false)
{
  PointingObservation pointing;
  pointing.is_valid = true;
  pointing.measured_depth = measured_depth;
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
  auto destination = inferDestinationRegion("white_side_table_1", 2.0, 0.0);
  destination.center.z = 0.60;
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

TEST(PlaceDestinationResolverTest, LegacyPixelRankingCannotOverrideMeasuredGeometry)
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
    0.0, 0.0, 1.20, 3.2, 0.0, -0.88, 0.90, true);
  const auto result = resolvePlaceDestination(
    {intended, depth_decoy},
    {noisy_depth_ray, noisy_depth_ray, noisy_depth_ray},
    "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(depth_decoy.name, result.name);
}

TEST(PlaceDestinationResolverTest, LoggedWhiteTableImageCannotSelectOffImageWagon)
{
  DestinationRegion table = inferDestinationRegion(
    "white_side_table_2", 4.92, -1.26);
  table.center.z = 0.66;
  table.image_projection_valid = true;
  table.image_ray_distance_px = 105.7;
  table.image_supporting_samples = 10;

  DestinationRegion wagon = inferDestinationRegion("wagon_2", 4.31, 3.38);
  wagon.center.z = 0.88;
  wagon.image_projection_valid = true;
  wagon.image_ray_distance_px = 9999.0;
  wagon.image_supporting_samples = 10;

  // Reproduce the 23:29 failure: internally consistent depth pointed toward
  // the wagon while the visible arm line uniquely pointed at the side table.
  const auto contaminated_depth = makePointing(
    0.0, 0.0, 1.10, 4.31, 3.38, -0.22, 0.96, true);
  const auto result = resolvePlaceDestination(
    {table, wagon},
    {contaminated_depth, contaminated_depth, contaminated_depth},
    "bear_doll");

  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, UsesUniqueImageRayWhenOnlyPoseWorldFallbackExists)
{
  DestinationRegion intended = inferDestinationRegion(
    "white_side_table_1", 1.2, 0.0);
  intended.center.z = 0.66;
  intended.image_projection_valid = true;
  intended.image_ray_distance_px = 12.0;
  intended.image_supporting_samples = 2;

  DestinationRegion pose_decoy = inferDestinationRegion(
    "cardboard_box", 3.2, 0.0);
  pose_decoy.center.z = 0.32;
  pose_decoy.image_projection_valid = true;
  pose_decoy.image_ray_distance_px = 145.0;
  pose_decoy.image_supporting_samples = 2;

  const auto pose_fallback = makePointing(
    0.0, 0.0, 1.20, 3.2, 0.0, -0.88);
  const auto result = resolvePlaceDestination(
    {intended, pose_decoy}, {pose_fallback, pose_fallback}, "bear_doll");

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(intended.name, result.name);
  EXPECT_TRUE(result.image_evidence_used);
  EXPECT_EQ(2, result.supporting_samples);
}

TEST(PlaceDestinationResolverTest, RejectsAmbiguousImageOnlyFallback)
{
  DestinationRegion first = inferDestinationRegion(
    "white_side_table_1", 1.2, 0.0);
  first.image_projection_valid = true;
  first.image_ray_distance_px = 20.0;
  first.image_supporting_samples = 2;
  DestinationRegion second = inferDestinationRegion(
    "white_side_table_2", 1.4, 0.1);
  second.image_projection_valid = true;
  second.image_ray_distance_px = 35.0;
  second.image_supporting_samples = 2;
  const auto pose_fallback = makePointing(
    0.0, 0.0, 1.2, 1.0, 0.0, -0.2);

  const auto result = resolvePlaceDestination(
    {first, second}, {pose_fallback}, "bear_doll");

  EXPECT_FALSE(result.valid);
  EXPECT_NE(std::string::npos, result.rejection_reason.find("ambiguous"));
}

TEST(PlaceDestinationResolverTest, SingleImageFrameCannotOverrideMeasuredDepth)
{
  DestinationRegion intended = inferDestinationRegion(
    "white_side_table_2", 1.2, 0.0);
  intended.center.z = 0.66;
  intended.image_projection_valid = true;
  intended.image_ray_distance_px = 105.0;
  intended.image_supporting_samples = 1;

  DestinationRegion depth_decoy = inferDestinationRegion(
    "cardboard_box", 3.2, 0.0);
  depth_decoy.center.z = 0.32;
  depth_decoy.image_projection_valid = true;
  depth_decoy.image_ray_distance_px = 175.0;
  depth_decoy.image_supporting_samples = 1;

  const auto noisy_depth_ray = makePointing(
    0.0, 0.0, 1.20, 3.2, 0.0, -0.20, 0.90, true);
  const auto result = resolvePlaceDestination(
    {intended, depth_decoy},
    {noisy_depth_ray, noisy_depth_ray, noisy_depth_ray}, "bear_doll");

  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, ImageCannotOverrideTwoMetreDepthConflict)
{
  DestinationRegion false_table = inferDestinationRegion(
    "white_side_table_2", 4.92, -1.26);
  false_table.center.z = 0.66;
  false_table.image_projection_valid = true;
  false_table.image_ray_distance_px = 106.5;
  false_table.image_supporting_samples = 10;

  const auto ray = makePointing(
    1.0, -0.2, 1.2, 0.2, -1.0, -0.05, 0.90, true);
  const auto result = resolvePlaceDestination(
    {false_table}, {ray, ray, ray, ray}, "bear_doll");

  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, RejectsLargeUnanchoredThreeDMiss)
{
  DestinationRegion implausible = inferDestinationRegion(
    "cardboard_box", 3.2, 0.0);
  implausible.center.z = 0.10;

  // Horizontally aligned in XY, but more than one metre away in height and
  // the horizontal ray can never intersect the destination surface.
  const auto horizontal_ray = makePointing(
    0.0, 0.0, 1.20, 1.0, 0.0, 0.0);
  const auto result = resolvePlaceDestination(
    {implausible}, {horizontal_ray, horizontal_ray, horizontal_ray},
    "bear_doll");

  EXPECT_FALSE(result.valid);
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

TEST(PlaceDestinationResolverTest, RejectsSeverelyInconsistentElevation)
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

  EXPECT_FALSE(result.valid);
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

TEST(PlaceDestinationResolverTest, NeverReversesRayToFitDestination)
{
  DestinationRegion destination = inferDestinationRegion("square_low_table", 2.0, 0.0);
  destination.center.z = 0.60;
  const auto reversed_ray = makePointing(0.0, 0.0, 1.20, -2.0, 0.0, 0.60);

  const auto result = resolvePlaceDestination(
    {destination}, {reversed_ray, reversed_ray, reversed_ray}, "spray_bottle");

  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, CapturedPlaneAndDepthAgreeOnKnownThreeDScene)
{
  auto target = inferDestinationRegion("table", 3.0, 0.0);
  target.center.z = 0.6;
  auto distractor = inferDestinationRegion("box", 3.0, 1.0);
  distractor.center.z = 0.6;
  auto ray = makePointing(0.2, 0.0, 1.2, 2.8, 0.0, -0.6, 0.95, true);
  ray.plane_valid = true;
  ray.plane_origin.z = 1.2;
  ray.plane_normal.y = 1.0;
  const auto result = resolvePlaceDestination(
    {target, distractor}, {ray, ray, ray}, "bear_doll");
  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(target.name, result.name);
}

TEST(PlaceDestinationResolverTest, PlaneCannotRescueAContradictoryMeasuredRay)
{
  auto target = inferDestinationRegion("table", 3.0, 0.0);
  target.center.z = 0.6;
  auto ray = makePointing(0.2, 0.0, 1.2, -1.0, 0.0, 0.0, 0.95, true);
  ray.plane_valid = true;
  ray.plane_origin.z = 1.2;
  ray.plane_normal.y = 1.0;
  EXPECT_FALSE(resolvePlaceDestination({target}, {ray, ray}, "bear_doll").valid);
}

TEST(PlaceDestinationResolverTest, LevelMeasuredGestureCanIndicateLowFurniture)
{
  // Geometry from 20261022 phase00-frame1 (not moderator ground truth).
  auto target = inferDestinationRegion("white_side_table_1", 1.22, -3.11);
  target.center.z = 0.66;
  auto ray = makePointing(2.1921, -0.1347, 1.2660,
    -0.50016, -0.86588, -0.00985, 0.97, true);
  ray.plane_valid = true;
  ray.plane_origin = ray.origin;
  ray.plane_normal.z = 1.0;
  auto result = resolvePlaceDestination({target}, {ray, ray, ray}, "bear_doll");
  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.horizontal_bearing_used);
  // Neither pose-only guesses nor two isolated frames are enough.
  EXPECT_FALSE(resolvePlaceDestination({target}, {ray, ray}, "bear_doll").valid);
  ray.measured_depth = false;
  EXPECT_FALSE(resolvePlaceDestination({target}, {ray, ray, ray}, "bear_doll").valid);
}

TEST(PlaceDestinationResolverTest, LevelBearingRejectsAmbiguityAndContradictoryDepth)
{
  auto target = inferDestinationRegion("white_side_table_1", 3.0, 0.0);
  target.center.z = 0.6;
  auto other = target; other.name = "other"; other.center.x = 3.2;
  auto ray = makePointing(0, 0, 1.2, 1, 0, 0, 0.95, true);
  ray.plane_valid = true; ray.plane_origin = ray.origin; ray.plane_normal.z = 1;
  EXPECT_FALSE(resolvePlaceDestination({target, other}, {ray,ray,ray}, "bear_doll").valid);
  ray.direction.x = -1;
  EXPECT_FALSE(resolvePlaceDestination({target}, {ray,ray,ray}, "bear_doll").valid);
  ray.direction.x = 1; ray.direction.z = 0.5;
  EXPECT_FALSE(resolvePlaceDestination({target}, {ray,ray,ray}, "bear_doll").valid);
}

TEST(PlaceDestinationResolverTest, PlaneFallbackRejectsBehindCameraAndAmbiguity)
{
  auto front = inferDestinationRegion("front", 3.0, 0.0);
  auto behind = inferDestinationRegion("behind", -3.0, 0.0);
  auto ray = makePointing(0.2, 0.0, 1.2, 1.0, 0.0, 0.0);
  ray.plane_valid = true;
  ray.plane_origin.z = 1.2;
  ray.plane_normal.y = 1.0;
  ray.camera_forward.x = 1.0;
  auto result = resolvePlaceDestination({front, behind}, {ray, ray}, "bear_doll");
  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_EQ(front.name, result.name);
  auto collinear = inferDestinationRegion("collinear", 4.0, 0.0);
  result = resolvePlaceDestination({front, collinear}, {ray, ray}, "bear_doll");
  EXPECT_FALSE(result.valid);
}

TEST(PlaceDestinationResolverTest, PlaneDistanceIsInvariantUnderRigidCameraMotion)
{
  geometry_msgs::msg::Point p, origin;
  geometry_msgs::msg::Vector3 normal;
  p.x = 3.0; p.y = 0.2; p.z = 0.6;
  origin.z = 1.2;
  normal.y = 1.0;
  const double before = pointingPlaneAngle(p, origin, normal);
  // Rotate 90 degrees about Z and translate. Both geometries move together.
  p.x = -0.2 + 4.0; p.y = 3.0 - 2.0; p.z += 0.7;
  origin.x = 4.0; origin.y = -2.0; origin.z += 0.7;
  normal.x = -1.0; normal.y = 0.0;
  EXPECT_NEAR(before, pointingPlaneAngle(p, origin, normal), 1e-12);
}

}  // namespace
}  // namespace interactive_cleanup
