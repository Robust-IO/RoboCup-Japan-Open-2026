#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "interactive_cleanup/place_destination_resolver.hpp"

namespace interactive_cleanup
{
namespace
{

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

  const auto result = resolvePlaceDestination(
    {aligned, off_axis}, {pointing_a, pointing_b}, "soysauce");

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

  PointingObservation low_conf_outlier = high_conf_a;
  low_conf_outlier.confidence = 0.15;
  low_conf_outlier.direction.x = 0.31;
  low_conf_outlier.direction.y = 0.95;

  const auto result = resolvePlaceDestination(
    {aligned, off_axis},
    {high_conf_a, high_conf_b, low_conf_outlier},
    "soysauce");

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(aligned.name, result.name);
}

}  // namespace
}  // namespace interactive_cleanup
