#include <gtest/gtest.h>
#include <bsoncxx/json.hpp>
#include "source_stamp.hpp"
TEST(SourceStamp, PreservesSensorTimeRatherThanReceiveDelay) {
  auto h = bsoncxx::from_json(R"({"stamp":{"sec":1793110332,"nanosec":189000000}})");
  const auto t = sigverse_bridge::sourceStamp(h.view());
  EXPECT_EQ(t.sec, 1793110332);
  EXPECT_EQ(t.nanosec, 189000000u);
}
TEST(SourceStamp, RejectsMissingDisabledAndMalformedStamps) {
  for (const auto *json : {R"({})", R"({"stamp":{"sec":0,"nanosec":0}})",
      R"({"stamp":{"sec":1,"nanosec":1000000000}})",
      R"({"stamp":{"sec":1,"nanosec":-1}})",
      R"({"stamp":{"sec":1.5,"nanosec":0}})"}) {
    auto h=bsoncxx::from_json(json);
    EXPECT_THROW(sigverse_bridge::sourceStamp(h.view()), std::exception);
  }
}
