#include "uav/simulation.hpp"
#include <gtest/gtest.h>

namespace {
class FaultScenario : public testing::TestWithParam<uav::sim::Scenario> {};

TEST_P(FaultScenario, MeetsSafetyInvariants) {
  auto r = uav::sim::run(GetParam());
  for (const auto &c : r.checks)
    EXPECT_TRUE(c.pass) << c.name << ": " << c.detail;
}

INSTANTIATE_TEST_SUITE_P(Catalog, FaultScenario, testing::ValuesIn(uav::sim::catalog()),
                         [](const auto &info) { return info.param.name; });

TEST(FaultScenario, BrokenControllerConfigurationIsCaught) {
  // A suite that cannot fail proves nothing: without a usable tracking monitor, wind drift must
  // fail.
  auto s = uav::sim::catalog()[2];
  ASSERT_EQ(s.name, "wind_strong");
  s.config.tracking_limit = 39;
  EXPECT_FALSE(uav::sim::run(s).pass());
}
} // namespace
