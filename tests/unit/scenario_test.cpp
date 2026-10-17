#include "uav/params.hpp"
#include "uav/simulation.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

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
TEST(FaultScenario, SweepsNeverBreakSafetyInvariants) {
  for (const auto &sweep : uav::sim::sweeps())
    for (size_t i = 0; i < sweep.results.size(); ++i)
      for (const auto &c : sweep.results[i].checks)
        EXPECT_TRUE(c.pass) << sweep.name << "=" << sweep.values[i] << " " << c.name << ": "
                            << c.detail;
}

TEST(FaultScenario, WindEnvelopeMatchesAirspeedMargin) {
  // 8 m/s airspeed minus the 2 m/s mission speed: upwind legs stop keeping pace above 6 m/s.
  auto wind = uav::sim::sweeps()[0];
  ASSERT_EQ(wind.name, "wind");
  for (size_t i = 0; i < wind.values.size(); ++i)
    EXPECT_EQ(wind.results[i].outcome == uav::Outcome::Complete, wind.values[i] <= 6)
        << wind.values[i] << " m/s";
}

class ShippedConfig : public testing::TestWithParam<std::string> {};

TEST_P(ShippedConfig, CatalogPassesForConfiguredMission) {
  std::ifstream in(std::string(UAV_SOURCE_DIR) + "/config/" + GetParam());
  ASSERT_TRUE(in) << GetParam();
  auto config = uav::parse_params(in).mission();
  for (const auto &s : uav::sim::catalog(config)) {
    auto r = uav::sim::run(s);
    for (const auto &c : r.checks)
      EXPECT_TRUE(c.pass) << s.name << " " << c.name << ": " << c.detail;
  }
}

INSTANTIATE_TEST_SUITE_P(Configs, ShippedConfig,
                         testing::Values("controller.yaml", "waypoint-mission.yaml"),
                         [](const auto &info) {
                           auto n = info.param.substr(0, info.param.find('.'));
                           for (auto &ch : n)
                             if (ch == '-')
                               ch = '_';
                           return n;
                         });

TEST(Params, ParsesNodeParameterFile) {
  std::istringstream in(R"(sitl_mission:
  ros__parameters:
    max_speed: 1.5   # m/s
    stale_seconds: 2.0
    takeoff_height: 4.0
    waypoints_enu: [5.0, 0.0, 4.0,
                    5.0, 5.0, 4.0]
)");
  auto p = uav::parse_params(in);
  EXPECT_DOUBLE_EQ(p.max_speed, 1.5);
  auto c = p.mission();
  ASSERT_EQ(c.waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(c.waypoints[0].up, 4);
  EXPECT_DOUBLE_EQ(c.waypoints[2].north, 5);
  EXPECT_DOUBLE_EQ(c.limits.stale_seconds, 2);
}

TEST(Params, RejectsUnknownKeysAndBadValues) {
  auto parse = [](const char *text) {
    std::istringstream in(text);
    return uav::parse_params(in).mission();
  };
  EXPECT_THROW(parse("max_sped: 2.0\n"), std::invalid_argument);
  EXPECT_THROW(parse("max_speed: fast\n"), std::invalid_argument);
  EXPECT_THROW(parse("max_speed: 2.0m\n"), std::invalid_argument);
  EXPECT_THROW(parse("waypoints_enu: [1.0, 2.0]\n"), std::invalid_argument);
  EXPECT_THROW(parse("waypoints_enu: [1.0, , 2.0, 3.0]\n"), std::invalid_argument);
  EXPECT_THROW(parse("waypoints_enu: [100.0, 0.0, 5.0]\n"), std::invalid_argument);
  EXPECT_NO_THROW(parse("waypoints_enu: []\n"));
}
} // namespace
