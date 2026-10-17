#include "uav/mission_controller.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace {
struct Harness {
  uav::MissionController controller;
  uav::Observation observation{{{}, .9, .05, true}, false, false, false, true};
  double time = -.05;
  uav::Decision step(bool execute = true) {
    time += .05;
    auto d = controller.tick(time, observation);
    if (execute && d.request) {
      controller.acknowledge(*d.request, true);
      switch (*d.request) {
      case uav::Request::Offboard:
        observation.offboard = true;
        break;
      case uav::Request::Arm:
        observation.armed = true;
        observation.landed = false;
        break;
      case uav::Request::Land:
        observation.landing_mode = true;
        observation.offboard = false;
        break;
      }
    }
    if (execute && d.stream && observation.armed)
      observation.telemetry.position = d.setpoint;
    if (execute && d.state == uav::State::Landing && observation.armed) {
      auto p = observation.telemetry.position;
      observation.telemetry.position = uav::step_toward(p, {p.east, p.north, 0}, .05, 2);
      if (observation.telemetry.position.up < .01) {
        observation.armed = false;
        observation.landed = true;
      }
    }
    return d;
  }
  void airborne() {
    for (int i = 0; i < 70; ++i)
      step();
    ASSERT_TRUE(observation.armed);
    ASSERT_GT(observation.telemetry.position.up, 0);
  }
};
TEST(MissionController, CompletesObservedWaypointMission) {
  Harness h;
  uav::Decision d;
  for (int i = 0; i < 1800 && d.outcome == uav::Outcome::Running; ++i)
    d = h.step();
  EXPECT_EQ(d.outcome, uav::Outcome::Complete);
  EXPECT_EQ(d.state, uav::State::Complete);
  EXPECT_EQ(d.waypoint, 4u);
  EXPECT_TRUE(h.observation.landed);
  EXPECT_FALSE(h.observation.armed);
}
TEST(MissionController, MissingTelemetryNeverArms) {
  Harness h;
  h.observation.telemetry.position_valid = false;
  for (int i = 0; i < 100; ++i) {
    auto d = h.step(false);
    EXPECT_FALSE(d.stream);
    EXPECT_FALSE(d.request);
  }
  EXPECT_EQ(h.controller.state(), uav::State::PreFlight);
}
TEST(MissionController, WarmsHeartbeatBeforeCommanding) {
  Harness h;
  for (int i = 0; i < 40; ++i)
    EXPECT_FALSE(h.step(false).request);
  bool saw_offboard = false;
  for (int i = 0; i < 5; ++i) {
    auto d = h.step(false);
    if (d.request) {
      EXPECT_EQ(*d.request, uav::Request::Offboard);
      saw_offboard = true;
    }
  }
  EXPECT_TRUE(saw_offboard);
}
TEST(MissionController, AcceptedCommandDoesNotReplaceObservedState) {
  Harness h;
  unsigned requests = 0;
  uav::Decision d;
  for (int i = 0; i < 190; ++i) {
    d = h.step(false);
    if (d.request) {
      ++requests;
      EXPECT_EQ(*d.request, uav::Request::Offboard);
      h.controller.acknowledge(*d.request, true);
    }
  }
  EXPECT_EQ(requests, 3u);
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
  EXPECT_EQ(d.reason, "COMMAND_TIMEOUT");
  EXPECT_FALSE(d.stream);
}
TEST(MissionController, RejectionStopsOffboardStream) {
  Harness h;
  for (int i = 0; i < 50; ++i) {
    auto d = h.step(false);
    if (d.request) {
      h.controller.acknowledge(*d.request, false);
      break;
    }
  }
  auto d = h.step(false);
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "COMMAND_REJECTED");
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
}
TEST(MissionController, UnrelatedAcknowledgmentIsIgnored) {
  Harness h;
  h.step();
  h.controller.acknowledge(uav::Request::Land, false);
  EXPECT_EQ(h.step().reason, "NORMAL");
}
TEST(MissionController, BatteryFaultPreemptsWaypoint) {
  Harness h;
  h.airborne();
  h.observation.telemetry.battery = .1;
  auto d = h.step();
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.state, uav::State::Landing);
  EXPECT_EQ(d.reason, "LOW_BATTERY");
  for (int i = 0; i < 200 && d.outcome == uav::Outcome::Running; ++i)
    d = h.step();
  EXPECT_EQ(d.outcome, uav::Outcome::AbortedLanded);
}
TEST(MissionController, AbortLatchesUntilLanding) {
  Harness h;
  h.airborne();
  h.controller.request_abort();
  auto d = h.step();
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "OPERATOR_ABORT");
  for (int i = 0; i < 200 && d.outcome == uav::Outcome::Running; ++i)
    d = h.step();
  EXPECT_EQ(d.outcome, uav::Outcome::AbortedLanded);
  EXPECT_NE(d.outcome, uav::Outcome::Complete);
}
TEST(MissionController, StaleLandingCannotClaimCompletion) {
  Harness h;
  h.airborne();
  h.observation.telemetry.age = 10;
  auto d = h.step();
  EXPECT_EQ(d.reason, "STALE_TELEMETRY");
  EXPECT_FALSE(d.stream);
  for (int i = 0; i < 900 && d.outcome == uav::Outcome::Running; ++i)
    d = h.step();
  EXPECT_TRUE(h.observation.landed);
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
  EXPECT_EQ(d.reason, "LANDING_UNVERIFIED");
}
TEST(MissionController, IgnoredLandRequestRetriedUntilLandingTimeout) {
  Harness h;
  h.airborne();
  h.observation.telemetry.battery = .1;
  unsigned lands = 0;
  uav::Decision d;
  for (int i = 0; i < 1000 && d.outcome == uav::Outcome::Running; ++i) {
    d = h.step(false);
    if (d.request && *d.request == uav::Request::Land)
      ++lands;
  }
  EXPECT_GT(lands, 3u);
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
  EXPECT_EQ(d.reason, "LANDING_UNVERIFIED");
}
TEST(MissionController, TrackingErrorStopsStreaming) {
  Harness h;
  h.airborne();
  h.observation.telemetry.position.east += 4;
  uav::Decision d;
  for (int i = 0; i < 30 && d.reason == "NORMAL"; ++i)
    d = h.step(false);
  EXPECT_EQ(d.reason, "TRACKING_ERROR");
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.state, uav::State::Landing);
}
TEST(MissionController, InvalidPositionPreemptsControl) {
  Harness h;
  h.airborne();
  h.observation.telemetry.position_valid = false;
  auto d = h.step();
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "INVALID_POSITION");
}
TEST(MissionController, GeofencePreemptsControl) {
  Harness h;
  h.airborne();
  h.observation.telemetry.position.east = 50;
  auto d = h.step(false);
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "GEOFENCE");
}
TEST(MissionController, SchedulerStallStopsStreaming) {
  Harness h;
  h.airborne();
  h.time += 1;
  auto d = h.step(false);
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "CONTROL_LOOP_STALL");
}
TEST(MissionController, NonMonotonicClockFailsClosed) {
  Harness h;
  h.step();
  auto d = h.controller.tick(h.time, h.observation);
  EXPECT_FALSE(d.stream);
  EXPECT_EQ(d.reason, "INVALID_CLOCK");
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
}
TEST(MissionController, NonFiniteClockFailsClosed) {
  uav::MissionController c;
  auto d = c.tick(std::numeric_limits<double>::quiet_NaN(), {});
  EXPECT_EQ(d.outcome, uav::Outcome::Failed);
  EXPECT_FALSE(d.request);
}
TEST(MissionController, UnsafePlanRejectedBeforeFlight) {
  uav::MissionConfig c;
  c.waypoints.push_back({100, 0, 5});
  EXPECT_THROW((uav::MissionController(c)), std::invalid_argument);
  c.waypoints.clear();
  EXPECT_THROW((uav::MissionController(c)), std::invalid_argument);
  c.waypoints = {{5, 0, 5}};
  EXPECT_THROW((uav::MissionController(c)), std::invalid_argument);
}
TEST(MissionController, InfeasiblePlanRejectedBeforeFlight) {
  uav::MissionConfig c;
  c.waypoints = {{0, 0, 5}, {35, 0, 5}, {-35, 0, 5}};
  EXPECT_THROW((uav::MissionController(c)), std::invalid_argument);
  c.waypoints = {{0, 0, 5}};
  for (int i = 0; i < 6; ++i) {
    c.waypoints.push_back({20, 0, 5});
    c.waypoints.push_back({-20, 0, 5});
  }
  EXPECT_THROW((uav::MissionController(c)), std::invalid_argument);
  c.deadline = 400;
  EXPECT_NO_THROW((uav::MissionController(c)));
}
TEST(MissionController, SetpointSpeedIsBounded) {
  Harness h;
  h.airborne();
  auto previous = h.observation.telemetry.position;
  for (int i = 0; i < 100; ++i) {
    auto d = h.step();
    if (d.stream) {
      EXPECT_LE(uav::distance(previous, d.setpoint), .100001);
    }
    previous = d.setpoint;
  }
}
} // namespace
