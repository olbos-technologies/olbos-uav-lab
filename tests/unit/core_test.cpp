#include "uav/core.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
int main() {
  int checks = 0;
  auto check = [&](bool p) {
    ++checks;
    if (!p)
      throw std::runtime_error("check " + std::to_string(checks));
  };
  auto n = uav::to_ned({3, 5, 7});
  check(n.north == 5 && n.east == 3 && n.down == -7);
  auto e = uav::to_enu(n);
  check(e.east == 3 && e.north == 5 && e.up == 7);
  uav::SafetySupervisor s;
  uav::Telemetry t{{0, 0, 5}, .8, .1, true};
  check(s.evaluate(t) == uav::Fault::None);
  t.age = 2;
  check(s.evaluate(t) == uav::Fault::StaleTelemetry);
  t.age = .1;
  t.battery = .1;
  check(s.evaluate(t) == uav::Fault::LowBattery);
  t.battery = .8;
  t.position.up = 16;
  check(s.evaluate(t) == uav::Fault::Geofence);
  t.position.up = 5;
  t.position_valid = false;
  check(s.evaluate(t) == uav::Fault::InvalidPosition);
  t.position_valid = true;
  t.position.east = std::numeric_limits<double>::quiet_NaN();
  check(s.evaluate(t) == uav::Fault::InvalidPosition);
  check(s.command({41, 0, 1}) == uav::Fault::InvalidCommand);
  check(s.command({0, 0, -1}) == uav::Fault::InvalidCommand);
  check(uav::distance({}, uav::step_toward({}, {10, 0, 0}, .1, 2)) <= .200001);
  check(uav::distance(uav::step_toward({}, {.1, 0, 0}, .1, 2), {.1, 0, 0}) < 1e-9);
  bool threw = false;
  try {
    uav::step_toward({}, {}, -1, 2);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw);
  uav::Limits bad;
  bad.max_speed = 0;
  threw = false;
  try {
    uav::SafetySupervisor x(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw);
  uav::MissionManager m;
  check(!m.transition(uav::State::Takeoff, true));
  check(m.transition(uav::State::PreFlight, true));
  check(!m.transition(uav::State::Ready, false));
  check(m.transition(uav::State::Ready, true));
  check(m.transition(uav::State::Takeoff, true));
  check(m.transition(uav::State::Navigation, true));
  check(m.transition(uav::State::Landing, true));
  check(m.transition(uav::State::Complete, true));
  check(!m.transition(uav::State::Takeoff, true));
  uav::MissionManager abort;
  check(abort.transition(uav::State::CommunicationLoss, true));
  check(abort.transition(uav::State::Landing, true));
  std::cout << checks << " core checks passed\n";
}
