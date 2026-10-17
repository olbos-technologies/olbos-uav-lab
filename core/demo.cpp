#include "uav/mission_controller.hpp"
#include <iostream>
#include <string>
int main(int argc, char **argv) {
  std::string fault = argc > 1 ? argv[1] : "normal";
  if (fault != "normal" && fault != "battery" && fault != "stale" && fault != "geofence" &&
      fault != "abort" && fault != "reject")
    return 2;
  uav::MissionController controller;
  uav::Observation o{{{}, .95, .05, true}, false, false, false, true};
  bool rejected = false;
  for (int i = 0; i < 2400; ++i) {
    double t = i * .05;
    o.telemetry.battery = .95 - t * .0008;
    if (i >= 120 && fault == "battery")
      o.telemetry.battery = .1;
    if (i >= 120 && fault == "stale")
      o.telemetry.age = 2;
    auto measured = o;
    if (i >= 120 && fault == "geofence")
      measured.telemetry.position.east = 50;
    if (i >= 120 && fault == "abort")
      controller.request_abort();
    auto d = controller.tick(t, measured);
    if (d.request) {
      bool accept = fault != "reject" || *d.request != uav::Request::Arm;
      controller.acknowledge(*d.request, accept);
      if (accept)
        switch (*d.request) {
        case uav::Request::Offboard:
          o.offboard = true;
          break;
        case uav::Request::Arm:
          o.armed = true;
          o.landed = false;
          break;
        case uav::Request::Land:
          o.landing_mode = true;
          o.offboard = false;
          break;
        }
      else
        rejected = true;
    }
    if (d.stream && o.armed)
      o.telemetry.position = d.setpoint;
    // Harness fallback emulates descent after offboard stream disappears; not PX4 physics.
    if (!d.stream && o.armed && d.state == uav::State::Landing) {
      o.telemetry.position = uav::step_toward(
          o.telemetry.position, {o.telemetry.position.east, o.telemetry.position.north, 0}, .05, 2);
      if (o.telemetry.position.up < .01) {
        o.landed = true;
        o.armed = false;
      }
    }
    auto mode = d.outcome == uav::Outcome::Complete
                    ? "COMPLETE"
                    : (d.outcome == uav::Outcome::AbortedLanded || d.outcome == uav::Outcome::Failed
                           ? "ABORTED"
                           : uav::name(d.state).data());
    std::cout << "{\"source\":\"KINEMATIC_HARNESS\",\"time_s\":" << t
              << ",\"east\":" << o.telemetry.position.east
              << ",\"north\":" << o.telemetry.position.north
              << ",\"altitude\":" << o.telemetry.position.up
              << ",\"battery\":" << o.telemetry.battery << ",\"mode\":\"" << mode
              << "\",\"alert\":\"" << d.reason << "\",\"waypoint\":" << d.waypoint << "}\n";
    if (d.outcome != uav::Outcome::Running) {
      if (fault == "normal")
        return d.outcome == uav::Outcome::Complete ? 0 : 4;
      if (fault == "stale")
        return d.outcome == uav::Outcome::Failed && o.landed ? 0 : 5;
      return (o.landed && (rejected || d.outcome == uav::Outcome::AbortedLanded)) ? 0 : 6;
    }
  }
  return 7;
}
