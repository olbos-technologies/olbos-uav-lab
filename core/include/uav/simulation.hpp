#pragma once
// Fault-injection scenarios for the shared MissionController.
// The plant is a deterministic kinematic approximation of a PX4 multicopter, not a physics model:
// first-order tracking of the *estimated* position (as PX4 does), bounded velocity, an assumed
// 8 m/s airspeed authority against wind, Land mode descent and auto-disarm. Results show
// controller logic under faults; they are not evidence of flight safety.
#include "uav/mission_controller.hpp"
#include <optional>
#include <string>
#include <vector>
namespace uav::sim {

enum class Policy { Accept, Reject, Ignore };

// Times are seconds from mission start; negative means the fault never occurs.
struct Faults {
  double wind_east = 0, wind_at = 0;
  double battery_drain = 0.0008, battery_sag_at = -1, battery_sag_to = 0.12;
  double gps_loss_at = -1;
  double gps_jump_at = -1;
  Enu gps_jump{};
  double motor_failure_at = -1;
  double telemetry_freeze_at = -1, telemetry_freeze_for = -1; // duration < 0: never resumes
  double telemetry_jitter = 0; // receipt age sawtooth period; 0 = steady 50 ms
  double abort_at = -1;
  double stall_at = -1, stall_seconds = 0;
  double land_detector_stuck_at = -1;
  Policy offboard = Policy::Accept, arm = Policy::Accept, land = Policy::Accept;
};

class Plant {
  static constexpr double kTau = 0.3, kMaxHorizontal = 5, kMaxVertical = 3, kLandSpeed = 0.7,
                          kSinkSpeed = 3, kAirspeed = 8, kAutoDisarm = 1;
  Faults f_;
  bool frozen_ = false;
  Observation snapshot_{};
  double on_ground_ = 0;

  static Enu clamp(Enu v) {
    double h = std::hypot(v.east, v.north);
    if (h > kMaxHorizontal) {
      v.east *= kMaxHorizontal / h;
      v.north *= kMaxHorizontal / h;
    }
    v.up = std::clamp(v.up, -kMaxVertical, kMaxVertical);
    return v;
  }
  // Ground velocity = air velocity + wind. Position control (with its integrator) commands the
  // ground velocity it wants; the airframe can only produce kAirspeed relative to the air, so
  // wind beyond that pushes it off course however hard it tries.
  Enu with_wind(Enu wanted, double t) const {
    double wind = t >= f_.wind_at ? f_.wind_east : 0;
    double ae = wanted.east - wind, an = wanted.north;
    double a = std::hypot(ae, an);
    if (a > kAirspeed) {
      ae *= kAirspeed / a;
      an *= kAirspeed / a;
    }
    return {ae + wind, an, wanted.up};
  }
  bool active(double at, double t) const { return at >= 0 && t >= at; }

public:
  Enu position{};
  double battery = .9, touchdown_speed = 0;
  bool armed = false, offboard = false, landing = false, landed = true, ever_armed = false;

  explicit Plant(Faults f) : f_(f) {}

  // Returns the PX4 ACK result, or nothing when the command is silently dropped.
  std::optional<bool> command(Request r) {
    Policy p = r == Request::Offboard ? f_.offboard : r == Request::Arm ? f_.arm : f_.land;
    if (p == Policy::Ignore)
      return std::nullopt;
    if (p == Policy::Reject)
      return false;
    switch (r) {
    case Request::Offboard:
      offboard = true;
      break;
    case Request::Arm:
      if (!offboard)
        return false;
      armed = ever_armed = true;
      break;
    case Request::Land:
      if (!armed)
        return false;
      landing = true;
      offboard = false;
      break;
    }
    return true;
  }

  void step(double t, double dt, bool streaming, Enu setpoint) {
    battery = .9 - f_.battery_drain * t;
    if (active(f_.battery_sag_at, t))
      battery = std::min(battery, f_.battery_sag_to);
    battery = std::max(battery, 0.0);
    if (!armed)
      return;
    Enu v{};
    if (active(f_.motor_failure_at, t))
      v = {0, 0, -kSinkSpeed};
    else if (landing)
      v = {0, 0, -kLandSpeed};
    else if (offboard && streaming)
      // PX4 steers its estimate onto the setpoint, so an estimate bias moves the real airframe.
      v = clamp({(setpoint.east - estimate(t).east) / kTau,
                 (setpoint.north - estimate(t).north) / kTau,
                 (setpoint.up - estimate(t).up) / kTau});
    // Without setpoints or Land mode, PX4 failsafe holds position (velocity zero).
    if (landed && v.up <= 0)
      v = {};
    else if (active(f_.motor_failure_at, t))
      v.east += t >= f_.wind_at ? f_.wind_east : 0; // no thrust: the airframe drifts with the wind
    else
      v = with_wind(v, t);
    position = {position.east + v.east * dt, position.north + v.north * dt,
                position.up + v.up * dt};
    if (position.up <= 0 && !landed) {
      touchdown_speed = std::max(touchdown_speed, -v.up);
      position.up = 0;
      landed = true;
    } else if (position.up > 0.02) {
      landed = false;
    }
    on_ground_ = landed ? on_ground_ + dt : 0;
    // PX4 auto-disarms after touchdown, but only once its land detector agrees.
    if (on_ground_ > kAutoDisarm && !active(f_.land_detector_stuck_at, t)) {
      armed = offboard = landing = false;
    }
  }

  Enu estimate(double t) const {
    if (!active(f_.gps_jump_at, t))
      return position;
    return {position.east + f_.gps_jump.east, position.north + f_.gps_jump.north,
            position.up + f_.gps_jump.up};
  }

  Observation observe(double t) {
    bool resumed =
        f_.telemetry_freeze_for >= 0 && t >= f_.telemetry_freeze_at + f_.telemetry_freeze_for;
    if (resumed)
      frozen_ = false;
    else if (frozen_ || active(f_.telemetry_freeze_at, t)) {
      if (!frozen_)
        snapshot_ = observe_live(t);
      frozen_ = true;
      auto o = snapshot_;
      o.telemetry.age = t - f_.telemetry_freeze_at + .05;
      return o;
    }
    return observe_live(t);
  }

private:
  Observation observe_live(double t) const {
    Observation o;
    o.telemetry.position = estimate(t);
    o.telemetry.position_valid = !active(f_.gps_loss_at, t);
    o.telemetry.battery = battery;
    o.telemetry.age = f_.telemetry_jitter > 0 ? std::fmod(t, f_.telemetry_jitter) + .05 : .05;
    o.armed = armed;
    o.offboard = offboard;
    o.landing_mode = landing;
    o.landed = landed && !active(f_.land_detector_stuck_at, t);
    return o;
  }
};

struct Expectation {
  std::vector<Outcome> outcomes;
  std::string alert; // first non-NORMAL reason; '|' separates alternatives, an empty one = none
  double detect_from = -1; // when the fault becomes observable
  double max_detect = -1;  // allowed seconds from detect_from to first alert
  unsigned min_land_requests = 0;
};

struct Scenario {
  std::string name, category, description;
  Faults faults;
  Expectation expect;
  MissionConfig config{};
};

struct Check {
  std::string name;
  bool pass = true;
  std::string detail;
};

struct Result {
  Scenario scenario;
  Outcome outcome = Outcome::Running;
  std::string reason, first_alert;
  double alert_at = -1, takeoff_at = -1, landing_at = -1, duration = 0, estimate_error = 0,
         max_altitude = 0, max_distance = 0, min_battery = 1, touchdown_speed = 0;
  unsigned land_requests = 0;
  bool truth_landed = false, truth_armed = false, ever_armed = false;
  std::vector<Check> checks;
  std::vector<std::string> trace;
  bool pass() const {
    for (auto &c : checks)
      if (!c.pass)
        return false;
    return true;
  }
  double detect_latency() const {
    return alert_at >= 0 && scenario.expect.detect_from >= 0
               ? alert_at - scenario.expect.detect_from
               : -1;
  }
};

namespace detail {
inline std::string fixed(double v, int digits = 2) {
  if (!std::isfinite(v))
    return "null";
  std::string s(32, '\0');
  s.resize(std::snprintf(s.data(), s.size(), "%.*f", digits, v));
  return s;
}
// Records only the first violation of each invariant so reports stay readable.
struct Invariant {
  Check check;
  void fail(double t, const std::string &what) {
    if (check.pass) {
      check.pass = false;
      check.detail = "t=" + fixed(t) + "s: " + what;
    }
  }
};
} // namespace detail

// Runs one scenario at 20 Hz and checks controller decisions against independent invariants.
inline Result run(const Scenario &s, bool keep_trace = false) {
  using detail::fixed;
  const double dt = .05;
  const auto &lim = s.config.limits;
  MissionController controller(s.config);
  Plant plant(s.faults);
  Result r;
  r.scenario = s;
  detail::Invariant envelope{
      {"command_envelope", true, "every streamed setpoint inside fence and altitude limits"}},
      rate{{"setpoint_rate", true, "setpoint steps never exceed max_speed x elapsed time"}},
      arm{{"arm_preconditions", true,
           "arm only requested with valid, fresh telemetry and battery"}},
      silence{
          {"stop_streaming_on_fault", true, "no offboard setpoints once a fault reason is set"}};
  Enu last_setpoint{};
  bool streamed = false;
  double last_tick = -1;
  const double end = s.config.deadline + s.config.landing_timeout + 10;
  Decision d;
  for (int i = 0;; ++i) {
    double t = i * dt;
    if (t > end)
      break;
    bool stalled = s.faults.stall_at >= 0 && t >= s.faults.stall_at &&
                   t < s.faults.stall_at + s.faults.stall_seconds;
    if (stalled) {
      plant.step(t, dt, false, d.setpoint);
      continue;
    }
    if (s.faults.abort_at >= 0 && t >= s.faults.abort_at)
      controller.request_abort();
    auto o = plant.observe(t);
    d = controller.tick(t, o);
    if (d.stream) {
      auto p = d.setpoint;
      if (!finite(p) || p.up < -.5 || p.up > lim.max_altitude ||
          std::hypot(p.east, p.north) > lim.radius)
        envelope.fail(t, "setpoint outside envelope");
      if (streamed && distance(last_setpoint, p) > lim.max_speed * (t - last_tick) + 1e-6)
        rate.fail(t, "step " + fixed(distance(last_setpoint, p), 3) + " m");
      if (d.reason != "NORMAL")
        silence.fail(t, "streaming while " + std::string(d.reason));
      last_setpoint = p;
    }
    streamed = d.stream;
    last_tick = t;
    if (d.state == State::Takeoff && r.takeoff_at < 0)
      r.takeoff_at = t;
    if (d.state == State::Landing && r.landing_at < 0)
      r.landing_at = t;
    if (d.reason != "NORMAL" && r.first_alert.empty()) {
      r.first_alert = d.reason;
      r.alert_at = t;
    }
    if (d.request) {
      if (*d.request == Request::Arm &&
          (!o.telemetry.position_valid || o.telemetry.age > lim.stale_seconds ||
           o.telemetry.battery < lim.min_battery))
        arm.fail(t, "arm requested without preconditions");
      if (*d.request == Request::Land)
        ++r.land_requests;
      if (auto ack = plant.command(*d.request))
        controller.acknowledge(*d.request, *ack);
    }
    plant.step(t, dt, d.stream, d.setpoint);
    r.max_altitude = std::max(r.max_altitude, plant.position.up);
    r.max_distance =
        std::max(r.max_distance, std::hypot(plant.position.east, plant.position.north));
    r.min_battery = std::min(r.min_battery, plant.battery);
    r.estimate_error = std::max(r.estimate_error, distance(plant.estimate(t), plant.position));
    r.duration = t;
    if (keep_trace) {
      auto m = o.telemetry.position;
      r.trace.push_back("{\"source\":\"KINEMATIC_HARNESS\",\"scenario\":\"" + s.name +
                        "\",\"time_s\":" + fixed(t) + ",\"east\":" + fixed(m.east, 3) +
                        ",\"north\":" + fixed(m.north, 3) + ",\"altitude\":" + fixed(m.up, 3) +
                        ",\"battery\":" + fixed(o.telemetry.battery, 4) +
                        ",\"age_s\":" + fixed(o.telemetry.age, 3) + ",\"mode\":\"" +
                        std::string(name(d.state)) + "\",\"alert\":\"" + std::string(d.reason) +
                        "\",\"waypoint\":" + std::to_string(d.waypoint) +
                        ",\"waypoints\":" + std::to_string(controller.waypoint_count()) +
                        ",\"state\":\"" + std::string(name(d.state)) + "\",\"outcome\":\"" +
                        std::string(name(d.outcome)) + "\"}");
    }
    if (d.outcome != Outcome::Running)
      break;
  }
  r.outcome = d.outcome;
  r.reason = d.reason;
  r.touchdown_speed = plant.touchdown_speed;
  r.truth_landed = plant.landed;
  r.truth_armed = plant.armed;
  r.ever_armed = plant.ever_armed;

  const auto &e = s.expect;
  r.checks = {envelope.check, rate.check, arm.check, silence.check};
  r.checks.push_back({"terminal_outcome", d.outcome != Outcome::Running,
                      "mission reached a terminal outcome within " + fixed(end, 0) + " s"});
  // Sweeps pass no expected outcomes: only the safety invariants apply to them.
  if (!e.outcomes.empty()) {
    bool expected = false;
    std::string allowed;
    for (auto o : e.outcomes) {
      expected |= o == d.outcome;
      allowed += (allowed.empty() ? "" : "|") + std::string(name(o));
    }
    r.checks.push_back({"expected_outcome", expected,
                        "got " + std::string(name(d.outcome)) + ", expected " + allowed});
    bool alert_ok = false;
    for (size_t start = 0;;) {
      auto bar = e.alert.find('|', start);
      alert_ok |= e.alert.substr(start, bar - start) == r.first_alert;
      if (bar == std::string::npos)
        break;
      start = bar + 1;
    }
    r.checks.push_back({"expected_alert", alert_ok,
                        "first alert " + (r.first_alert.empty() ? "none" : r.first_alert) +
                            ", expected " + (e.alert.empty() ? "none" : e.alert)});
  }
  if (e.max_detect >= 0)
    r.checks.push_back({"detection_latency",
                        r.detect_latency() >= 0 && r.detect_latency() <= e.max_detect,
                        "detected after " + fixed(r.detect_latency()) + " s, limit " +
                            fixed(e.max_detect) + " s"});
  bool claims_landing = d.outcome == Outcome::Complete || d.outcome == Outcome::AbortedLanded;
  r.checks.push_back({"honest_landing_claim", !claims_landing || (plant.landed && !plant.armed),
                      "landing outcomes require the vehicle to be on the ground and disarmed"});
  r.checks.push_back({"complete_only_when_clean",
                      d.outcome != Outcome::Complete || r.first_alert.empty(),
                      "nominal completion is never reported after a fault"});
  if (e.min_land_requests)
    r.checks.push_back({"land_retried", r.land_requests >= e.min_land_requests,
                        std::to_string(r.land_requests) + " Land requests, need " +
                            std::to_string(e.min_land_requests)});
  return r;
}

// Midpoint of the airborne phase of a fault-free run, where faults are injected.
inline double mid_flight(const MissionConfig &config) {
  Scenario probe;
  probe.name = "probe";
  probe.config = config;
  auto r = run(probe);
  if (r.outcome != Outcome::Complete || r.takeoff_at < 0 || r.landing_at <= r.takeoff_at)
    throw std::invalid_argument("mission does not complete fault-free in the kinematic plant: " +
                                r.reason);
  return std::round((r.takeoff_at + .5 * (r.landing_at - r.takeoff_at)) * 20) / 20;
}

// Built-in catalog for a mission. Faults start mid-flight; detection limits derive from the
// configuration (tracking monitor, stale threshold), so a customer mission is judged by its own
// limits.
inline std::vector<Scenario> catalog(const MissionConfig &config = {}) {
  using O = Outcome;
  using detail::fixed;
  const std::vector<O> landed{O::AbortedLanded};
  const auto &lim = config.limits;
  const double mid = mid_flight(config);
  const std::string at = " at " + fixed(mid, 1) + " s";
  const double tracking_detect = config.tracking_seconds + 2;
  std::vector<Scenario> c;
  auto add = [&](std::string name, std::string category, std::string description, Faults f,
                 Expectation e) {
    c.push_back(
        {std::move(name), std::move(category), std::move(description), f, std::move(e), config});
  };
  Faults f;
  add("nominal", "baseline", "Planned mission with no faults.", f, {{O::Complete}, ""});

  f = {};
  f.wind_east = 4;
  add("wind_light", "environment", "Steady 4 m/s crosswind within control authority.", f,
      {{O::Complete}, ""});

  f = {};
  f.wind_east = 14, f.wind_at = mid;
  add("wind_strong", "environment",
      "14 m/s wind" + at + " exceeds control authority; airframe drifts.", f,
      {landed, "TRACKING_ERROR", mid, tracking_detect + config.tracking_limit / 1.5});

  f = {};
  f.battery_sag_at = mid, f.battery_sag_to = lim.min_battery * .6;
  add("battery_sag", "power", "Battery collapses below the reserve" + at + ".", f,
      {landed, "LOW_BATTERY", mid, .1});

  f = {};
  f.battery_drain = (.9 - lim.min_battery) / mid;
  add("battery_fast_drain", "power",
      "Battery drains " + fixed(f.battery_drain * 100, 1) +
          "%/s and crosses the reserve mid-flight.",
      f, {landed, "LOW_BATTERY", mid, .15});

  f = {};
  f.gps_loss_at = mid;
  add("gps_loss", "navigation", "Position estimate becomes invalid" + at + ".", f,
      {landed, "INVALID_POSITION", mid, .1});

  f = {};
  f.gps_jump_at = mid, f.gps_jump = {2 * config.tracking_limit, 0, 0};
  // The autopilot follows its estimate, so it flies the airframe off by the jump and tracking looks
  // normal. Mission-level monitoring cannot see this; the report flags the displacement instead.
  add("gps_jump", "navigation",
      "Position estimate jumps " + fixed(2 * config.tracking_limit, 1) + " m east" + at +
          ". The autopilot follows the bad estimate and physically displaces the airframe.",
      f, {{O::Complete, O::AbortedLanded}, "|TRACKING_ERROR"});

  f = {};
  f.gps_jump_at = mid, f.gps_jump = {lim.radius + 5, 0, 0};
  add("estimator_reset_outside_fence", "navigation",
      "Estimator reset places the vehicle " + fixed(lim.radius + 5, 0) +
          " m east, outside the fence.",
      f, {landed, "GEOFENCE", mid, .1});

  f = {};
  f.motor_failure_at = mid;
  // From low altitude the airframe can hit the ground and disarm before the tracking monitor fires.
  add("motor_failure", "actuator",
      "Loss of thrust" + at + "; vehicle sinks at 3 m/s regardless of setpoints.", f,
      {landed, "TRACKING_ERROR|UNEXPECTED_VEHICLE_STATE", mid,
       tracking_detect + config.tracking_limit / 3});

  f = {};
  f.telemetry_freeze_at = mid;
  add("telemetry_freeze", "communication",
      "All telemetry stops updating" + at +
          ". Landing cannot be verified, so the honest outcome is FAILED.",
      f, {{O::Failed}, "STALE_TELEMETRY", mid, lim.stale_seconds + .15});

  f = {};
  f.telemetry_jitter = .7 * lim.stale_seconds;
  add("telemetry_jitter", "communication",
      "Receipt age oscillates up to " + fixed(f.telemetry_jitter + .05) + " s, below the " +
          fixed(lim.stale_seconds, 1) + " s stale limit. Must not raise false alarms.",
      f, {{O::Complete}, ""});

  f = {};
  f.stall_at = mid, f.stall_seconds = .8;
  add("control_loop_stall", "compute", "Controller is not scheduled for 0.8 s" + at + ".", f,
      {landed, "CONTROL_LOOP_STALL", mid, 1});

  f = {};
  f.abort_at = mid;
  add("operator_abort", "operator", "Operator requests abort" + at + ".", f,
      {landed, "OPERATOR_ABORT", mid, .1});

  f = {};
  f.arm = Policy::Reject;
  add("arm_rejected", "command", "Autopilot rejects the arm command.", f,
      {{O::Failed}, "COMMAND_REJECTED"});

  f = {};
  f.offboard = Policy::Ignore;
  add("offboard_ignored", "command", "Autopilot never answers the offboard mode request.", f,
      {{O::Failed}, "COMMAND_TIMEOUT"});

  f = {};
  f.battery_sag_at = mid, f.battery_sag_to = lim.min_battery * .6, f.land = Policy::Ignore;
  add("land_ignored", "command",
      "Battery fault" + at +
          ", but the autopilot ignores Land. Controller must keep requesting Land "
          "and must not claim a landing.",
      f, {{O::Failed}, "LOW_BATTERY", mid, .1, 10});

  f = {};
  f.land_detector_stuck_at = mid;
  add("land_detector_stuck", "sensor",
      "Land detector never reports touchdown. Mission must not be reported COMPLETE.", f,
      {{O::Failed}, "LANDING_UNVERIFIED"});
  return c;
}

// One fault swept across a range: where does the configuration stop completing, and does it stay
// safe?
struct Sweep {
  std::string name, parameter, unit, description;
  std::vector<double> values;
  std::vector<Result> results;
};

inline std::vector<Sweep> sweeps(const MissionConfig &config = {}) {
  const double mid = mid_flight(config);
  auto range = [](double lo, double hi, double step) {
    std::vector<double> v;
    for (int i = 0; lo + i * step <= hi + 1e-9; ++i)
      v.push_back(lo + i * step);
    return v;
  };
  struct Def {
    std::string name, parameter, unit, description;
    std::vector<double> values;
    void (*apply)(Faults &, double, double);
  };
  const std::vector<Def> defs = {
      {"wind", "steady wind speed", "m/s", "Steady east wind for the whole mission.",
       range(0, 20, 1), [](Faults &f, double v, double) { f.wind_east = v; }},
      {"gps_jump", "position jump", "m",
       "Position estimate jumps east mid-flight; the autopilot follows it and displaces the "
       "airframe.",
       range(0, 10, .5),
       [](Faults &f, double v, double mid) { f.gps_jump_at = mid, f.gps_jump = {v, 0, 0}; }},
      {"telemetry_gap", "telemetry gap", "s",
       "All telemetry freezes mid-flight for this long, then resumes.", range(0, 3, .25),
       [](Faults &f, double v, double mid) {
         f.telemetry_freeze_at = mid, f.telemetry_freeze_for = v;
       }},
      {"loop_stall", "control-loop stall", "s",
       "Controller is not scheduled for this long mid-flight.", range(0, 1.5, .1),
       [](Faults &f, double v, double mid) { f.stall_at = mid, f.stall_seconds = v; }},
  };
  std::vector<Sweep> out;
  for (const auto &d : defs) {
    Sweep sw{d.name, d.parameter, d.unit, d.description, d.values, {}};
    for (double v : d.values) {
      Scenario s;
      s.name = d.name + "_" + detail::fixed(v, 2);
      s.config = config;
      d.apply(s.faults, v, mid);
      s.expect.detect_from = d.name == "wind" ? 0 : mid;
      sw.results.push_back(run(s));
    }
    out.push_back(std::move(sw));
  }
  return out;
}

} // namespace uav::sim
