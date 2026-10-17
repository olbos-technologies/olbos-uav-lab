#pragma once
#include "uav/core.hpp"
#include <optional>
#include <vector>
namespace uav {
enum class Request { Offboard, Arm, Land };
enum class Outcome { Running, Complete, AbortedLanded, Failed };
inline std::string_view name(Outcome o) {
  switch (o) {
  case Outcome::Running:
    return "RUNNING";
  case Outcome::Complete:
    return "COMPLETE";
  case Outcome::AbortedLanded:
    return "ABORTED_LANDED";
  case Outcome::Failed:
    return "FAILED";
  }
  return "UNKNOWN";
}
struct Observation {
  Telemetry telemetry;
  bool armed = false, offboard = false, landing_mode = false, landed = false;
};
struct MissionConfig {
  Limits limits;
  std::vector<Enu> waypoints{{0, 0, 5}, {10, 0, 5}, {10, 10, 5}, {0, 0, 5}};
  double warmup_seconds = 2, request_interval = 2, deadline = 90, leg_timeout = 30,
         landing_timeout = 40;
  unsigned max_attempts = 3;
  void validate() const {
    limits.validate();
    if (waypoints.empty() || waypoints.size() > 64 || max_attempts == 0 || max_attempts > 10)
      throw std::invalid_argument("invalid mission size/retry bound");
    SafetySupervisor s(limits);
    for (auto p : waypoints)
      if (s.command(p) != Fault::None || p.up < 1)
        throw std::invalid_argument("unsafe waypoint");
    if (std::hypot(waypoints[0].east, waypoints[0].north) > 0.001)
      throw std::invalid_argument("first waypoint must be vertical takeoff at origin");
    for (double t : {warmup_seconds, request_interval, deadline, leg_timeout, landing_timeout})
      if (!std::isfinite(t) || t <= 0)
        throw std::invalid_argument("invalid timeout");
    // Reject plans that would time out in flight. 1.5x allows for vehicle tracking lag.
    double flight = 0;
    Enu previous{};
    for (auto p : waypoints) {
      double leg = 1.5 * distance(previous, p) / limits.max_speed;
      if (leg > leg_timeout)
        throw std::invalid_argument("waypoint leg exceeds leg timeout");
      flight += leg;
      previous = p;
    }
    if (warmup_seconds + 2 * request_interval + flight + landing_timeout > deadline)
      throw std::invalid_argument("route exceeds mission deadline");
  }
};
struct Decision {
  State state = State::Idle;
  Outcome outcome = Outcome::Running;
  bool stream = false;
  Enu setpoint;
  std::optional<Request> request;
  std::string_view reason = "NORMAL";
  size_t waypoint = 0;
};
// Deterministic reducer: no ROS, threads, I/O or ML. Caller owns a monotonic clock.
class MissionController {
  MissionConfig config_;
  SafetySupervisor safety_;
  MissionManager mission_;
  Outcome outcome_ = Outcome::Running;
  Enu setpoint_{};
  double last_time_ = -1, entered_ = 0, leg_started_ = 0, land_started_ = -1, last_request_ = -1e9;
  unsigned attempts_ = 0;
  size_t waypoint_ = 0;
  bool ever_armed_ = false, aborted_ = false, cancel_ = false, rejected_ = false;
  std::optional<Request> pending_;
  std::string_view reason_ = "NORMAL";
  void enter(State s, double now) {
    if (!mission_.transition(s, true))
      throw std::logic_error("invalid internal transition");
    entered_ = now;
    attempts_ = 0;
    pending_.reset();
    last_request_ = -1e9;
  }
  void abort(State s, std::string_view why, double now) {
    if (aborted_ || outcome_ != Outcome::Running)
      return;
    aborted_ = true;
    reason_ = why;
    enter(s, now);
  }
  Decision decision() const {
    return {mission_.state(), outcome_, false, setpoint_, {}, reason_, waypoint_};
  }
  void request(Decision &d, Request req, double now) {
    if (pending_ != req) {
      pending_ = req;
      attempts_ = 0;
      last_request_ = -1e9;
    }
    if (now - last_request_ < config_.request_interval)
      return;
    // Land is the fallback for every fault, so it is retried until landing_timeout instead.
    if (attempts_ >= config_.max_attempts && req != Request::Land) {
      aborted_ = true;
      reason_ = "COMMAND_TIMEOUT";
      outcome_ = Outcome::Failed;
      return;
    }
    d.request = req;
    ++attempts_;
    last_request_ = now;
  }

  void begin_landing(double now) {
    enter(State::Landing, now);
    land_started_ = now;
  }
  // Copies live controller state into a phase decision; failure always stops the stream.
  Decision finish(Decision d) const {
    d.state = mission_.state();
    d.outcome = outcome_;
    d.setpoint = setpoint_;
    d.reason = reason_;
    d.waypoint = waypoint_;
    if (outcome_ == Outcome::Failed)
      d.stream = false;
    return d;
  }
  // Latched interrupts that apply in every non-terminal state.
  void check_interrupts(double now, double dt) {
    if (dt > .5)
      abort(State::CommunicationLoss, "CONTROL_LOOP_STALL", now);
    if (cancel_)
      abort(State::Abort, "OPERATOR_ABORT", now);
    if (rejected_) {
      rejected_ = false;
      abort(State::Emergency, "COMMAND_REJECTED", now);
    }
    if (now > config_.deadline)
      abort(State::Emergency, "MISSION_DEADLINE", now);
  }
  // Initial observation may be absent; do not arm until all prerequisites arrive.
  Decision preflight(double now, const Observation &o) {
    if (safety_.evaluate(o.telemetry) == Fault::None && o.landed && !o.armed) {
      if (std::hypot(o.telemetry.position.east, o.telemetry.position.north) > 2 ||
          std::abs(o.telemetry.position.up) > 1)
        abort(State::Emergency, "INVALID_SPAWN", now);
      else {
        setpoint_ = o.telemetry.position;
        enter(State::Ready, now);
      }
    }
    return decision();
  }
  Decision ready(double now, const Observation &o) {
    auto d = decision();
    d.stream = true;
    if (now - entered_ < config_.warmup_seconds)
      return d;
    if (!o.offboard)
      request(d, Request::Offboard, now);
    else if (!o.armed)
      request(d, Request::Arm, now);
    else {
      ever_armed_ = true;
      enter(State::Takeoff, now);
      leg_started_ = now;
    }
    return finish(d);
  }
  Decision fly(double now, const Observation &o, double dt) {
    if (!o.armed || !o.offboard)
      return abort_in_flight("UNEXPECTED_VEHICLE_STATE", now, o);
    if (now - leg_started_ > config_.leg_timeout)
      return abort_in_flight("LEG_TIMEOUT", now, o);
    auto target = config_.waypoints[waypoint_];
    setpoint_ = step_toward(setpoint_, target, dt, config_.limits.max_speed);
    if (safety_.command(setpoint_) != Fault::None)
      return abort_in_flight("INVALID_COMMAND", now, o);
    if (distance(o.telemetry.position, target) < config_.limits.arrival_radius &&
        distance(setpoint_, target) < .01) {
      ++waypoint_;
      leg_started_ = now;
      if (mission_.state() == State::Takeoff)
        enter(State::Navigation, now);
      if (waypoint_ == config_.waypoints.size()) {
        begin_landing(now);
        auto d = decision();
        request(d, Request::Land, now);
        return d;
      }
    }
    auto d = decision();
    d.stream = true;
    return finish(d);
  }
  Decision abort_in_flight(std::string_view why, double now, const Observation &o) {
    abort(State::Emergency, why, now);
    begin_landing(now);
    auto d = decision();
    if (o.armed)
      request(d, Request::Land, now);
    return d;
  }
  // Offboard heartbeats stop here; PX4 Land mode/failsafe owns descent.
  Decision landing(double now, const Observation &o) {
    if (land_started_ < 0)
      land_started_ = now;
    bool fresh = std::isfinite(o.telemetry.age) && o.telemetry.age >= 0 &&
                 o.telemetry.age <= config_.limits.stale_seconds;
    if (ever_armed_ && fresh && o.landed && !o.armed) {
      if (aborted_)
        outcome_ = Outcome::AbortedLanded;
      else {
        enter(State::Complete, now);
        outcome_ = Outcome::Complete;
      }
      return decision();
    }
    if (now - land_started_ > config_.landing_timeout) {
      reason_ = "LANDING_UNVERIFIED";
      outcome_ = Outcome::Failed;
      return decision();
    }
    auto d = decision();
    if (!o.landing_mode)
      request(d, Request::Land, now);
    d.outcome = outcome_;
    d.reason = reason_;
    return d;
  }

public:
  explicit MissionController(MissionConfig config = {})
      : config_(std::move(config)), safety_(config_.limits) {
    config_.validate();
  }
  State state() const { return mission_.state(); }
  Outcome outcome() const { return outcome_; }
  size_t waypoint_count() const { return config_.waypoints.size(); }
  void request_abort() { cancel_ = true; }
  void acknowledge(Request request, bool accepted_or_in_progress) {
    if (pending_ == request && !accepted_or_in_progress && outcome_ == Outcome::Running)
      rejected_ = true;
  }
  Decision tick(double now, const Observation &o) {
    if (outcome_ != Outcome::Running)
      return decision();
    if (!std::isfinite(now) || now < 0 || (last_time_ >= 0 && now <= last_time_)) {
      reason_ = "INVALID_CLOCK";
      outcome_ = Outcome::Failed;
      return decision();
    }
    double dt = last_time_ < 0 ? .05 : now - last_time_;
    last_time_ = now;
    if (mission_.state() == State::Idle)
      enter(State::PreFlight, now);
    check_interrupts(now, dt);
    if (mission_.state() == State::PreFlight && !aborted_)
      return preflight(now, o);
    // Any active leg must satisfy safety before a setpoint or arm request is emitted.
    if (!aborted_ && mission_.state() != State::Landing) {
      auto fault = safety_.evaluate(o.telemetry);
      if (fault != Fault::None)
        abort(fault == Fault::StaleTelemetry ? State::CommunicationLoss : State::Emergency,
              name(fault), now);
    }
    if (aborted_ && mission_.state() != State::Landing) {
      if (!ever_armed_ && !o.armed) {
        outcome_ = Outcome::Failed;
        return decision();
      }
      begin_landing(now);
    }
    if (mission_.state() == State::Landing)
      return landing(now, o);
    if (mission_.state() == State::Ready)
      return ready(now, o);
    return fly(now, o, dt);
  }
};
} // namespace uav
