#pragma once
#include "uav/core.hpp"
#include <optional>
#include <vector>
namespace uav {
enum class Request { Offboard, Arm, Land };
enum class Outcome { Running, Complete, AbortedLanded, Failed };
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
    if (attempts_ >= config_.max_attempts) {
      aborted_ = true;
      reason_ = "COMMAND_TIMEOUT";
      outcome_ = Outcome::Failed;
      return;
    }
    d.request = req;
    ++attempts_;
    last_request_ = now;
  }

public:
  explicit MissionController(MissionConfig config = {})
      : config_(std::move(config)), safety_(config_.limits) {
    config_.validate();
  }
  State state() const { return mission_.state(); }
  Outcome outcome() const { return outcome_; }
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
    // Initial observation may be absent; do not arm until all prerequisites arrive.
    if (mission_.state() == State::PreFlight && !aborted_) {
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
      enter(State::Landing, now);
      land_started_ = now;
    }
    if (mission_.state() == State::Landing) {
      // Stop offboard heartbeats. PX4 Land mode/failsafe owns descent.
      if (land_started_ < 0)
        land_started_ = now;
      auto d = decision();
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
      if (!o.landing_mode)
        request(d, Request::Land, now);
      d.outcome = outcome_;
      d.reason = reason_;
      return d;
    }
    auto d = decision();
    if (mission_.state() == State::Ready) {
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
    } else if (mission_.state() == State::Takeoff || mission_.state() == State::Navigation) {
      if (!o.armed || !o.offboard) {
        abort(State::Emergency, "UNEXPECTED_VEHICLE_STATE", now);
        return tick_after_abort(now, o);
      }
      if (now - leg_started_ > config_.leg_timeout) {
        abort(State::Emergency, "LEG_TIMEOUT", now);
        return tick_after_abort(now, o);
      }
      auto target = config_.waypoints[waypoint_];
      setpoint_ = step_toward(setpoint_, target, dt, config_.limits.max_speed);
      if (safety_.command(setpoint_) != Fault::None) {
        abort(State::Emergency, "INVALID_COMMAND", now);
        return tick_after_abort(now, o);
      }
      if (distance(o.telemetry.position, target) < config_.limits.arrival_radius &&
          distance(setpoint_, target) < .01) {
        ++waypoint_;
        leg_started_ = now;
        if (mission_.state() == State::Takeoff)
          enter(State::Navigation, now);
        if (waypoint_ == config_.waypoints.size()) {
          enter(State::Landing, now);
          land_started_ = now;
          d = decision();
          request(d, Request::Land, now);
          return d;
        }
      }
      d.stream = true;
    }
    if (outcome_ == Outcome::Failed) {
      d.stream = false;
    }
    d.state = state();
    d.outcome = outcome_;
    d.setpoint = setpoint_;
    d.reason = reason_;
    d.waypoint = waypoint_;
    return d;
  }

private:
  Decision tick_after_abort(double now, const Observation &o) {
    enter(State::Landing, now);
    land_started_ = now;
    auto d = decision();
    if (o.armed)
      request(d, Request::Land, now);
    return d;
  }
};
inline std::string_view name(State s) {
  switch (s) {
  case State::Idle:
    return "IDLE";
  case State::PreFlight:
    return "PRE_FLIGHT_CHECK";
  case State::Ready:
    return "READY";
  case State::Takeoff:
    return "TAKEOFF";
  case State::Navigation:
    return "NAVIGATION";
  case State::Landing:
    return "LANDING";
  case State::Complete:
    return "COMPLETE";
  case State::Abort:
    return "ABORT";
  case State::Emergency:
    return "EMERGENCY";
  case State::ReturnToLaunch:
    return "RETURN_TO_LAUNCH";
  case State::CommunicationLoss:
    return "COMMUNICATION_LOSS";
  }
  return "UNKNOWN";
}
} // namespace uav
