#pragma once
#include "uav/types.hpp"
namespace uav {
enum class State {
  Idle,
  PreFlight,
  Ready,
  Takeoff,
  Navigation,
  Landing,
  Complete,
  Abort,
  Emergency,
  ReturnToLaunch,
  CommunicationLoss
};
class MissionManager {
  State state_ = State::Idle;

public:
  State state() const { return state_; }
  bool transition(State next, bool valid) {
    if (!valid || state_ == State::Complete)
      return false;
    bool exceptional = next == State::Abort || next == State::Emergency ||
                       next == State::CommunicationLoss || next == State::ReturnToLaunch;
    bool normal = (state_ == State::Idle && next == State::PreFlight) ||
                  (state_ == State::PreFlight && next == State::Ready) ||
                  (state_ == State::Ready && next == State::Takeoff) ||
                  (state_ == State::Takeoff && next == State::Navigation) ||
                  (state_ == State::Navigation && next == State::Landing) ||
                  (state_ == State::Landing && next == State::Complete) ||
                  ((state_ == State::Abort || state_ == State::Emergency ||
                    state_ == State::ReturnToLaunch || state_ == State::CommunicationLoss) &&
                   next == State::Landing);
    if (!exceptional && !normal)
      return false;
    state_ = next;
    return true;
  }
};
} // namespace uav
