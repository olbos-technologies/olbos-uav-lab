#pragma once
#include "uav/types.hpp"
namespace uav {
class SafetySupervisor {
  Limits limits_;

public:
  explicit SafetySupervisor(Limits limits = {}) : limits_(limits) { limits_.validate(); }
  bool safe_point(Enu p) const {
    return finite(p) && p.up >= -0.5 && p.up <= limits_.max_altitude &&
           std::hypot(p.east, p.north) <= limits_.radius;
  }
  Fault evaluate(Telemetry t) const {
    if (!t.position_valid || !finite(t.position))
      return Fault::InvalidPosition;
    if (!std::isfinite(t.age) || t.age < 0 || t.age > limits_.stale_seconds)
      return Fault::StaleTelemetry;
    if (!std::isfinite(t.battery) || t.battery < limits_.min_battery || t.battery > 1)
      return Fault::LowBattery;
    if (!safe_point(t.position))
      return Fault::Geofence;
    return Fault::None;
  }
  Fault command(Enu p) const { return safe_point(p) ? Fault::None : Fault::InvalidCommand; }
};
} // namespace uav
