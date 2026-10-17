#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>
namespace uav {
struct Enu {
  double east{}, north{}, up{};
};
struct Ned {
  double north{}, east{}, down{};
};
inline Ned to_ned(Enu p) { return {p.north, p.east, -p.up}; }
inline Enu to_enu(Ned p) { return {p.east, p.north, -p.down}; }
inline bool finite(Enu p) {
  return std::isfinite(p.east) && std::isfinite(p.north) && std::isfinite(p.up);
}
inline double distance(Enu a, Enu b) {
  return std::hypot(std::hypot(a.east - b.east, a.north - b.north), a.up - b.up);
}
struct Limits {
  double max_altitude = 15, radius = 40, max_speed = 2, arrival_radius = 0.5, min_battery = 0.2,
         stale_seconds = 1;
  void validate() const {
    for (double x : {max_altitude, radius, max_speed, arrival_radius, min_battery, stale_seconds})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("invalid limit");
    if (min_battery >= 1 || arrival_radius >= radius)
      throw std::invalid_argument("inconsistent limits");
  }
};
struct Telemetry {
  Enu position;
  double battery = 0, age = 1e9;
  bool position_valid = false;
};
enum class Fault {
  None,
  InvalidPosition,
  StaleTelemetry,
  LowBattery,
  Geofence,
  InvalidCommand,
  MlTimeout
};
inline std::string_view name(Fault f) {
  switch (f) {
  case Fault::None:
    return "NORMAL";
  case Fault::InvalidPosition:
    return "INVALID_POSITION";
  case Fault::StaleTelemetry:
    return "STALE_TELEMETRY";
  case Fault::LowBattery:
    return "LOW_BATTERY";
  case Fault::Geofence:
    return "GEOFENCE";
  case Fault::InvalidCommand:
    return "INVALID_COMMAND";
  case Fault::MlTimeout:
    return "ML_TIMEOUT";
  }
  return "UNKNOWN";
}
} // namespace uav
