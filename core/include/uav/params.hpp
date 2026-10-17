#pragma once
// Parameters of the PX4 adapter node (config/*.yaml) and their translation to MissionConfig.
// Shared by the ROS node and the scenario runner so both test and fly the same mission.
#include "uav/mission_controller.hpp"
#include <istream>
#include <string>
#include <vector>
namespace uav {

struct NodeParams {
  double max_altitude = 15, geofence_radius = 40, max_speed = 2, stale_seconds = 2,
         min_battery = .2, mission_deadline = 90, leg_timeout = 30, tracking_limit = 3,
         tracking_seconds = 1, takeoff_height = 5;
  // Optional ENU triples flown after the vertical takeoff waypoint.
  std::vector<double> waypoints_enu;

  MissionConfig mission() const {
    MissionConfig c;
    c.limits.max_altitude = max_altitude;
    c.limits.radius = geofence_radius;
    c.limits.max_speed = max_speed;
    c.limits.stale_seconds = stale_seconds;
    c.limits.min_battery = min_battery;
    c.deadline = mission_deadline;
    c.leg_timeout = leg_timeout;
    c.tracking_limit = tracking_limit;
    c.tracking_seconds = tracking_seconds;
    if (waypoints_enu.size() % 3 != 0 || waypoints_enu.size() > 189)
      throw std::invalid_argument("waypoints_enu requires up to 63 ENU triples");
    c.waypoints = {{0, 0, takeoff_height}};
    for (size_t i = 0; i < waypoints_enu.size(); i += 3)
      c.waypoints.push_back({waypoints_enu[i], waypoints_enu[i + 1], waypoints_enu[i + 2]});
    c.validate();
    return c;
  }
};

namespace detail {
inline std::string trim(const std::string &s) {
  auto b = s.find_first_not_of(" \t\r");
  auto e = s.find_last_not_of(" \t\r");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}
inline double number(const std::string &key, const std::string &text) {
  auto t = trim(text);
  size_t used = 0;
  double v = 0;
  try {
    v = std::stod(t, &used);
  } catch (const std::exception &) {
    used = 0;
  }
  if (t.empty() || used != t.size() || !std::isfinite(v))
    throw std::invalid_argument("parameter " + key + ": expected a number, got '" + t + "'");
  return v;
}
} // namespace detail

// Reads the flat ros__parameters subset used in config/*.yaml: `key: number` and
// `key: [n, n, ...]`, comments and section headers. Unknown keys are errors, not ignored.
inline NodeParams parse_params(std::istream &in) {
  NodeParams p;
  std::string line;
  while (std::getline(in, line)) {
    line = detail::trim(line.substr(0, line.find('#')));
    auto colon = line.find(':');
    if (line.empty() || colon == std::string::npos)
      continue;
    auto key = detail::trim(line.substr(0, colon));
    auto value = detail::trim(line.substr(colon + 1));
    if (value.empty())
      continue; // section header such as `sitl_mission:` or `ros__parameters:`
    if (key == "waypoints_enu") {
      while (value.find(']') == std::string::npos && std::getline(in, line))
        value += " " + detail::trim(line.substr(0, line.find('#')));
      if (value.front() != '[' || value.back() != ']')
        throw std::invalid_argument("parameter waypoints_enu: expected [n, n, ...]");
      auto body = value.substr(1, value.size() - 2);
      p.waypoints_enu.clear();
      for (size_t start = 0; start <= body.size();) {
        auto comma = body.find(',', start);
        auto item = detail::trim(body.substr(start, comma - start));
        if (!item.empty())
          p.waypoints_enu.push_back(detail::number(key, item));
        else if (comma != std::string::npos)
          throw std::invalid_argument("parameter waypoints_enu: empty list item");
        if (comma == std::string::npos)
          break;
        start = comma + 1;
      }
      continue;
    }
    double *field = key == "max_altitude"       ? &p.max_altitude
                    : key == "geofence_radius"  ? &p.geofence_radius
                    : key == "max_speed"        ? &p.max_speed
                    : key == "stale_seconds"    ? &p.stale_seconds
                    : key == "min_battery"      ? &p.min_battery
                    : key == "mission_deadline" ? &p.mission_deadline
                    : key == "leg_timeout"      ? &p.leg_timeout
                    : key == "tracking_limit"   ? &p.tracking_limit
                    : key == "tracking_seconds" ? &p.tracking_seconds
                    : key == "takeoff_height"   ? &p.takeoff_height
                                                : nullptr;
    if (!field)
      throw std::invalid_argument("unknown parameter: " + key);
    *field = detail::number(key, value);
  }
  return p;
}
} // namespace uav
