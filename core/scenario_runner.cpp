// Runs the fault-injection catalog and envelope sweeps; writes a machine-readable safety report.
// Usage: uav_scenarios [--config params.yaml] [--out dir] [--no-sweeps] [scenario ...]
// Exit status 1 if any scenario fails or any sweep point breaks a safety invariant.
#include "uav/params.hpp"
#include "uav/simulation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>

namespace {
using uav::sim::detail::fixed;
std::string quote(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out + "\"";
}
std::string number(double v) { return v < 0 ? "null" : fixed(v); }
const char *boolean(bool b) { return b ? "true" : "false"; }

std::string metrics(const uav::sim::Result &r) {
  return ",\"outcome\":" + quote(std::string(uav::name(r.outcome))) +
         ",\"reason\":" + quote(r.reason) + ",\"first_alert\":" + quote(r.first_alert) +
         ",\"alert_at_s\":" + number(r.alert_at) +
         ",\"detect_latency_s\":" + number(r.detect_latency()) +
         ",\"duration_s\":" + number(r.duration) + ",\"max_altitude_m\":" + number(r.max_altitude) +
         ",\"max_distance_m\":" + number(r.max_distance) +
         ",\"geofence_m\":" + number(r.scenario.config.limits.radius) +
         ",\"min_battery\":" + number(r.min_battery) +
         ",\"touchdown_speed_mps\":" + number(r.touchdown_speed) +
         ",\"estimate_error_m\":" + number(r.estimate_error) +
         ",\"land_requests\":" + std::to_string(r.land_requests) +
         ",\"truth_landed\":" + boolean(r.truth_landed) +
         ",\"truth_armed\":" + boolean(r.truth_armed) + ",\"ever_armed\":" + boolean(r.ever_armed);
}

std::string checks(const uav::sim::Result &r) {
  std::string out = ",\"checks\":[";
  for (size_t i = 0; i < r.checks.size(); ++i)
    out += std::string(i ? "," : "") + "{\"name\":" + quote(r.checks[i].name) +
           ",\"pass\":" + boolean(r.checks[i].pass) + ",\"detail\":" + quote(r.checks[i].detail) +
           "}";
  return out + "]";
}

int usage() {
  std::cerr
      << "usage: uav_scenarios [--config params.yaml] [--out dir] [--no-sweeps] [scenario ...]\n";
  return 2;
}
} // namespace

int main(int argc, char **argv) {
  namespace fs = std::filesystem;
  fs::path out = "artifacts/scenarios";
  std::string config_file;
  bool run_sweeps = true;
  std::set<std::string> only;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc)
      config_file = argv[++i];
    else if (a == "--out" && i + 1 < argc)
      out = argv[++i];
    else if (a == "--no-sweeps")
      run_sweeps = false;
    else if (a.rfind("--", 0) == 0)
      return usage();
    else
      only.insert(a);
  }

  uav::MissionConfig config;
  try {
    if (!config_file.empty()) {
      std::ifstream in(config_file);
      if (!in)
        throw std::invalid_argument("cannot read " + config_file);
      config = uav::parse_params(in).mission();
    }
    config.validate();
  } catch (const std::exception &e) {
    std::cerr << "invalid mission configuration: " << e.what() << '\n';
    return 2;
  }

  std::vector<uav::sim::Scenario> catalog;
  try {
    catalog = uav::sim::catalog(config);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }

  fs::create_directories(out / "traces");
  std::ofstream report(out / "safety-report.json");
  const auto &lim = config.limits;
  report << "{\"generator\":\"uav_scenarios\",\"model\":\"kinematic plant, not PX4 physics\","
         << "\"config\":{\"source\":"
         << quote(config_file.empty() ? "built-in square route" : config_file)
         << ",\"waypoints\":" << config.waypoints.size()
         << ",\"max_altitude_m\":" << number(lim.max_altitude)
         << ",\"geofence_m\":" << number(lim.radius)
         << ",\"max_speed_mps\":" << number(lim.max_speed)
         << ",\"stale_seconds\":" << number(lim.stale_seconds)
         << ",\"min_battery\":" << number(lim.min_battery)
         << ",\"tracking_limit_m\":" << number(config.tracking_limit)
         << ",\"tracking_seconds\":" << number(config.tracking_seconds)
         << ",\"fault_time_s\":" << number(uav::sim::mid_flight(config)) << "},\"scenarios\":[";
  unsigned run = 0, failed = 0;
  for (const auto &s : catalog) {
    if (!only.empty() && !only.count(s.name))
      continue;
    auto r = uav::sim::run(s, true);
    std::ofstream trace(out / "traces" / (s.name + ".jsonl"));
    for (const auto &line : r.trace)
      trace << line << '\n';
    report << (run++ ? "," : "") << "{\"name\":" << quote(s.name)
           << ",\"category\":" << quote(s.category) << ",\"description\":" << quote(s.description)
           << ",\"pass\":" << boolean(r.pass()) << metrics(r) << checks(r) << "}";
    failed += !r.pass();
    std::cout << (r.pass() ? "PASS  " : "FAIL  ") << s.name << "  " << uav::name(r.outcome)
              << (r.first_alert.empty() ? "" : "  " + r.first_alert) << '\n';
    for (const auto &c : r.checks)
      if (!c.pass)
        std::cout << "      x " << c.name << ": " << c.detail << '\n';
  }
  report << "],\"sweeps\":[";
  unsigned unsafe = 0;
  if (run_sweeps && only.empty()) {
    auto all = uav::sim::sweeps(config);
    for (size_t i = 0; i < all.size(); ++i) {
      const auto &sw = all[i];
      report << (i ? "," : "") << "{\"name\":" << quote(sw.name)
             << ",\"parameter\":" << quote(sw.parameter) << ",\"unit\":" << quote(sw.unit)
             << ",\"description\":" << quote(sw.description) << ",\"points\":[";
      unsigned bad = 0;
      for (size_t j = 0; j < sw.results.size(); ++j) {
        const auto &r = sw.results[j];
        bad += !r.pass();
        report << (j ? "," : "") << "{\"value\":" << fixed(sw.values[j], 2)
               << ",\"safe\":" << boolean(r.pass()) << metrics(r) << checks(r) << "}";
        for (const auto &c : r.checks)
          if (!c.pass)
            std::cout << "UNSAFE " << sw.name << "=" << fixed(sw.values[j]) << "  " << c.name
                      << ": " << c.detail << '\n';
      }
      report << "]}";
      unsafe += bad;
      std::cout << (bad ? "UNSAFE " : "SWEEP  ") << sw.name << "  " << sw.results.size() - bad
                << "/" << sw.results.size() << " points within safety invariants\n";
    }
  }
  report << "],\"passed\":" << run - failed << ",\"failed\":" << failed
         << ",\"unsafe_sweep_points\":" << unsafe << "}\n";
  if (!run) {
    std::cerr << "no matching scenarios\n";
    return 2;
  }
  std::cout << run - failed << "/" << run
            << " scenarios passed. Report: " << (out / "safety-report.json").string() << '\n';
  return failed || unsafe ? 1 : 0;
}
