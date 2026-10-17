// Runs the fault-injection catalog and writes a machine-readable safety report.
// Usage: uav_scenarios [output_dir] [scenario ...]   Exit status 1 if any scenario fails.
#include "uav/simulation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>

namespace {
std::string quote(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out + "\"";
}
std::string number(double v) { return v < 0 ? "null" : uav::sim::detail::fixed(v); }
} // namespace

int main(int argc, char **argv) {
  namespace fs = std::filesystem;
  fs::path out = argc > 1 ? argv[1] : "artifacts/scenarios";
  std::set<std::string> only(argv + std::min(argc, 2), argv + argc);
  fs::create_directories(out / "traces");
  std::ofstream report(out / "safety-report.json");
  report << "{\"generator\":\"uav_scenarios\",\"model\":\"kinematic plant, not PX4 physics\","
         << "\"scenarios\":[";
  unsigned run = 0, failed = 0;
  for (const auto &s : uav::sim::catalog()) {
    if (!only.empty() && !only.count(s.name))
      continue;
    auto r = uav::sim::run(s, true);
    std::ofstream trace(out / "traces" / (s.name + ".jsonl"));
    for (const auto &line : r.trace)
      trace << line << '\n';
    report << (run++ ? "," : "") << "{\"name\":" << quote(s.name)
           << ",\"category\":" << quote(s.category) << ",\"description\":" << quote(s.description)
           << ",\"pass\":" << (r.pass() ? "true" : "false")
           << ",\"outcome\":" << quote(std::string(uav::name(r.outcome)))
           << ",\"reason\":" << quote(r.reason) << ",\"first_alert\":" << quote(r.first_alert)
           << ",\"alert_at_s\":" << number(r.alert_at)
           << ",\"detect_latency_s\":" << number(r.detect_latency())
           << ",\"duration_s\":" << number(r.duration)
           << ",\"max_altitude_m\":" << number(r.max_altitude)
           << ",\"max_distance_m\":" << number(r.max_distance)
           << ",\"geofence_m\":" << number(s.config.limits.radius)
           << ",\"min_battery\":" << number(r.min_battery)
           << ",\"touchdown_speed_mps\":" << number(r.touchdown_speed)
           << ",\"land_requests\":" << r.land_requests
           << ",\"truth_landed\":" << (r.truth_landed ? "true" : "false")
           << ",\"truth_armed\":" << (r.truth_armed ? "true" : "false")
           << ",\"ever_armed\":" << (r.ever_armed ? "true" : "false") << ",\"checks\":[";
    for (size_t i = 0; i < r.checks.size(); ++i)
      report << (i ? "," : "") << "{\"name\":" << quote(r.checks[i].name)
             << ",\"pass\":" << (r.checks[i].pass ? "true" : "false")
             << ",\"detail\":" << quote(r.checks[i].detail) << "}";
    report << "]}";
    failed += !r.pass();
    std::cout << (r.pass() ? "PASS  " : "FAIL  ") << s.name << "  " << uav::name(r.outcome)
              << (r.first_alert.empty() ? "" : "  " + r.first_alert) << '\n';
    for (const auto &c : r.checks)
      if (!c.pass)
        std::cout << "      x " << c.name << ": " << c.detail << '\n';
  }
  report << "],\"passed\":" << run - failed << ",\"failed\":" << failed << "}\n";
  if (!run) {
    std::cerr << "no matching scenarios\n";
    return 2;
  }
  std::cout << run - failed << "/" << run
            << " scenarios passed. Report: " << (out / "safety-report.json").string() << '\n';
  return failed ? 1 : 0;
}
