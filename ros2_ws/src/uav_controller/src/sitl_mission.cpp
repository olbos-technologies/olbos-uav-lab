#include "uav/mission_controller.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using namespace px4_msgs::msg;
template <class Message> std::string topic(std::string base) {
  if constexpr (requires { Message::MESSAGE_VERSION; }) {
    if constexpr (Message::MESSAGE_VERSION > 0)
      base += "_v" + std::to_string(Message::MESSAGE_VERSION);
  }
  return base;
}
class SitlMission final : public rclcpp::Node {
  Clock::time_point started_ = Clock::now(), pos_at_{}, status_at_{}, battery_at_{}, land_at_{};
  Clock::time_point raw_pos_at_{}, raw_status_at_{}, raw_battery_at_{}, raw_land_at_{};
  uav::Observation observation_, raw_observation_;
  std::unique_ptr<uav::MissionController> mission_;
  uav::Decision last_;
  std::ofstream trace_;
  std::string fault_scenario_ = "none";
  double fault_after_seconds_ = 5.0, geofence_radius_ = 40.0;
  bool fault_applied_ = false, evidence_written_ = false;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr heartbeat_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoints_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr commands_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;
  double age(Clock::time_point p) const {
    return std::chrono::duration<double>(Clock::now() - p).count();
  }
  bool fault_due() const {
    return fault_scenario_ != "none" && !fault_applied_ && age(started_) >= fault_after_seconds_ &&
           raw_observation_.armed && !raw_observation_.landed;
  }
  void record_fault() {
    if (fault_applied_)
      return;
    fault_applied_ = true;
    RCLCPP_WARN(get_logger(),
                "INJECTED_FAULT scenario=%s source=ROS_ADAPTER_OBSERVATION raw_px4_untouched=true",
                fault_scenario_.c_str());
  }
  uint64_t stamp() { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }
  void command(uav::Request request) {
    VehicleCommand m{};
    m.timestamp = stamp();
    switch (request) {
    case uav::Request::Offboard:
      m.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
      m.param1 = 1;
      m.param2 = 6;
      break;
    case uav::Request::Arm:
      m.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      m.param1 = 1;
      break;
    case uav::Request::Land:
      m.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
      break;
    }
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 191;
    m.from_external = true;
    commands_->publish(m);
    RCLCPP_INFO(get_logger(), "REQUEST command=%u (await ACK and observed state)", m.command);
  }
  void stream() {
    OffboardControlMode h{};
    h.timestamp = stamp();
    h.position = true;
    heartbeat_->publish(h);
    auto n = uav::to_ned(last_.setpoint);
    TrajectorySetpoint p{};
    p.timestamp = h.timestamp;
    p.position = {static_cast<float>(n.north), static_cast<float>(n.east),
                  static_cast<float>(n.down)};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    p.velocity = {nan, nan, nan};
    p.acceleration = {nan, nan, nan};
    p.yaw = 0;
    p.yawspeed = nan;
    setpoints_->publish(p);
  }
  void tick() {
    observation_.telemetry.age =
        std::max({age(pos_at_), age(status_at_), age(battery_at_), age(land_at_)});
    raw_observation_.telemetry.age =
        std::max({age(raw_pos_at_), age(raw_status_at_), age(raw_battery_at_), age(raw_land_at_)});
    const auto previous = last_;
    last_ = mission_->tick(age(started_), observation_);
    if (last_.request)
      command(*last_.request);
    if (last_.stream)
      stream();
    if (previous.state != last_.state || previous.outcome != last_.outcome ||
        previous.reason != last_.reason)
      RCLCPP_INFO(get_logger(), "STATE %s outcome=%d reason=%s waypoint=%zu",
                  uav::name(last_.state).data(), static_cast<int>(last_.outcome),
                  last_.reason.data(), last_.waypoint);
    if (!evidence_written_ && last_.outcome != uav::Outcome::Running) {
      RCLCPP_INFO(get_logger(),
                  "EVIDENCE tag=GAZEBO_SITL outcome=%s reason=%s fault=%s fault_injected=%d "
                  "px4_armed=%d px4_landed=%d px4_position_valid=%d px4_position_east=%.3f "
                  "px4_position_north=%.3f px4_position_up=%.3f px4_telemetry_age=%.3f "
                  "controller_position_valid=%d controller_position_east=%.3f "
                  "controller_position_north=%.3f controller_position_up=%.3f "
                  "controller_telemetry_age=%.3f",
                  uav::name(last_.outcome).data(), last_.reason.data(), fault_scenario_.c_str(),
                  static_cast<int>(fault_applied_), static_cast<int>(raw_observation_.armed),
                  static_cast<int>(raw_observation_.landed),
                  static_cast<int>(raw_observation_.telemetry.position_valid),
                  raw_observation_.telemetry.position.east,
                  raw_observation_.telemetry.position.north, raw_observation_.telemetry.position.up,
                  raw_observation_.telemetry.age,
                  static_cast<int>(observation_.telemetry.position_valid),
                  observation_.telemetry.position.east, observation_.telemetry.position.north,
                  observation_.telemetry.position.up, observation_.telemetry.age);
      evidence_written_ = true;
    }
    if (trace_) {
      auto number = [this](double value) {
        if (std::isfinite(value))
          trace_ << value;
        else
          trace_ << "null";
      };
      trace_ << std::setprecision(9) << "{\"tag\":\"GAZEBO_SITL\",\"elapsed_s\":" << age(started_)
             << ",\"scenario\":\"" << fault_scenario_ << "\",\"state\":\"" << uav::name(last_.state)
             << "\",\"outcome\":\"" << uav::name(last_.outcome) << "\",\"reason\":\""
             << last_.reason << "\",\"stream\":" << (last_.stream ? "true" : "false")
             << ",\"px4_armed\":" << (raw_observation_.armed ? "true" : "false")
             << ",\"px4_landed\":" << (raw_observation_.landed ? "true" : "false")
             << ",\"px4_position_valid\":"
             << (raw_observation_.telemetry.position_valid ? "true" : "false")
             << ",\"px4_east_m\":";
      number(raw_observation_.telemetry.position.east);
      trace_ << ",\"px4_north_m\":";
      number(raw_observation_.telemetry.position.north);
      trace_ << ",\"px4_up_m\":";
      number(raw_observation_.telemetry.position.up);
      trace_ << ",\"controller_position_valid\":"
             << (observation_.telemetry.position_valid ? "true" : "false")
             << ",\"controller_east_m\":";
      number(observation_.telemetry.position.east);
      trace_ << ",\"controller_north_m\":";
      number(observation_.telemetry.position.north);
      trace_ << ",\"controller_up_m\":";
      number(observation_.telemetry.position.up);
      trace_ << ",\"controller_telemetry_age_s\":";
      number(observation_.telemetry.age);
      trace_ << ",\"setpoint_east_m\":";
      number(last_.setpoint.east);
      trace_ << ",\"setpoint_north_m\":";
      number(last_.setpoint.north);
      trace_ << ",\"setpoint_up_m\":";
      number(last_.setpoint.up);
      trace_ << "}\n";
      trace_.flush();
    }
  }

public:
  SitlMission() : Node("sitl_mission") {
    const char *guard = std::getenv("UAV_SIMULATION_ONLY");
    if (!guard || std::string_view(guard) != "1")
      throw std::runtime_error("use scripts/run_controller.sh in isolated SITL environment");
    uav::MissionConfig config;
    config.limits.max_altitude = declare_parameter("max_altitude", 15.0);
    config.limits.radius = declare_parameter("geofence_radius", 40.0);
    geofence_radius_ = config.limits.radius;
    config.limits.max_speed = declare_parameter("max_speed", 2.0);
    config.limits.stale_seconds = declare_parameter("stale_seconds", 2.0);
    config.limits.min_battery = declare_parameter("min_battery", .2);
    config.deadline = declare_parameter("mission_deadline", 90.0);
    config.leg_timeout = declare_parameter("leg_timeout", 30.0);
    config.tracking_limit = declare_parameter("tracking_limit", 3.0);
    config.tracking_seconds = declare_parameter("tracking_seconds", 1.0);
    fault_scenario_ = declare_parameter("fault_scenario", std::string("none"));
    fault_after_seconds_ = declare_parameter("fault_after_seconds", 5.0);
    if (fault_scenario_ != "none" && fault_scenario_ != "gnss_loss" &&
        fault_scenario_ != "position_jump" && fault_scenario_ != "stale_telemetry" &&
        fault_scenario_ != "geofence")
      throw std::invalid_argument("unsupported fault_scenario");
    if (!std::isfinite(fault_after_seconds_) || fault_after_seconds_ < 0)
      throw std::invalid_argument("fault_after_seconds must be finite and non-negative");
    if (const char *trace_path = std::getenv("UAV_SITL_TRACE_FILE")) {
      trace_.open(trace_path, std::ios::out | std::ios::trunc);
      if (!trace_)
        throw std::runtime_error("cannot open UAV_SITL_TRACE_FILE");
    }
    const auto height = declare_parameter("takeoff_height", 5.0);
    const auto route =
        declare_parameter<std::vector<double>>("waypoints_enu", std::vector<double>{});
    config.waypoints = {{0, 0, height}};
    if (route.size() % 3 != 0 || route.size() > 189)
      throw std::invalid_argument("waypoints_enu requires up to 63 ENU triples");
    for (size_t i = 0; i < route.size(); i += 3)
      config.waypoints.push_back({route[i], route[i + 1], route[i + 2]});
    mission_ = std::make_unique<uav::MissionController>(config);
    auto q = rclcpp::SensorDataQoS().keep_last(5);
    heartbeat_ = create_publisher<OffboardControlMode>(
        topic<OffboardControlMode>("/fmu/in/offboard_control_mode"), 10);
    setpoints_ = create_publisher<TrajectorySetpoint>(
        topic<TrajectorySetpoint>("/fmu/in/trajectory_setpoint"), 10);
    commands_ =
        create_publisher<VehicleCommand>(topic<VehicleCommand>("/fmu/in/vehicle_command"), 10);
    position_sub_ = create_subscription<VehicleLocalPosition>(
        topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), q,
        [this](VehicleLocalPosition::SharedPtr p) {
          const auto now = Clock::now();
          raw_observation_.telemetry.position = uav::to_enu({p->x, p->y, p->z});
          raw_observation_.telemetry.position_valid = p->xy_valid && p->z_valid;
          raw_pos_at_ = now;
          if (fault_scenario_ == "stale_telemetry" && fault_applied_)
            return;
          observation_.telemetry.position = raw_observation_.telemetry.position;
          observation_.telemetry.position_valid = raw_observation_.telemetry.position_valid;
          if (fault_due()) {
            if (fault_scenario_ == "stale_telemetry") {
              record_fault();
              return;
            }
            if (fault_scenario_ == "gnss_loss") {
              observation_.telemetry.position_valid = false;
            } else if (fault_scenario_ == "position_jump") {
              observation_.telemetry.position.east += 6.0;
            } else if (fault_scenario_ == "geofence") {
              observation_.telemetry.position.east += geofence_radius_ + 5.0;
            }
            record_fault();
          } else if (fault_applied_ && fault_scenario_ == "gnss_loss") {
            observation_.telemetry.position_valid = false;
          } else if (fault_applied_ && fault_scenario_ == "position_jump") {
            observation_.telemetry.position.east += 6.0;
          } else if (fault_applied_ && fault_scenario_ == "geofence") {
            observation_.telemetry.position.east += geofence_radius_ + 5.0;
          }
          pos_at_ = now;
        });
    status_sub_ = create_subscription<VehicleStatus>(
        topic<VehicleStatus>("/fmu/out/vehicle_status"), q, [this](VehicleStatus::SharedPtr p) {
          raw_observation_.armed = p->arming_state == VehicleStatus::ARMING_STATE_ARMED;
          raw_observation_.offboard = p->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          raw_observation_.landing_mode = p->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
          observation_.armed = raw_observation_.armed;
          observation_.offboard = raw_observation_.offboard;
          observation_.landing_mode = raw_observation_.landing_mode;
          status_at_ = raw_status_at_ = Clock::now();
        });
    battery_sub_ = create_subscription<BatteryStatus>(
        topic<BatteryStatus>("/fmu/out/battery_status"), q, [this](BatteryStatus::SharedPtr p) {
          raw_observation_.telemetry.battery = p->connected ? p->remaining : 0;
          observation_.telemetry.battery = raw_observation_.telemetry.battery;
          battery_at_ = raw_battery_at_ = Clock::now();
        });
    land_sub_ = create_subscription<VehicleLandDetected>(
        topic<VehicleLandDetected>("/fmu/out/vehicle_land_detected"), q,
        [this](VehicleLandDetected::SharedPtr p) {
          raw_observation_.landed = p->landed;
          observation_.landed = raw_observation_.landed;
          land_at_ = raw_land_at_ = Clock::now();
        });
    ack_sub_ = create_subscription<VehicleCommandAck>(
        topic<VehicleCommandAck>("/fmu/out/vehicle_command_ack"), q,
        [this](VehicleCommandAck::SharedPtr p) {
          if (p->target_system != 1 || p->target_component != 191)
            return;
          std::optional<uav::Request> request;
          if (p->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE)
            request = uav::Request::Offboard;
          if (p->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
            request = uav::Request::Arm;
          if (p->command == VehicleCommand::VEHICLE_CMD_NAV_LAND)
            request = uav::Request::Land;
          if (!request)
            return;
          RCLCPP_INFO(get_logger(), "ACK command=%u result=%u (vehicle state still authoritative)",
                      p->command, p->result);
          mission_->acknowledge(*request,
                                p->result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED ||
                                    p->result == VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS);
        });
    abort_service_ = create_service<std_srvs::srv::Trigger>(
        "~/abort", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (mission_->outcome() != uav::Outcome::Running) {
            response->success = false;
            response->message = "Mission is terminal";
            return;
          }
          mission_->request_abort();
          response->success = true;
          response->message =
              "Abort queued for next safety-supervised tick; PX4 landing unconfirmed";
        });
    timer_ = create_wall_timer(50ms, [this] { tick(); });
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SitlMission>());
  } catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
