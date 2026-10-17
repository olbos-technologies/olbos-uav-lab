#include "uav/mission_controller.hpp"
#include <chrono>
#include <cstdlib>
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
  uav::Observation observation_;
  std::unique_ptr<uav::MissionController> mission_;
  uav::Decision last_;
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
  uint64_t stamp() const { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }
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
  }

public:
  SitlMission() : Node("sitl_mission") {
    const char *guard = std::getenv("UAV_SIMULATION_ONLY");
    if (!guard || std::string_view(guard) != "1")
      throw std::runtime_error("use scripts/run_controller.sh in isolated SITL environment");
    uav::MissionConfig config;
    config.limits.max_altitude = declare_parameter("max_altitude", 15.0);
    config.limits.radius = declare_parameter("geofence_radius", 40.0);
    config.limits.max_speed = declare_parameter("max_speed", 2.0);
    config.limits.stale_seconds = declare_parameter("stale_seconds", 2.0);
    config.limits.min_battery = declare_parameter("min_battery", .2);
    config.deadline = declare_parameter("mission_deadline", 90.0);
    config.leg_timeout = declare_parameter("leg_timeout", 30.0);
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
          observation_.telemetry.position = uav::to_enu({p->x, p->y, p->z});
          observation_.telemetry.position_valid = p->xy_valid && p->z_valid;
          pos_at_ = Clock::now();
        });
    status_sub_ = create_subscription<VehicleStatus>(
        topic<VehicleStatus>("/fmu/out/vehicle_status"), q, [this](VehicleStatus::SharedPtr p) {
          observation_.armed = p->arming_state == VehicleStatus::ARMING_STATE_ARMED;
          observation_.offboard = p->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          observation_.landing_mode = p->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
          status_at_ = Clock::now();
        });
    battery_sub_ = create_subscription<BatteryStatus>(
        topic<BatteryStatus>("/fmu/out/battery_status"), q, [this](BatteryStatus::SharedPtr p) {
          observation_.telemetry.battery = p->connected ? p->remaining : 0;
          battery_at_ = Clock::now();
        });
    land_sub_ = create_subscription<VehicleLandDetected>(
        topic<VehicleLandDetected>("/fmu/out/vehicle_land_detected"), q,
        [this](VehicleLandDetected::SharedPtr p) {
          observation_.landed = p->landed;
          land_at_ = Clock::now();
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
