#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include "uav/core.hpp"
#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
using namespace px4_msgs::msg;
template<class Message> std::string topic(std::string base){
 if constexpr(requires { Message::MESSAGE_VERSION; }) {
  if constexpr(Message::MESSAGE_VERSION>0)base+="_v"+std::to_string(Message::MESSAGE_VERSION);
 }
 return base;
}
class SitlMission final: public rclcpp::Node {
 enum class Stage{Waiting,Warmup,Mode,Arm,Climb,Land,Done,Fault};
 Stage stage_=Stage::Waiting;
 Clock::time_point entered_=Clock::now(),started_=entered_,pos_at_{},status_at_{},battery_at_{},land_at_{};
 uav::Telemetry telemetry_;
 uav::Limits limits_;
 std::unique_ptr<uav::SafetySupervisor> safety_;
 VehicleStatus status_{};
 bool landed_=false,ever_armed_=false,land_requested_=false;
 int requests_=0;
 Clock::time_point last_request_{};
 uav::Enu setpoint_{};
 double height_=5;
 uint16_t pending_=0;
 rclcpp::Publisher<OffboardControlMode>::SharedPtr heartbeat_;
 rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoints_;
 rclcpp::Publisher<VehicleCommand>::SharedPtr commands_;
 rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
 rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
 rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
 rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
 rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
 rclcpp::TimerBase::SharedPtr timer_;
 double age(Clock::time_point p)const{return std::chrono::duration<double>(Clock::now()-p).count();}
 uint64_t stamp()const{return static_cast<uint64_t>(get_clock()->now().nanoseconds()/1000);}
 bool armed()const{return status_.arming_state==VehicleStatus::ARMING_STATE_ARMED;}
 void enter(Stage s){stage_=s;entered_=Clock::now();requests_=0;pending_=0;}
 void command(uint16_t c,float p1=0,float p2=0){
  VehicleCommand m{};m.timestamp=stamp();m.command=c;m.param1=p1;m.param2=p2;
  m.target_system=1;m.target_component=1;m.source_system=1;m.source_component=191;m.from_external=true;
  commands_->publish(m);pending_=c;last_request_=Clock::now();++requests_;
  RCLCPP_INFO(get_logger(),"REQUEST command=%u attempt=%d",c,requests_);
 }
 void fail(const char* reason){
  RCLCPP_ERROR(get_logger(),"FAULT %s; stop offboard heartbeat; request LAND if armed; completion unverified",reason);
  if(armed()&&!land_requested_){command(VehicleCommand::VEHICLE_CMD_NAV_LAND);land_requested_=true;}
  enter(Stage::Fault);
 }
 void retry(uint16_t c,float p1=0,float p2=0){
  if(requests_==0||age(last_request_)>2){if(requests_>=3){fail("command/state timeout");return;}command(c,p1,p2);}
 }
 void stream(){
  OffboardControlMode h{};h.timestamp=stamp();h.position=true;heartbeat_->publish(h);
  auto n=uav::to_ned(setpoint_);TrajectorySetpoint p{};p.timestamp=h.timestamp;
  p.position={static_cast<float>(n.north),static_cast<float>(n.east),static_cast<float>(n.down)};
  const float nan=std::numeric_limits<float>::quiet_NaN();
  p.velocity={nan,nan,nan};p.acceleration={nan,nan,nan};p.yaw=0;p.yawspeed=nan;setpoints_->publish(p);
 }
 void tick(){
  if(stage_==Stage::Done||stage_==Stage::Fault)return;
  if(age(started_)>90){fail("mission deadline");return;}
  telemetry_.age=std::max({age(pos_at_),age(status_at_),age(battery_at_),age(land_at_)});
  if(stage_==Stage::Waiting){
   if(safety_->evaluate(telemetry_)==uav::Fault::None&&landed_&&!armed()){
    setpoint_=telemetry_.position;
    if(std::hypot(setpoint_.east,setpoint_.north)>2||std::abs(setpoint_.up)>1){fail("spawn must be at local origin");return;}
    enter(Stage::Warmup);
   }
   return;
  }
  if(stage_!=Stage::Land){
   auto f=safety_->evaluate(telemetry_);if(f!=uav::Fault::None){fail(uav::name(f).data());return;}
  }else if(telemetry_.age>limits_.stale_seconds){fail("landing telemetry stale");return;}
  if(stage_==Stage::Land){
   if(ever_armed_&&landed_&&!armed()){
    RCLCPP_INFO(get_logger(),"COMPLETE observed landed and disarmed");enter(Stage::Done);return;
   }
   if(status_.nav_state!=VehicleStatus::NAVIGATION_STATE_AUTO_LAND) retry(VehicleCommand::VEHICLE_CMD_NAV_LAND);
   if(age(entered_)>40)fail("landing deadline");
   return;
  }
  if(stage_==Stage::Climb){
   if(!armed()||status_.nav_state!=VehicleStatus::NAVIGATION_STATE_OFFBOARD){fail("unexpected mode/arming state");return;}
   setpoint_=uav::step_toward(setpoint_,{0,0,height_},.05,limits_.max_speed);
   if(safety_->command(setpoint_)!=uav::Fault::None){fail("invalid setpoint");return;}
   if(uav::distance(telemetry_.position,{0,0,height_})<limits_.arrival_radius&&age(entered_)>8){
    enter(Stage::Land);command(VehicleCommand::VEHICLE_CMD_NAV_LAND);return;
   }
   if(age(entered_)>30){fail("takeoff tracking timeout");return;}
  }
  stream();
  if(stage_==Stage::Warmup&&age(entered_)>2)enter(Stage::Mode);
  if(stage_==Stage::Mode){
   if(status_.nav_state==VehicleStatus::NAVIGATION_STATE_OFFBOARD)enter(Stage::Arm);
   else retry(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
  }
  if(stage_==Stage::Arm){
   if(armed()){ever_armed_=true;enter(Stage::Climb);}
   else retry(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
  }
 }
 public:
 SitlMission():Node("sitl_mission"){
  const char* guard=std::getenv("UAV_SIMULATION_ONLY");
  if(!guard||std::string_view(guard)!="1")throw std::runtime_error("use scripts/run_controller.sh in isolated SITL environment");
  limits_.max_altitude=declare_parameter("max_altitude",15.0);limits_.radius=declare_parameter("geofence_radius",40.0);
  limits_.max_speed=declare_parameter("max_speed",2.0);limits_.stale_seconds=declare_parameter("stale_seconds",1.0);
  limits_.min_battery=declare_parameter("min_battery",.2);height_=declare_parameter("takeoff_height",5.0);
  safety_=std::make_unique<uav::SafetySupervisor>(limits_);
  if(!std::isfinite(height_)||height_<1||height_>limits_.max_altitude)throw std::invalid_argument("invalid takeoff height");
  auto q=rclcpp::SensorDataQoS().keep_last(5);
  heartbeat_=create_publisher<OffboardControlMode>(topic<OffboardControlMode>("/fmu/in/offboard_control_mode"),10);
  setpoints_=create_publisher<TrajectorySetpoint>(topic<TrajectorySetpoint>("/fmu/in/trajectory_setpoint"),10);
  commands_=create_publisher<VehicleCommand>(topic<VehicleCommand>("/fmu/in/vehicle_command"),10);
  position_sub_=create_subscription<VehicleLocalPosition>(topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"),q,[this](VehicleLocalPosition::SharedPtr p){
   telemetry_.position=uav::to_enu({p->x,p->y,p->z});telemetry_.position_valid=p->xy_valid&&p->z_valid;pos_at_=Clock::now();});
  status_sub_=create_subscription<VehicleStatus>(topic<VehicleStatus>("/fmu/out/vehicle_status"),q,[this](VehicleStatus::SharedPtr p){status_=*p;status_at_=Clock::now();});
  battery_sub_=create_subscription<BatteryStatus>(topic<BatteryStatus>("/fmu/out/battery_status"),q,[this](BatteryStatus::SharedPtr p){telemetry_.battery=p->connected?p->remaining:0;battery_at_=Clock::now();});
  land_sub_=create_subscription<VehicleLandDetected>(topic<VehicleLandDetected>("/fmu/out/vehicle_land_detected"),q,[this](VehicleLandDetected::SharedPtr p){landed_=p->landed;land_at_=Clock::now();});
  ack_sub_=create_subscription<VehicleCommandAck>(topic<VehicleCommandAck>("/fmu/out/vehicle_command_ack"),q,[this](VehicleCommandAck::SharedPtr p){
   if(p->command!=pending_)return;
   RCLCPP_INFO(get_logger(),"ACK command=%u result=%u (vehicle state still authoritative)",p->command,p->result);
   if(p->result!=VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED&&p->result!=VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)fail("command rejected");
  });
  timer_=create_wall_timer(50ms,[this]{tick();});
 }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SitlMission>());}catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
