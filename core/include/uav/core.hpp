#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>
namespace uav {
struct Enu { double east{}, north{}, up{}; };
struct Ned { double north{}, east{}, down{}; };
inline Ned to_ned(Enu p) { return {p.north,p.east,-p.up}; }
inline Enu to_enu(Ned p) { return {p.east,p.north,-p.down}; }
inline bool finite(Enu p) { return std::isfinite(p.east)&&std::isfinite(p.north)&&std::isfinite(p.up); }
inline double distance(Enu a, Enu b) { return std::hypot(std::hypot(a.east-b.east,a.north-b.north),a.up-b.up); }
struct Limits {
 double max_altitude=15, radius=40, max_speed=2, arrival_radius=0.5, min_battery=0.2, stale_seconds=1;
 void validate() const {
  for(double x : {max_altitude,radius,max_speed,arrival_radius,min_battery,stale_seconds})
   if(!std::isfinite(x)||x<=0) throw std::invalid_argument("invalid limit");
  if(min_battery>=1||arrival_radius>=radius) throw std::invalid_argument("inconsistent limits");
 }
};
struct Telemetry { Enu position; double battery=0, age=1e9; bool position_valid=false; };
enum class Fault { None, InvalidPosition, StaleTelemetry, LowBattery, Geofence, InvalidCommand, MlTimeout };
inline std::string_view name(Fault f) {
 switch(f) {case Fault::None:return "NORMAL";case Fault::InvalidPosition:return "INVALID_POSITION";
 case Fault::StaleTelemetry:return "STALE_TELEMETRY";case Fault::LowBattery:return "LOW_BATTERY";
 case Fault::Geofence:return "GEOFENCE";case Fault::InvalidCommand:return "INVALID_COMMAND";
 case Fault::MlTimeout:return "ML_TIMEOUT";} return "UNKNOWN";
}
class SafetySupervisor {
 Limits limits_;
 public:
 explicit SafetySupervisor(Limits limits={}) : limits_(limits) { limits_.validate(); }
 bool safe_point(Enu p) const {
  return finite(p)&&p.up>=-0.5&&p.up<=limits_.max_altitude&&std::hypot(p.east,p.north)<=limits_.radius;
 }
 Fault evaluate(Telemetry t) const {
  if(!t.position_valid||!finite(t.position)) return Fault::InvalidPosition;
  if(!std::isfinite(t.age)||t.age<0||t.age>limits_.stale_seconds) return Fault::StaleTelemetry;
  if(!std::isfinite(t.battery)||t.battery<limits_.min_battery||t.battery>1) return Fault::LowBattery;
  if(!safe_point(t.position)) return Fault::Geofence;
  return Fault::None;
 }
 Fault command(Enu p) const {return safe_point(p)?Fault::None:Fault::InvalidCommand;}
};
inline Enu step_toward(Enu from, Enu to, double dt, double speed) {
 if(!finite(from)||!finite(to)||!std::isfinite(dt)||!std::isfinite(speed)||dt<=0||dt>0.5||speed<=0)
  throw std::invalid_argument("invalid trajectory input");
 double d=distance(from,to); double k=d>0?std::min(1.0,speed*dt/d):0;
 return {from.east+k*(to.east-from.east),from.north+k*(to.north-from.north),from.up+k*(to.up-from.up)};
}
enum class State { Idle, PreFlight, Ready, Takeoff, Navigation, Landing, Complete, Abort, Emergency, ReturnToLaunch, CommunicationLoss };
class MissionManager {
 State state_=State::Idle;
 public:
 State state()const{return state_;}
 bool transition(State next,bool valid) {
  if(!valid||state_==State::Complete) return false;
  bool exceptional=next==State::Abort||next==State::Emergency||next==State::CommunicationLoss||next==State::ReturnToLaunch;
  bool normal=(state_==State::Idle&&next==State::PreFlight)||(state_==State::PreFlight&&next==State::Ready)||
   (state_==State::Ready&&next==State::Takeoff)||(state_==State::Takeoff&&next==State::Navigation)||
   (state_==State::Navigation&&next==State::Landing)||(state_==State::Landing&&next==State::Complete)||
   ((state_==State::Abort||state_==State::Emergency||state_==State::ReturnToLaunch||state_==State::CommunicationLoss)&&next==State::Landing);
  if(!exceptional&&!normal)return false;
  state_=next;return true;
 }
};
} // namespace uav
