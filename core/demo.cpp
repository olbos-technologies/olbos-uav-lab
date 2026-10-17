#include "uav/core.hpp"
#include <fstream>
#include <iostream>
#include <string>
int main(int argc,char** argv){
 std::string fault=argc>1?argv[1]:"normal";
 if(fault!="normal"&&fault!="battery"&&fault!="stale"&&fault!="geofence")return 2;
 uav::SafetySupervisor safety;uav::Enu p{};
 std::array<uav::Enu,4> route{{{0,0,5},{10,0,5},{10,10,5},{0,0,5}}};
 size_t waypoint=0; bool landing=false,aborted=false;
 for(int i=0;i<2400;++i){
  double t=i*.05;uav::Telemetry telemetry{p,.95-t*.0008,.05,true};
  if(i>=120&&fault=="battery")telemetry.battery=.1;
  if(i>=120&&fault=="stale")telemetry.age=2;
  if(i>=120&&fault=="geofence")telemetry.position.east=50;
  auto f=safety.evaluate(telemetry);
  if(f!=uav::Fault::None){aborted=true;landing=true;}
  auto target=landing?uav::Enu{p.east,p.north,0}:route[waypoint];
  if(safety.command(target)!=uav::Fault::None)return 3;
  p=uav::step_toward(p,target,.05,2);
  if(!landing&&uav::distance(p,target)<.1){if(++waypoint==route.size())landing=true;}
  bool done=landing&&p.up<.01;
  std::cout<<"{\"source\":\"KINEMATIC_HARNESS\",\"time_s\":"<<t<<",\"east\":"<<p.east
   <<",\"north\":"<<p.north<<",\"altitude\":"<<p.up<<",\"battery\":"<<telemetry.battery
   <<",\"mode\":\""<<(done?(aborted?"ABORTED":"COMPLETE"):(landing?"LANDING":"NAVIGATION"))
   <<"\",\"alert\":\""<<uav::name(f)<<"\",\"waypoint\":"<<std::min(waypoint,route.size())<<"}\n";
  if(done)return 0;
 }
 return 4;
}
