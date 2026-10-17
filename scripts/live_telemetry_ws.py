"""ROS 2 telemetry -> bounded latest snapshot -> loopback read-only WebSocket.
Run with ROS 2 and px4_msgs sourced. No command subscriptions or publishers.
"""
import asyncio,json,math,threading,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition,VehicleStatus,BatteryStatus
from websockets.asyncio.server import serve
def topic(cls,base):
 version=getattr(cls,'MESSAGE_VERSION',0)
 return base+(f'_v{version}' if version else '')
class Monitor(Node):
 def __init__(self):
  super().__init__('telemetry_monitor');self.lock=threading.Lock();self.data={'source':'PX4_SITL_UNVERIFIED','time_s':0};self.seen={}
  self.subs=[self.create_subscription(VehicleLocalPosition,topic(VehicleLocalPosition,'/fmu/out/vehicle_local_position'),self.position,qos_profile_sensor_data),
   self.create_subscription(VehicleStatus,topic(VehicleStatus,'/fmu/out/vehicle_status'),self.status,qos_profile_sensor_data),
   self.create_subscription(BatteryStatus,topic(BatteryStatus,'/fmu/out/battery_status'),self.battery,qos_profile_sensor_data)]
 def update(self,key,values):
  with self.lock:self.data.update(values);self.seen[key]=time.monotonic()
 def position(self,m):self.update('position',{'east':m.y,'north':m.x,'altitude':-m.z,'position_valid':bool(m.xy_valid and m.z_valid)})
 def status(self,m):self.update('status',{'mode':str(m.nav_state),'armed':m.arming_state==VehicleStatus.ARMING_STATE_ARMED})
 def battery(self,m):self.update('battery',{'battery':m.remaining if m.connected else None})
 def snapshot(self):
  with self.lock:
   d=dict(self.data);d['age_s']=max([time.monotonic()-self.seen.get(k,0) for k in ['position','status','battery']]);d['time_s']=time.monotonic()
  d.update(transport='LIVE',sent_at=time.time(),alert='STALE_TELEMETRY' if d['age_s']>2 else 'UNASSESSED')
  return {k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in d.items()}
async def main(node):
 async def client(ws):
  while True:
   await asyncio.wait_for(ws.send(json.dumps(node.snapshot(),allow_nan=False)),2);await asyncio.sleep(.1)
 async with serve(client,'127.0.0.1',8765,max_size=4096,max_queue=4,write_limit=4096):await asyncio.Future()
if __name__=='__main__':
 rclpy.init();node=Monitor();thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
 try:asyncio.run(main(node))
 finally:rclpy.shutdown();thread.join(timeout=2)
