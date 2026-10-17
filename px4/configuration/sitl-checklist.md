# SITL preflight gate
Use an isolated VM with no serial/USB flight hardware and no external MAVLink forwarding.
Launch one x500 using scripts/launch_sitl.sh. Keep QGroundControl connected.
In the **SITL PX4 shell only**, set and verify:
```
param set UXRCE_DDS_DOM_ID 42
param set COM_OF_LOSS_T 1
param set COM_OBL_RC_ACT 4
param show UXRCE_DDS_DOM_ID
param show COM_OF_LOSS_T
param show COM_OBL_RC_ACT
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```
For v1.16 COM_OBL_RC_ACT=4 requests Land. Verify actual parameter descriptions against the checked-out release before running.
Do not disable arming checks. If offboard/arm is rejected, inspect QGC and controller logs.
DDS domain and environment guards prevent accidental defaults; they do not authenticate a simulator.
Network/VM isolation is the simulation boundary. Never launch against a physical autopilot.
