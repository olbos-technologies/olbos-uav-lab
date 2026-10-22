# Olbos UAV: Shadow Operations — Game prototype

Standalone arcade-style **fictional** 3D drone game. Uses Three.js and does **not** connect to PX4, ROS, MAVLink, GNSS, flight hardware or weapon systems.

## Run
```bash
cd game
npm install
npm run dev
```

## Test / release
```bash
npm test
npm run build
```

Controls: WASD move; R/Q up/down; Shift boost; E scan/deliver; C camera; P pause; Enter restart. Scan three blue beacons below altitude 20; travel to the green supply pad below altitude 7 and press E. Red patrols raise the alert meter.

All coordinates and AI detection are abstract arcade mechanics; no real maps, combat, real-world tactics or exportable drone routes. This application is isolated from the robotics research modules. Closes #7 when accepted.
