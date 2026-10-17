#!/usr/bin/env bash
# PX4 SITL acceptance for the shared MissionController, headless and CI-friendly.
# Runs PX4 v1.16 with its built-in SIH quadrotor (no Gazebo, no GUI), the Micro XRCE-DDS agent and
# the ROS 2 adapter node, then flies three missions and checks each terminal outcome:
#   smoke     config/controller.yaml        -> COMPLETE
#   waypoint  config/waypoint-mission.yaml  -> COMPLETE
#   abort     waypoint route, ~/abort called in NAVIGATION -> ABORTED_LANDED
# Usage: PX4_DIR=/path/to/PX4-Autopilot bash scripts/sitl/ci_acceptance.sh [out_dir]
# Needs: built px4_sitl_default, MicroXRCEAgent on PATH, ros2_ws built (uav_controller + px4_msgs).
# Simulation only: never point this at a physical autopilot.
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${PX4_DIR:?Set PX4_DIR to a PX4-Autopilot checkout with px4_sitl_default built}"
OUT=$(mkdir -p "${1:-artifacts/sitl}" && cd "${1:-artifacts/sitl}" && pwd)
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin
# The runner is a throwaway, isolated VM. ROS_LOCALHOST_ONLY=1 restricts ROS discovery to loopback, which the
# Micro XRCE-DDS agent's own participant does not follow, so PX4 topics never appear; use full discovery here.
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 UAV_SIMULATION_ONLY=1
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

pids=()
diagnose() {
  echo "::group::SITL diagnostics"
  echo "--- agent.log (tail)"; tail -n 40 "$OUT/agent.log" 2>/dev/null || true
  echo "--- px4.log (DDS, commander, errors)"
  grep -iE "uxrce|dds|ready|arm|preflight|error|warn" "$OUT/px4.log" 2>/dev/null | tail -n 60 || true
  echo "--- px4.log (tail)"; tail -n 30 "$OUT/px4.log" 2>/dev/null || true
  echo "--- ros2 topic list (ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY, domain $ROS_DOMAIN_ID)"
  timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1 | head -n 40 || true
  for f in "$OUT"/*.log; do
    case $f in */agent.log | */px4.log) continue ;; esac
    echo "--- $(basename "$f") (tail)"; tail -n 40 "$f"
  done
  echo "::endgroup::"
}
cleanup() {
  local status=$?
  ((status == 0)) || diagnose
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  pkill -f sitl_mission 2>/dev/null || true
  wait 2>/dev/null || true
  # Flight logs are the acceptance evidence; keep them even when a mission fails.
  cp -r "$PX4_DIR/build/px4_sitl_default/rootfs/log" "$OUT/px4-ulog" 2>/dev/null || true
}
trap cleanup EXIT

wait_for() { # wait_for <seconds> <description> <command...>
  local limit=$1 what=$2
  shift 2
  for ((i = 0; i < limit; i++)); do
    if "$@" >/dev/null 2>&1; then return 0; fi
    sleep 1
  done
  echo "timed out after ${limit}s waiting for $what" >&2
  return 1
}

echo "== Starting Micro XRCE-DDS agent"
MicroXRCEAgent udp4 -p 8888 >"$OUT/agent.log" 2>&1 &
pids+=($!)

echo "== Starting PX4 SITL (SIH quadrotor, airframe 10040, daemon mode)"
(cd "$PX4_DIR" && PX4_SYS_AUTOSTART=10040 exec ./build/px4_sitl_default/bin/px4 -d) >"$OUT/px4.log" 2>&1 &
pids+=($!)
wait_for 180 "PX4 'Ready for takeoff'" grep -q "Ready for takeoff" "$OUT/px4.log"

echo "== Applying px4/configuration/sitl-checklist.md parameters"
# COM_RC_IN_MODE 4 (no RC) is a CI-only deviation from the checklist, which keeps QGroundControl connected.
for kv in "UXRCE_DDS_DOM_ID 42" "COM_OF_LOSS_T 1" "COM_OBL_RC_ACT 4" "COM_RC_IN_MODE 4"; do
  "$PX4_BIN/px4-param" set $kv
done
"$PX4_BIN/px4-param" show UXRCE_DDS_DOM_ID COM_OF_LOSS_T COM_OBL_RC_ACT COM_RC_IN_MODE | tee "$OUT/params.txt"
"$PX4_BIN/px4-uxrce_dds_client" stop || true
"$PX4_BIN/px4-uxrce_dds_client" start -t udp -h 127.0.0.1 -p 8888
wait_for 60 "PX4 topics on ROS domain 42" bash -c "ros2 topic list --no-daemon --spin-time 3 | grep -q /fmu/out/vehicle_status"
ros2 topic list --no-daemon --spin-time 3 | grep /fmu/ | tee "$OUT/topics.txt"

declare -A outcome_name=([1]=COMPLETE [2]=ABORTED_LANDED [3]=FAILED)
results=()
failed=0

fly() { # fly <name> <config> <expected outcome> [state that triggers ~/abort]
  local name=$1 config=$2 expect=$3 abort_in=${4:-} log="$OUT/$1.log" aborted=0 code=""
  echo "== Mission $name ($config), expecting $expect"
  wait_for 60 "vehicle landed and disarmed before $name" bash -c \
    "ros2 topic echo --once --qos-reliability best_effort \$(ros2 topic list --no-daemon --spin-time 3 | grep /fmu/out/vehicle_status | head -1) \
     | grep -q 'arming_state: 1'"
  bash scripts/run_controller.sh "$config" >"$log" 2>&1 &
  local pid=$!
  for ((t = 0; t < 180; t++)); do
    code=$(grep -o 'outcome=[123]' "$log" | head -1 | cut -d= -f2 || true)
    [[ -n $code ]] && break
    if [[ -n $abort_in && $aborted == 0 ]] && grep -q "STATE $abort_in " "$log"; then
      ros2 service call /sitl_mission/abort std_srvs/srv/Trigger | tee -a "$OUT/$name-abort.txt"
      aborted=1
    fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  kill -INT "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  pkill -f sitl_mission 2>/dev/null || true
  local got=${outcome_name[${code:-0}]:-NO_TERMINAL_OUTCOME}
  local reason
  reason=$(grep -o 'reason=[A-Z_]*' "$log" | tail -1 | cut -d= -f2 || true)
  if [[ $got == "$expect" ]]; then
    results+=("| $name | $expect | $got | ${reason:-—} | PASS |")
  else
    results+=("| $name | $expect | $got | ${reason:-—} | **FAIL** |")
    failed=1
  fi
  grep -E "STATE|REQUEST|ACK" "$log" | tail -40 || true
}

fly smoke config/controller.yaml COMPLETE
fly waypoint config/waypoint-mission.yaml COMPLETE
fly abort config/waypoint-mission.yaml ABORTED_LANDED NAVIGATION

{
  echo "## PX4 SITL acceptance (SIH quadrotor, PX4 $(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo unknown))"
  echo
  echo "| Mission | Expected | Observed | Last reason | Result |"
  echo "|---|---|---|---|---|"
  printf '%s\n' "${results[@]}"
  echo
  echo "Controller logs, PX4 console, agent log and ULogs are in the sitl-evidence artifact."
} | tee "$OUT/summary.md"
exit "$failed"
