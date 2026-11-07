#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${PX4_DIR:?Set PX4_DIR to a PX4 v1.16.0 checkout}"
test "$(git -C "$PX4_DIR" describe --tags --exact-match)" = v1.16.0 || {
  echo "PX4 v1.16.0 required" >&2
  exit 1
}
OUT=$(mkdir -p "${1:-artifacts/gazebo-sitl}" && cd "${1:-artifacts/gazebo-sitl}" && pwd)
mkdir -p "$OUT/runs" "$OUT/traces"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 UAV_SIMULATION_ONLY=1
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

{
  echo "claim_tag=GAZEBO_SITL"
  echo "simulator=gz_x500"
  echo "px4_ref=v1.16.0"
  echo "px4_commit=$(git -C "$PX4_DIR" rev-parse HEAD)"
  echo "px4_description=$(git -C "$PX4_DIR" describe --tags --always)"
  echo "agent_ref=v2.4.3"
  echo "agent_commit=$(git -C /tmp/Micro-XRCE-DDS-Agent rev-parse HEAD)"
  echo "repository_commit=$(git rev-parse HEAD)"
  echo "runner=$(lsb_release -ds)"
  echo "ros_distribution=${ROS_DISTRO:-Humble}"
  echo "gazebo_version=$(gz sim --version 2>&1 | head -1)"
  echo "recorded_at_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  cat config/resolved-sources.json
  dpkg-query -W -f='${binary:Package}=${Version}\n' 'ros-humble-*' 'gz-*' 'libgz-*' 2>/dev/null || true
} >"$OUT/versions.txt"

pids=()
diagnose() {
  echo "::group::Gazebo SITL diagnostics"
  for f in "$OUT"/*.log "$OUT"/runs/*.log; do
    [[ -f $f ]] || continue
    echo "--- $f (tail)"
    tail -n 50 "$f" || true
  done
  timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1 | head -n 80 || true
  echo "::endgroup::"
}
cleanup() {
  local status=$?
  ((status == 0)) || diagnose
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 3
  for pid in "${pids[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  cp -r "$PX4_DIR/build/px4_sitl_default/rootfs/log" "$OUT/px4-ulog" 2>/dev/null || true
}
trap cleanup EXIT

wait_for() {
  local deadline=$((SECONDS + $1))
  local description=$2
  shift 2
  while ((SECONDS < deadline)); do
    if "$@" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "timed out waiting for $description" >&2
  return 1
}

topic_name() {
  timeout 15 ros2 topic list --no-daemon --spin-time 2 |
    grep -E "$1" | head -1
}
capture_vehicle_state() {
  local prefix=$1 status_topic land_topic
  status_topic=$(topic_name '^/fmu/out/vehicle_status(_v[0-9]+)?$') || return 1
  land_topic=$(topic_name '^/fmu/out/vehicle_land_detected(_v[0-9]+)?$') || return 1
  timeout 10 ros2 topic echo --once --qos-reliability best_effort "$status_topic" \
    >"$prefix-status.yaml" 2>&1 || return 1
  timeout 10 ros2 topic echo --once --qos-reliability best_effort "$land_topic" \
    >"$prefix-land.yaml" 2>&1 || return 1
}
vehicle_safe() {
  local prefix
  prefix=$(mktemp -d)
  if ! capture_vehicle_state "$prefix/state"; then
    rm -rf "$prefix"
    return 1
  fi
  grep -Eq '^[[:space:]]*arming_state: 1$' "$prefix/state-status.yaml" &&
    grep -Eqi '^[[:space:]]*landed: true$' "$prefix/state-land.yaml"
  local result=$?
  rm -rf "$prefix"
  return "$result"
}
stop_group() {
  local pid=$1
  kill -INT -- "-$pid" 2>/dev/null || true
  for ((i = 0; i < 20; i++)); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

echo "== Starting Micro XRCE-DDS agent on the isolated GitHub runner"
setsid MicroXRCEAgent udp4 -p 8888 >"$OUT/agent.log" 2>&1 &
pids+=($!)
echo "== Starting PX4 v1.16.0 with Gazebo Harmonic gz_x500"
setsid env PX4_DIR="$PX4_DIR" HEADLESS=1 bash scripts/launch_sitl.sh >"$OUT/px4-gazebo.log" 2>&1 &
pids+=($!)
wait_for 240 "Gazebo x500 and PX4 ready" grep -q "Ready for takeoff" "$OUT/px4-gazebo.log"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin"
for kv in "UXRCE_DDS_DOM_ID 42" "COM_OF_LOSS_T 1" "COM_OBL_RC_ACT 4" "COM_RC_IN_MODE 4"; do
  timeout 30 "$PX4_BIN/px4-param" set $kv
done
{
  echo "configured UXRCE_DDS_DOM_ID=42"
  echo "configured COM_OF_LOSS_T=1"
  echo "configured COM_OBL_RC_ACT=4"
  echo "configured COM_RC_IN_MODE=4"
  timeout 30 "$PX4_BIN/px4-param" show UXRCE_DDS_DOM_ID COM_OF_LOSS_T COM_OBL_RC_ACT COM_RC_IN_MODE \
    | tee "$OUT/params-readback.txt"
} | tee "$OUT/params.txt"
timeout 20 "$PX4_BIN/px4-uxrce_dds_client" stop || true
timeout 20 "$PX4_BIN/px4-uxrce_dds_client" start -t udp -h 127.0.0.1 -p 8888
wait_for 90 "PX4 vehicle_status on ROS domain 42" \
  topic_name '^/fmu/out/vehicle_status(_v[0-9]+)?$'
timeout 20 ros2 topic list --no-daemon --spin-time 3 | grep /fmu/ | tee "$OUT/topics.txt"

failed=0
run_mission() {
  local name=$1 repeat=$2 config=$3 expected_outcome=$4 expected_reason=$5 fault=${6:-none} abort_state=${7:-}
  local stem="$OUT/runs/$name-$repeat" log="$OUT/runs/$name-$repeat.log"
  local abort_file="$OUT/runs/$name-$repeat-abort.txt"
  local status_file="$OUT/runs/$name-$repeat-status.yaml"
  local land_file="$OUT/runs/$name-$repeat-land.yaml"
  local trace_file="$OUT/traces/$name-$repeat.jsonl"
  local pid outcome reason code
  local -a extra=()
  echo "== Mission $name #$repeat: expected $expected_outcome / $expected_reason"
  if ! wait_for 90 "landed/disarmed before $name" vehicle_safe; then
    echo "vehicle was not observed landed/disarmed before mission" >"$stem-precondition.txt"
    failed=1
    : >"$log"
    : >"$trace_file"
    capture_vehicle_state "$OUT/runs/$name-$repeat" || true
    python3 scripts/sitl/gazebo_evidence.py record \
      --scenario "$name" --repeat "$repeat" --log "$log" \
      --status "$status_file" --land "$land_file" --abort "$abort_file" \
      --trace "$trace_file" --output "$stem.json"
    return
  fi
  if [[ $fault != none ]]; then
    extra=(-p "fault_scenario:=$fault" -p fault_after_seconds:=5.0)
  fi
  UAV_SITL_TRACE_FILE="$trace_file" setsid bash scripts/run_controller.sh "$config" "${extra[@]}" \
    >"$log" 2>&1 &
  pid=$!
  pids+=("$pid")
  local deadline=$((SECONDS + 180))
  local abort_sent=0
  while ((SECONDS < deadline)); do
    if [[ -n $abort_state && $abort_sent == 0 ]] && grep -q "STATE $abort_state " "$log"; then
      timeout 30 ros2 service call /sitl_mission/abort std_srvs/srv/Trigger '{}' \
        | tee "$abort_file" || true
      abort_sent=1
    fi
    if grep -Eq 'STATE .* outcome=[123] ' "$log"; then
      break
    fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  if ! grep -Eq 'STATE .* outcome=[123] ' "$log"; then
    echo "no terminal controller outcome before deadline" >>"$log"
    failed=1
  fi
  stop_group "$pid"
  if ! wait_for 90 "PX4 landed/disarmed after $name" vehicle_safe; then
    echo "PX4 did not report landed and disarmed after $name" >"$stem-state-timeout.txt"
    failed=1
  fi
  capture_vehicle_state "$OUT/runs/$name-$repeat" || true
  python3 scripts/sitl/gazebo_evidence.py record \
    --scenario "$name" --repeat "$repeat" --log "$log" \
    --status "$status_file" --land "$land_file" --abort "$abort_file" \
    --trace "$trace_file" --output "$stem.json"
  code=$(grep -oE 'STATE .* outcome=[0-3] reason=[A-Z_]+' "$log" | tail -1 || true)
  outcome=$(sed -nE 's/.* outcome=([0-3]) reason=.*/\1/p' <<<"$code")
  reason=$(sed -nE 's/.* reason=([A-Z_]+).*/\1/p' <<<"$code")
  case ${outcome:-0} in
    1) outcome=COMPLETE ;;
    2) outcome=ABORTED_LANDED ;;
    3) outcome=FAILED ;;
    *) outcome=UNKNOWN ;;
  esac
  if [[ $outcome != "$expected_outcome" || $reason != "$expected_reason" ]]; then
    echo "$name #$repeat observed $outcome / ${reason:-UNKNOWN}" >&2
    failed=1
  fi
}

for repeat in 1 2; do
  run_mission smoke "$repeat" config/controller.yaml COMPLETE NORMAL
  run_mission waypoint "$repeat" config/waypoint-mission.yaml COMPLETE NORMAL
  run_mission abort "$repeat" config/waypoint-mission.yaml ABORTED_LANDED OPERATOR_ABORT none NAVIGATION
done
run_mission gnss_loss 1 config/controller.yaml ABORTED_LANDED INVALID_POSITION gnss_loss
run_mission position_jump 1 config/controller.yaml ABORTED_LANDED TRACKING_ERROR position_jump
run_mission stale_telemetry 1 config/controller.yaml FAILED LANDING_UNVERIFIED stale_telemetry
run_mission geofence 1 config/controller.yaml ABORTED_LANDED GEOFENCE geofence

python3 scripts/sitl/gazebo_evidence.py validate --input "$OUT" \
  --output "$OUT/evidence.json" --summary "$OUT/summary.md" || failed=1
exit "$failed"
