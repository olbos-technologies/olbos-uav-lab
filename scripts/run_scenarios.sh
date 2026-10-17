#!/usr/bin/env bash
# Build and run the fault-injection catalog and envelope sweeps, then render the safety report.
# Usage: bash scripts/run_scenarios.sh [--config params.yaml] [--out dir] [--no-sweeps] [scenario ...]
# Exits non-zero if any scenario fails or any sweep point breaks a safety invariant.
set -euo pipefail
cd "$(dirname "$0")/.."
out=artifacts/scenarios
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
  if [[ ${args[i]} == --out && $((i + 1)) -lt ${#args[@]} ]]; then out=${args[i + 1]}; fi
done
mkdir -p build
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -Icore/include core/scenario_runner.cpp -o build/uav_scenarios
# A stale report must never be re-rendered after a failed or rejected run.
rm -f "$out"/safety-report.{json,md,html}
status=0
./build/uav_scenarios --out "$out" "$@" || status=$?
if [[ -f $out/safety-report.json ]]; then
  python3 scripts/safety_report.py "$out/safety-report.json"
  echo "Traces for dashboard replay: $out/traces/<name>.jsonl"
fi
exit "$status"
