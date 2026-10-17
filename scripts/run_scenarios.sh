#!/usr/bin/env bash
# Build and run the fault-injection catalog, then render the safety report.
# Usage: bash scripts/run_scenarios.sh [scenario ...]   Exits non-zero if any scenario fails.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build artifacts/scenarios
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -Icore/include core/scenario_runner.cpp -o build/uav_scenarios
status=0
./build/uav_scenarios artifacts/scenarios "$@" || status=$?
python3 scripts/safety_report.py artifacts/scenarios/safety-report.json
echo "Traces for dashboard replay: artifacts/scenarios/traces/<name>.jsonl"
exit "$status"
