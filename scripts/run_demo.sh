#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build artifacts
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -Icore/include core/demo.cpp -o build/uav_demo
./build/uav_demo "${1:-normal}" > artifacts/flight.jsonl
echo "Kinematic harness only: artifacts/flight.jsonl"
