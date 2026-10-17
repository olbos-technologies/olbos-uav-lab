#!/usr/bin/env bash
set -euo pipefail
: "${PX4_DIR:?Set PX4_DIR to your PX4 v1.16.0 checkout}"
test "$(git -C "$PX4_DIR" describe --tags --exact-match)" = v1.16.0 || { echo "PX4 v1.16.0 required"; exit 1; }
cd "$PX4_DIR"
export PX4_UXRCE_DDS_PORT=8888
exec make px4_sitl gz_x500
