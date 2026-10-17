#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if ! python3 -c 'import numpy, sklearn, joblib, websockets' >/dev/null 2>&1; then
  echo "Install demo dependencies first: python3 -m pip install -r machine_learning/requirements.txt" >&2
  exit 2
fi

if [[ ! -f artifacts/health_rf.joblib ]]; then
  echo "Training the repository's synthetic-only advisory model..."
  python3 machine_learning/training/train.py
fi

exec python3 scripts/sim_demo_ws.py --host 127.0.0.1 --port 8766
