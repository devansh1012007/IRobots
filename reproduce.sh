#!/usr/bin/env bash
# One-command reproduction script for IRobots v2 (PLAN:irobots-v2:C61, SC5).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON_BIN="${PYTHON_BIN:-python3}"
if [[ -x "/tmp/irobots_venv/bin/python" ]]; then
  PYTHON_BIN="/tmp/irobots_venv/bin/python"
elif [[ -x "$SCRIPT_DIR/.venv/bin/python" ]]; then
  PYTHON_BIN="$SCRIPT_DIR/.venv/bin/python"
fi

echo "[reproduce] 1/5 Verifying generated firmware header matches config/robot.yaml..."
"$PYTHON_BIN" tools/gen_firmware_config.py
git diff --exit-code firmware/esp32_arm/generated/robot_config.h

echo "[reproduce] 2/5 Running unit and integration test suite..."
"$PYTHON_BIN" -m pytest ros2_ws/src/irobots_core/tests

echo "[reproduce] 3/5 Running Phase 0 perception gate (C02)..."
"$PYTHON_BIN" spike/perception_bench.py --trials 50 --no-semantic

echo "[reproduce] 4/5 Running RQ0-RQ3 evaluations and SC1/SC2/SC4 gates..."
"$PYTHON_BIN" -m irobots_eval.exp_rq0
"$PYTHON_BIN" -m irobots_eval.exp_rq1
"$PYTHON_BIN" -m irobots_eval.exp_rq2
"$PYTHON_BIN" -m irobots_eval.exp_rq3

echo "[reproduce] 5/5 Regenerating docs/RESULTS.md from runs/ JSON..."
"$PYTHON_BIN" tools/generate_results_md.py
echo "[reproduce] Done. Headline results written to docs/RESULTS.md."
