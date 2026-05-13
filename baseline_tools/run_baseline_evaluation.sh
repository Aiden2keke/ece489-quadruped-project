#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DURATION="${DURATION:-20}"
TRIALS="${TRIALS:-10}"
OUT_ROOT="${OUT_ROOT:-${ROOT_DIR}/project_outputs_baseline/eval_logs}"
BASELINE_REPO="${BASELINE_REPO:-${ROOT_DIR}/baselines/go2-convex-mpc}"
PYTHON_BIN="${PYTHON_BIN:-python}"
INIT_MODE="${INIT_MODE:-baseline_demo}"
LOGGER="${ROOT_DIR}/baseline_tools/eval_go2_convex_mpc_logger.py"

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"

mkdir -p "${OUT_ROOT}"

failures=0

run_condition() {
  local condition="$1"
  local scene="$2"
  local vx="$3"
  local push_force="$4"
  local out_dir="${OUT_ROOT}/${condition}"

  mkdir -p "${out_dir}"
  echo "=== ${condition}: scene=${scene}, vx=${vx}, push=${push_force}N, trials=${TRIALS}, duration=${DURATION}s, init=${INIT_MODE} ==="

  local trial
  for ((trial = 0; trial < TRIALS; trial++)); do
    printf -v trial_padded "%02d" "${trial}"
    echo "[${condition}] trial ${trial_padded}"
    if ! "${PYTHON_BIN}" "${LOGGER}" \
      --scene "${scene}" \
      --cmd_vx "${vx}" \
      --cmd_vy 0 \
      --cmd_yaw 0 \
      --duration "${DURATION}" \
      --trial "${trial}" \
      --out_dir "${out_dir}" \
      --push_force "${push_force}" \
      --push_start 5.0 \
      --push_duration 0.1 \
      --baseline_repo "${BASELINE_REPO}" \
      --init_mode "${INIT_MODE}" \
      --headless; then
      failures=$((failures + 1))
      echo "trial ${trial_padded} failed; keep any partial CSV and sidecar error file for analysis" > "${out_dir}/trial${trial_padded}.failed.txt"
    fi
  done
}

run_condition "flat_0p4" "flat" "0.4" "0"
run_condition "flat_0p8" "flat" "0.8" "0"
run_condition "terrain_0p4" "terrain" "0.4" "0"
run_condition "terrain_0p8" "terrain" "0.8" "0"
run_condition "push_flat_0p4" "flat" "0.4" "40"
run_condition "push_terrain_0p4" "terrain" "0.4" "40"

echo "Baseline evaluation finished with ${failures} failed trial command(s)."
if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi
