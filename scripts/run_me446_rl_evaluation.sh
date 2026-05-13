#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PYTHON_BIN="${PYTHON_BIN:-python}"
EVAL_SCRIPT="${PROJECT_ROOT}/mujoco_test/test_script/eval_go2_policy_logger.py"
CHECKPOINT="${CHECKPOINT:-${PROJECT_ROOT}/legged_gym/logs/rough_go2/TS_re3/model_9000.pt}"
POLICY_PATH="${POLICY_PATH:-}"
ENCODER_PATH="${ENCODER_PATH:-${PROJECT_ROOT}/mujoco_test/model/rsl_rl_teacher_student/proprio_encoder/proprio_oracle_ece489.pth}"
OUT_ROOT="${OUT_ROOT:-${PROJECT_ROOT}/project_outputs/eval_logs}"
DURATION="${DURATION:-20}"
HEADLESS_FLAG="${HEADLESS_FLAG:---headless}"

if [[ -n "${POLICY_PATH}" ]]; then
  POLICY_ARGS=(--policy_path "${POLICY_PATH}" --encoder_path "${ENCODER_PATH}")
  echo "Evaluation policy mode: actor/proprio encoder"
  echo "  policy:  ${POLICY_PATH}"
  echo "  encoder: ${ENCODER_PATH}"
else
  POLICY_ARGS=(--checkpoint "${CHECKPOINT}")
  echo "Evaluation policy mode: checkpoint"
  echo "  checkpoint: ${CHECKPOINT}"
fi

run_condition () {
  local condition="$1"
  local scene="$2"
  local vx="$3"
  local push_force="${4:-0}"
  local out_dir="${OUT_ROOT}/${condition}"

  mkdir -p "${out_dir}"
  for trial in $(seq 0 9); do
    "${PYTHON_BIN}" "${EVAL_SCRIPT}" \
      --scene "${scene}" \
      --cmd_vx "${vx}" \
      --cmd_vy 0 \
      --cmd_yaw 0 \
      --duration "${DURATION}" \
      --trial "${trial}" \
      --out_dir "${out_dir}" \
      "${POLICY_ARGS[@]}" \
      --push_force "${push_force}" \
      --push_start 5.0 \
      --push_duration 0.1 \
      ${HEADLESS_FLAG}
  done
}

run_condition "flat_0p4" "flat" "0.4"
run_condition "flat_0p8" "flat" "0.8"
run_condition "terrain_0p4" "terrain" "0.4"
run_condition "terrain_0p8" "terrain" "0.8"
run_condition "push_flat_0p4" "flat" "0.4" "40"
run_condition "push_terrain_0p4" "terrain" "0.4" "40"

echo "Evaluation logs written to ${OUT_ROOT}"
