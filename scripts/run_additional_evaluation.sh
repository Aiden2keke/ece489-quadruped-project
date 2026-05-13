#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ADDITIONAL_ROOT="${ADDITIONAL_ROOT:-${PROJECT_ROOT}/project_outputs_additional}"
DURATION="${DURATION:-20}"
TRIALS="${TRIALS:-10}"
FORCE="${FORCE:-0}"

RL_PYTHON_CMD="${RL_PYTHON_CMD:-conda run -n rlgpu python}"
BASELINE_PYTHON_CMD="${BASELINE_PYTHON_CMD:-conda run -n go2-convex-mpc python}"
ANALYSIS_PYTHON_CMD="${ANALYSIS_PYTHON_CMD:-python}"

read -r -a RL_PYTHON <<< "${RL_PYTHON_CMD}"
read -r -a BASELINE_PYTHON <<< "${BASELINE_PYTHON_CMD}"
read -r -a ANALYSIS_PYTHON <<< "${ANALYSIS_PYTHON_CMD}"

RL_LOGGER="${PROJECT_ROOT}/mujoco_test/test_script/eval_go2_policy_logger.py"
BASELINE_LOGGER="${PROJECT_ROOT}/baseline_tools/eval_go2_convex_mpc_logger.py"
ANALYZER="${PROJECT_ROOT}/tools/analyze_eval_logs.py"
COMPARER="${PROJECT_ROOT}/tools/compare_additional_evaluations.py"
CHECKPOINT="${CHECKPOINT:-${PROJECT_ROOT}/legged_gym/logs/rough_go2/TS_re3/model_9000.pt}"
BASELINE_REPO="${BASELINE_REPO:-${PROJECT_ROOT}/baselines/go2-convex-mpc}"
BASELINE_INIT_MODE="${BASELINE_INIT_MODE:-baseline_demo}"

mkdir -p \
  "${ADDITIONAL_ROOT}/RL/speed_sweep/eval_logs" \
  "${ADDITIONAL_ROOT}/RL/speed_sweep/eval_results" \
  "${ADDITIONAL_ROOT}/RL/robustness/eval_logs" \
  "${ADDITIONAL_ROOT}/RL/robustness/eval_results" \
  "${ADDITIONAL_ROOT}/baseline/speed_sweep/eval_logs" \
  "${ADDITIONAL_ROOT}/baseline/speed_sweep/eval_results" \
  "${ADDITIONAL_ROOT}/baseline/robustness/eval_logs" \
  "${ADDITIONAL_ROOT}/baseline/robustness/eval_results" \
  "${ADDITIONAL_ROOT}/comparison"

START_TIME="$(date -Iseconds)"
GIT_HASH="$(git -C "${PROJECT_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
trial_command_failures=0
analysis_failures=0

vx_token() {
  local value="$1"
  echo "${value}" | sed 's/-/m/g; s/\./p/g'
}

run_trial() {
  local method="$1"
  local task="$2"
  local condition="$3"
  local scene="$4"
  local vx="$5"
  local trial="$6"
  shift 6
  local extra_args=("$@")

  local out_dir
  local logger
  local cmd=()
  local token
  local csv_path
  local error_path
  local tmp_log

  if [[ "${task}" == "speed_sweep" ]]; then
    out_dir="${ADDITIONAL_ROOT}/${method}/speed_sweep/eval_logs/${condition}"
  else
    out_dir="${ADDITIONAL_ROOT}/${method}/robustness/eval_logs/${condition}"
  fi
  mkdir -p "${out_dir}"

  token="$(vx_token "${vx}")"
  printf -v trial_padded "%02d" "${trial}"
  csv_path="${out_dir}/${scene}_vx${token}_trial${trial_padded}.csv"
  error_path="${out_dir}/${scene}_vx${token}_trial${trial_padded}.error.txt"

  if [[ "${FORCE}" != "1" && ( -f "${csv_path}" || -f "${error_path}" ) ]]; then
    echo "[skip] ${method} ${task}/${condition} trial ${trial_padded} already has output"
    return 0
  fi

  echo "[run] ${method} ${task}/${condition} trial ${trial_padded}: scene=${scene}, vx=${vx}, duration=${DURATION}s"
  tmp_log="$(mktemp)"

  if [[ "${method}" == "RL" ]]; then
    logger="${RL_LOGGER}"
    cmd=(
      "${RL_PYTHON[@]}" "${logger}"
      --scene "${scene}"
      --cmd_vx "${vx}"
      --cmd_vy 0
      --cmd_yaw 0
      --duration "${DURATION}"
      --trial "${trial}"
      --out_dir "${out_dir}"
      --checkpoint "${CHECKPOINT}"
      --headless
      "${extra_args[@]}"
    )
  else
    logger="${BASELINE_LOGGER}"
    cmd=(
      "${BASELINE_PYTHON[@]}" "${logger}"
      --scene "${scene}"
      --cmd_vx "${vx}"
      --cmd_vy 0
      --cmd_yaw 0
      --duration "${DURATION}"
      --trial "${trial}"
      --out_dir "${out_dir}"
      --baseline_repo "${BASELINE_REPO}"
      --init_mode "${BASELINE_INIT_MODE}"
      --headless
      "${extra_args[@]}"
    )
  fi

  if "${cmd[@]}" > "${tmp_log}" 2>&1; then
    rm -f "${tmp_log}"
    return 0
  fi

  trial_command_failures=$((trial_command_failures + 1))
  {
    echo "Additional evaluation command failed."
    echo "method=${method}"
    echo "task=${task}"
    echo "condition=${condition}"
    echo "trial=${trial_padded}"
    echo "command=${cmd[*]}"
    echo
    echo "--- command output ---"
    cat "${tmp_log}"
  } >> "${error_path}"
  rm -f "${tmp_log}"
  echo "[failed] ${method} ${task}/${condition} trial ${trial_padded}; see ${error_path}"
}

run_speed_sweep_for_method() {
  local method="$1"
  local vx
  for vx in 0.4 0.6 0.8 1.0 1.2; do
    local condition="flat_$(vx_token "${vx}")"
    local trial
    for ((trial = 0; trial < TRIALS; trial++)); do
      run_trial "${method}" "speed_sweep" "${condition}" "flat" "${vx}" "${trial}"
    done
  done
}

run_robustness_for_method() {
  local method="$1"
  local trial
  for ((trial = 0; trial < TRIALS; trial++)); do
    run_trial "${method}" "robustness" "friction_low" "flat" "0.4" "${trial}" --ground_friction 0.5
  done
  for ((trial = 0; trial < TRIALS; trial++)); do
    run_trial "${method}" "robustness" "payload_high" "flat" "0.4" "${trial}" --base_mass_scale 1.2
  done
  for ((trial = 0; trial < TRIALS; trial++)); do
    run_trial "${method}" "robustness" "motor_weak" "flat" "0.4" "${trial}" --torque_scale 0.9
  done
}

run_analysis() {
  local label="$1"
  local eval_root="$2"
  local out_dir="$3"
  echo "[analysis] ${label}"
  if ! "${ANALYSIS_PYTHON[@]}" "${ANALYZER}" --eval_root "${eval_root}" --out_dir "${out_dir}"; then
    analysis_failures=$((analysis_failures + 1))
    echo "[analysis failed] ${label}"
  fi
}

count_files() {
  local dir="$1"
  local pattern="$2"
  find "${dir}" -name "${pattern}" 2>/dev/null | wc -l | tr -d ' '
}

append_condition_status() {
  local status_file="$1"
  local root="$2"
  local expected_per_condition="$3"
  local any=0
  local condition

  for condition in "$root"/*; do
    [[ -d "${condition}" ]] || continue
    local csv_count
    local error_count
    csv_count="$(count_files "${condition}" "*.csv")"
    error_count="$(count_files "${condition}" "*.error.txt")"
    if [[ "${csv_count}" -ne "${expected_per_condition}" || "${error_count}" -ne 0 ]]; then
      any=1
      echo "- $(basename "${condition}"): ${csv_count}/${expected_per_condition} CSV, ${error_count} .error.txt" >> "${status_file}"
    fi
  done

  if [[ "${any}" -eq 0 ]]; then
    echo "- none" >> "${status_file}"
  fi
}

append_success_issues() {
  local status_file="$1"
  local csv_path="$2"
  if [[ ! -f "${csv_path}" ]]; then
    echo "- ${csv_path}: missing" >> "${status_file}"
    return
  fi
  awk -F, 'NR > 1 && $6 != "" && $6 + 0 < 1 {printf("- %s %s/%s: success_rate=%s\n", $1, $2, $3, $6)}' "${csv_path}" >> "${status_file}"
}

write_status() {
  local status_file="${ADDITIONAL_ROOT}/additional_run_status.md"
  local speed_rl="${ADDITIONAL_ROOT}/RL/speed_sweep/eval_logs"
  local speed_baseline="${ADDITIONAL_ROOT}/baseline/speed_sweep/eval_logs"
  local robust_rl="${ADDITIONAL_ROOT}/RL/robustness/eval_logs"
  local robust_baseline="${ADDITIONAL_ROOT}/baseline/robustness/eval_logs"
  local comparison="${ADDITIONAL_ROOT}/comparison"
  local plot

  {
    echo "# Additional Evaluation Run Status"
    echo
    echo "- Run start: ${START_TIME}"
    echo "- Status written: $(date -Iseconds)"
    echo "- Git commit: ${GIT_HASH}"
    echo "- Project root: ${PROJECT_ROOT}"
    echo "- Additional output root: ${ADDITIONAL_ROOT}"
    echo "- Duration per trial: ${DURATION} s"
    echo "- Trials per condition: ${TRIALS}"
    echo "- FORCE rerun: ${FORCE}"
    echo
    echo "## Environment"
    echo
    echo "- RL command: \`${RL_PYTHON_CMD}\`"
    echo "- Baseline command: \`${BASELINE_PYTHON_CMD}\`"
    echo "- Analysis command: \`${ANALYSIS_PYTHON_CMD}\`"
    echo "- RL checkpoint: \`${CHECKPOINT}\`"
    echo "- Baseline repo: \`${BASELINE_REPO}\`"
    echo "- Baseline init mode: \`${BASELINE_INIT_MODE}\`"
    echo
    echo "## Commands Run"
    echo
    echo "- \`${RL_PYTHON_CMD} ${RL_LOGGER} --scene flat --cmd_vx <0.4|0.6|0.8|1.0|1.2> --duration ${DURATION} --trial <0..$((TRIALS - 1))> --out_dir project_outputs_additional/RL/speed_sweep/eval_logs/<condition> --checkpoint ${CHECKPOINT} --headless\`"
    echo "- \`${BASELINE_PYTHON_CMD} ${BASELINE_LOGGER} --scene flat --cmd_vx <0.4|0.6|0.8|1.0|1.2> --duration ${DURATION} --trial <0..$((TRIALS - 1))> --out_dir project_outputs_additional/baseline/speed_sweep/eval_logs/<condition> --baseline_repo ${BASELINE_REPO} --init_mode ${BASELINE_INIT_MODE} --headless\`"
    echo "- Robustness variants used the same commands with one of: \`--ground_friction 0.5\`, \`--base_mass_scale 1.2\`, \`--torque_scale 0.9\`."
    echo "- \`${ANALYSIS_PYTHON_CMD} ${ANALYZER} --eval_root <eval_logs> --out_dir <eval_results>\`"
    echo "- \`${ANALYSIS_PYTHON_CMD} ${COMPARER} --additional_root ${ADDITIONAL_ROOT}\`"
    echo
    echo "## File Counts"
    echo
    echo "| Method/task | CSV | .error.txt | Expected CSV |"
    echo "| --- | ---: | ---: | ---: |"
    echo "| RL speed_sweep | $(count_files "${speed_rl}" "*.csv") | $(count_files "${speed_rl}" "*.error.txt") | $((5 * TRIALS)) |"
    echo "| baseline speed_sweep | $(count_files "${speed_baseline}" "*.csv") | $(count_files "${speed_baseline}" "*.error.txt") | $((5 * TRIALS)) |"
    echo "| RL robustness | $(count_files "${robust_rl}" "*.csv") | $(count_files "${robust_rl}" "*.error.txt") | $((3 * TRIALS)) |"
    echo "| baseline robustness | $(count_files "${robust_baseline}" "*.csv") | $(count_files "${robust_baseline}" "*.error.txt") | $((3 * TRIALS)) |"
    echo
    echo "## Conditions With Missing CSV Or .error.txt"
    echo
    echo "### RL speed_sweep"
  } > "${status_file}"
  append_condition_status "${status_file}" "${speed_rl}" "${TRIALS}"
  echo "### baseline speed_sweep" >> "${status_file}"
  append_condition_status "${status_file}" "${speed_baseline}" "${TRIALS}"
  echo "### RL robustness" >> "${status_file}"
  append_condition_status "${status_file}" "${robust_rl}" "${TRIALS}"
  echo "### baseline robustness" >> "${status_file}"
  append_condition_status "${status_file}" "${robust_baseline}" "${TRIALS}"

  {
    echo
    echo "## Conditions With Non-Unit Success Rate"
  } >> "${status_file}"
  append_success_issues "${status_file}" "${comparison}/comparison_speed_sweep.csv"
  append_success_issues "${status_file}" "${comparison}/comparison_robustness.csv"

  {
    echo
    echo "## Comparison Outputs"
    for plot in \
      comparison_speed_sweep.csv \
      comparison_speed_sweep_success_rate.png \
      comparison_speed_sweep_cot.png \
      comparison_speed_sweep_velocity_error.png \
      comparison_energy_efficiency.csv \
      comparison_energy_efficiency.png \
      comparison_robustness.csv \
      comparison_robustness_success_rate.png \
      comparison_robustness_cot.png \
      comparison_robustness_velocity_error.png; do
      if [[ -f "${comparison}/${plot}" ]]; then
        echo "- ${plot}: generated"
      else
        echo "- ${plot}: not generated"
      fi
    done
    echo
    echo "## Perturbation Implementation Notes"
    echo
    echo "- friction_low: MuJoCo ground geom named \`floor\` has sliding friction set to 0.5 at model load time."
    echo "- payload_high: MuJoCo body \`base_link\` mass and diagonal inertia are multiplied by 1.2 at model load time."
    echo "- motor_weak: applied joint torque is multiplied by 0.9 before writing \`data.ctrl\`; logged torque is the scaled applied torque."
    echo "- Source XML files are not modified."
    echo
    echo "## Failure Handling"
    echo
    echo "- Trial command/controller failures are retained as partial CSV when available plus a same-stem \`.error.txt\`."
    echo "- Falls are retained in CSV via \`fall_flag\` and \`success_flag=0\`; they are not deleted."
    echo "- CoT in the comparison tables is computed from successful full-length trials only where per-trial metrics are available."
    echo "- Do-not-fabricate-data policy: no synthetic CSV, success rates, CoT values, or plots are created."
    echo
    echo "## Remaining Work / Next Commands"
    echo
    if [[ "${trial_command_failures}" -ne 0 ]]; then
      echo "- Some trial commands/controller solves failed and were recorded as evaluation data. Inspect \`project_outputs_additional/**/**/*.error.txt\` for details; rerun with \`FORCE=1 scripts/run_additional_evaluation.sh\` only if intentionally repeating the experiments after changing the controller or environment."
    elif [[ "${analysis_failures}" -ne 0 ]]; then
      echo "- Some analysis steps failed. Rerun \`python tools/analyze_eval_logs.py --eval_root <eval_logs> --out_dir <eval_results>\` and then \`python tools/compare_additional_evaluations.py --additional_root project_outputs_additional\`."
    else
      echo "- No incomplete script step detected. To reproduce from scratch, run \`FORCE=1 scripts/run_additional_evaluation.sh\`."
    fi
  } >> "${status_file}"

  echo "[status] wrote ${status_file}"
}

echo "=== Additional Evaluation: RL speed sweep ==="
run_speed_sweep_for_method "RL"
echo "=== Additional Evaluation: baseline speed sweep ==="
run_speed_sweep_for_method "baseline"
echo "=== Additional Evaluation: RL robustness ==="
run_robustness_for_method "RL"
echo "=== Additional Evaluation: baseline robustness ==="
run_robustness_for_method "baseline"

run_analysis "RL speed_sweep" "${ADDITIONAL_ROOT}/RL/speed_sweep/eval_logs" "${ADDITIONAL_ROOT}/RL/speed_sweep/eval_results"
run_analysis "baseline speed_sweep" "${ADDITIONAL_ROOT}/baseline/speed_sweep/eval_logs" "${ADDITIONAL_ROOT}/baseline/speed_sweep/eval_results"
run_analysis "RL robustness" "${ADDITIONAL_ROOT}/RL/robustness/eval_logs" "${ADDITIONAL_ROOT}/RL/robustness/eval_results"
run_analysis "baseline robustness" "${ADDITIONAL_ROOT}/baseline/robustness/eval_logs" "${ADDITIONAL_ROOT}/baseline/robustness/eval_results"

echo "[comparison] additional evaluation"
if ! "${ANALYSIS_PYTHON[@]}" "${COMPARER}" --additional_root "${ADDITIONAL_ROOT}"; then
  analysis_failures=$((analysis_failures + 1))
  echo "[comparison failed]"
fi

write_status

echo "Additional evaluation trial command failures: ${trial_command_failures}"
echo "Additional evaluation analysis/comparison failures: ${analysis_failures}"

if [[ "${analysis_failures}" -ne 0 ]]; then
  exit 1
fi
exit 0
