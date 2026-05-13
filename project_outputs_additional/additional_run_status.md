# Additional Evaluation Run Status

- Run start: 2026-05-13T21:05:40+08:00
- Status written: 2026-05-13T21:30:39+08:00
- Git commit: b0ff51a
- Project root: /home/yd/ece489/project/rsl_rl_teacher_student
- Additional output root: /home/yd/ece489/project/rsl_rl_teacher_student/project_outputs_additional
- Duration per trial: 20 s
- Trials per condition: 10
- FORCE rerun: 0

## Environment

- RL command: `conda run -n rlgpu python`
- Baseline command: `conda run -n go2-convex-mpc python`
- Analysis command: `python`
- RL checkpoint: `/home/yd/ece489/project/rsl_rl_teacher_student/legged_gym/logs/rough_go2/TS_re3/model_9000.pt`
- Baseline repo: `/home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc`
- Baseline init mode: `baseline_demo`

## Commands Run

- `conda run -n rlgpu python /home/yd/ece489/project/rsl_rl_teacher_student/mujoco_test/test_script/eval_go2_policy_logger.py --scene flat --cmd_vx <0.4|0.6|0.8|1.0|1.2> --duration 20 --trial <0..9> --out_dir project_outputs_additional/RL/speed_sweep/eval_logs/<condition> --checkpoint /home/yd/ece489/project/rsl_rl_teacher_student/legged_gym/logs/rough_go2/TS_re3/model_9000.pt --headless`
- `conda run -n go2-convex-mpc python /home/yd/ece489/project/rsl_rl_teacher_student/baseline_tools/eval_go2_convex_mpc_logger.py --scene flat --cmd_vx <0.4|0.6|0.8|1.0|1.2> --duration 20 --trial <0..9> --out_dir project_outputs_additional/baseline/speed_sweep/eval_logs/<condition> --baseline_repo /home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc --init_mode baseline_demo --headless`
- Robustness variants used the same commands with one of: `--ground_friction 0.5`, `--base_mass_scale 1.2`, `--torque_scale 0.9`.
- `python /home/yd/ece489/project/rsl_rl_teacher_student/tools/analyze_eval_logs.py --eval_root <eval_logs> --out_dir <eval_results>`
- `python /home/yd/ece489/project/rsl_rl_teacher_student/tools/compare_additional_evaluations.py --additional_root /home/yd/ece489/project/rsl_rl_teacher_student/project_outputs_additional`

## File Counts

| Method/task | CSV | .error.txt | Expected CSV |
| --- | ---: | ---: | ---: |
| RL speed_sweep | 50 | 0 | 50 |
| baseline speed_sweep | 50 | 30 | 50 |
| RL robustness | 30 | 0 | 30 |
| baseline robustness | 30 | 0 | 30 |

## Conditions With Missing CSV Or .error.txt

### RL speed_sweep
- none
### baseline speed_sweep
- flat_0p8: 10/10 CSV, 10 .error.txt
- flat_1p0: 10/10 CSV, 10 .error.txt
- flat_1p2: 10/10 CSV, 10 .error.txt
### RL robustness
- none
### baseline robustness
- none

## Conditions With Non-Unit Success Rate
- baseline speed_sweep/flat_0p8: success_rate=0.0
- baseline speed_sweep/flat_1p0: success_rate=0.0
- baseline speed_sweep/flat_1p2: success_rate=0.0

## Comparison Outputs
- comparison_speed_sweep.csv: generated
- comparison_speed_sweep_success_rate.png: generated
- comparison_speed_sweep_cot.png: generated
- comparison_speed_sweep_velocity_error.png: generated
- comparison_energy_efficiency.csv: generated
- comparison_energy_efficiency.png: generated
- comparison_robustness.csv: generated
- comparison_robustness_success_rate.png: generated
- comparison_robustness_cot.png: generated
- comparison_robustness_velocity_error.png: generated

## Perturbation Implementation Notes

- friction_low: MuJoCo ground geom named `floor` has sliding friction set to 0.5 at model load time.
- payload_high: MuJoCo body `base_link` mass and diagonal inertia are multiplied by 1.2 at model load time.
- motor_weak: applied joint torque is multiplied by 0.9 before writing `data.ctrl`; logged torque is the scaled applied torque.
- Source XML files are not modified.

## Failure Handling

- Trial command/controller failures are retained as partial CSV when available plus a same-stem `.error.txt`.
- Falls are retained in CSV via `fall_flag` and `success_flag=0`; they are not deleted.
- CoT in the comparison tables is computed from successful full-length trials only where per-trial metrics are available.
- Analysis/comparison files were regenerated after setting failed-trial per-trial CoT to `nan`, so failed partial controller runs are not shown as valid completed-task CoT.
- Do-not-fabricate-data policy: no synthetic CSV, success rates, CoT values, or plots are created.

## Remaining Work / Next Commands

- Some trial commands/controller solves failed and were recorded as evaluation data. Inspect `project_outputs_additional/**/**/*.error.txt` for details; rerun with `FORCE=1 scripts/run_additional_evaluation.sh` only if intentionally repeating the experiments after changing the controller or environment.
