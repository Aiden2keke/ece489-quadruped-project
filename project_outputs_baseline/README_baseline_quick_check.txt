Baseline 快速检查清单

1. 主要目录
- baselines/go2-convex-mpc：下载的 convex MPC baseline 原仓库，没有为了评估大改源码。
- baseline_tools：本项目新增的 baseline 评估 wrapper、批量运行脚本、分析入口。
- project_outputs_baseline：baseline 的资料、日志、结果和截图输出目录。
- project_outputs_baseline/eval_logs_failed_rl_scene_init：第一次尝试使用 RL scene 内嵌初始姿态时产生的失败 trial，已归档保留，不作为最终 summary 的输入。
- project_outputs_RL：已有 RL 输出目录，不要覆盖、删除或移动。

2. 关键文件
- project_outputs_baseline/baseline_repo_info.md：baseline 仓库 URL、commit、环境检查、模型路径、改动说明。
- project_outputs_baseline/baseline_code_inspection.md：baseline 代码结构和运行方式检查。
- project_outputs_baseline/README_BASELINE_EXPERIMENTS_AND_EVALUATION.md：baseline 实验和评估说明。
- project_outputs_baseline/evaluation_run_status.md：最终 batch 是否跑完、哪些 condition 有失败 sidecar、分析命令和对比命令。
- baseline_tools/eval_go2_convex_mpc_logger.py：单次 trial logger。
- baseline_tools/run_baseline_evaluation.sh：六个 condition 的批量评估脚本。
- tools/compare_rl_baseline_metrics.py：可选 RL vs baseline 对比脚本。

3. 是否跑完 60 个 baseline trials
在项目根目录运行：

find project_outputs_baseline/eval_logs -name '*.csv' | wc -l

正常完整结果应该是 60。
本次最终结果已经生成 60 个 CSV，其中 flat_0p8 和 terrain_0p8 各有 10 个 .error.txt，表示 controller QP failure 的 partial trial。

4. 检查每个 condition 是否各有 10 个 CSV

for d in project_outputs_baseline/eval_logs/*; do echo "$d"; find "$d" -maxdepth 1 -name '*.csv' | wc -l; done

每个目录应该输出 10。

5. 检查 summary_metrics.csv

ls project_outputs_baseline/eval_results/summary_metrics.csv
cat project_outputs_baseline/eval_results/summary_metrics.csv

如果文件不存在，先运行：

python tools/analyze_eval_logs.py --eval_root project_outputs_baseline/eval_logs --out_dir project_outputs_baseline/eval_results

6. 和 RL 对比
核心对比文件：

project_outputs_RL/eval_results/summary_metrics.csv
project_outputs_baseline/eval_results/summary_metrics.csv

可选生成对比图：

python tools/compare_rl_baseline_metrics.py --rl_summary project_outputs_RL/eval_results/summary_metrics.csv --baseline_summary project_outputs_baseline/eval_results/summary_metrics.csv --out_dir project_outputs_comparison

7. 失败 trial 处理
如果某些 condition 摔倒或 controller 报错，不要删除 CSV、.error.txt 或 .failed.txt。
失败 trial 要保留，用于 failure mode 分析。

8. 当前环境提醒
默认 python 缺少 mujoco、pinocchio、casadi。
已经按 environment.yml 创建了 go2-convex-mpc 环境；运行 baseline 时建议使用：

PYTHON_BIN=/home/yd/anaconda3/envs/go2-convex-mpc/bin/python baseline_tools/run_baseline_evaluation.sh

原始 examples/ex02_trot_forward 的仿真段可以跑，但最后 viewer 在当前无 GUI 会话中会因为 X11 display 不可用而失败；批量评估请用 headless wrapper。

9. 初始化模式说明
最终 baseline 默认使用 INIT_MODE=baseline_demo，也就是上游 demo 的站立姿态。
如果使用 INIT_MODE=rl_scene，当前代码中会在 0.14-0.18 秒左右出现 OSQP failure；这些失败日志已经保留在 eval_logs_failed_rl_scene_init。
