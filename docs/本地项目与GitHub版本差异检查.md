# 本地项目与 GitHub 版本差异检查

## 发布范围（2026-09-30）

> 下文是2026-09-30整理上传之前的历史快照，不是当前待提交清单。整理后范围与用法见 [当前版本指南](版本整理与使用指南_20260930.md)，实时状态以Git检查为准。

检查日期：2026-09-30（香港时间）。对象为 `stm32-mpu6050-f9p-navigation` 和 `pi5-mpu6050-f9p-logger`。

本次已分别成功执行 `git fetch origin --prune`，比较基准为 GitHub 主仓库最新的 `origin/main`。fetch 只更新远端引用，不合并代码、不改写工作文件、不提交或推送。

## 1. 当前结论

两个项目的本地 `main` 提交历史都与 `origin/main` 一致，领先和落后提交数均为 0。

- STM32 项目存在尚未提交的修改，以及本地新增文件；完整清单见本文末尾。
- Pi 项目当前没有已跟踪文件差异，也没有非忽略的未跟踪文件。

这不表示被忽略的采集数据已经上传到 GitHub，也不表示树莓派上的实际部署代码与本地一致。本次未比较树莓派部署目录。

## 2. STM32 主要改动概览

以下概览来自文本 diff；二进制文件只确认有字节变化，未进行数值或图像等价性分析。

| 文件或范围 | 本地改动 |
|---|---|
| `tools/allan_compare_tc_configs.py` | 增加对比阶段选择、已温补 CSV 输入、数据序列与模型一致性检查，以及绘图布局、尺寸等参数；报告输出有调整 |
| `tools/fit_temp_bias_raw.m` | 增加温补前后六轴分块均值图，调整单位、曲线样式、字体和导出布局 |
| 电动车会话的 `test.m` | 使用新的 Allan 白噪声与自相关 GM 参数，将初始 eb/db 与过程误差参数分开；算法与绘图的其他变化见完整 diff |
| `compare_acc_calibration_static_ins.m` | 调整静态图线宽、图例、标签、字号和导出尺寸 |
| `.gitignore` | 增加 `/data/turn_on_bias/` 忽略规则 |
| 工具 README、新增分析脚本及说明 | 增加手动上电重复性分析；`analyze_turn_on_bias.py`、`TURN_ON_BIAS.md` 尚未提交 |
| 其他新增文件 | GM 自相关、零偏模型比较、时间窗口比较脚本及结果，以及 `.asv` 备份、PNG/MAT、DOCX 文件 |
| 已跟踪的标定结果及图表 | 多个 MAT、PNG 与 GitHub 文件字节不同，不能仅凭 Git diff 判断数值是否改变 |

STM32 还配置了 `upstream`，指向历史仓库 `stm32-mpu6050-allan-toolkit`。检查当前项目的 GitHub 版本用 `origin/main`，不要把历史仓库当作同一个比较基准。

## 3. 日后快速检查：PowerShell

每次先 fetch；失败时不要把旧的远端缓存称为 GitHub 最新版本。以下命令不执行 pull、checkout、reset、提交或推送。

```powershell
$repoPaths = @(
  "C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation",
  "C:\Users\12597\Desktop\lowcost\pi5-mpu6050-f9p-logger"
)

foreach ($repoPath in $repoPaths) {
  Write-Host "项目：$repoPath"
  git -C $repoPath fetch origin --prune
  if ($LASTEXITCODE -ne 0) { throw "获取 GitHub 版本失败：$repoPath" }
  git -C $repoPath remote -v
  git -C $repoPath branch -vv
  git -C $repoPath -c core.quotepath=false status --short --untracked-files=all
  git -C $repoPath rev-list --left-right --count HEAD...origin/main
  git -C $repoPath --no-pager log --oneline origin/main..HEAD
  git -C $repoPath --no-pager log --oneline HEAD..origin/main
  git -C $repoPath --no-pager diff --stat origin/main
  git -C $repoPath -c core.quotepath=false --no-pager diff --name-status origin/main
  git -C $repoPath -c core.quotepath=false ls-files --others --exclude-standard
}
```

`rev-list` 两列中，左侧为本地独有提交数，右侧为 GitHub 独有提交数。`0 0` 只说明提交历史一致，不排除工作目录有未提交修改或新增文件。

`status --short` 第一列表示暂存区相对 HEAD 的变化，第二列表示工作目录相对暂存区的变化。` M` 是未暂存修改，`M ` 是已暂存修改，`??` 是未跟踪文件。

## 4. 查看具体代码变化

```powershell
$repoPath = "C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation"

# GitHub 最新主分支与当前本地已跟踪文件的完整内容差异
git -C $repoPath --no-pager diff origin/main -- tools/allan_compare_tc_configs.py

# 未暂存修改：工作目录相对暂存区
git -C $repoPath --no-pager diff

# 已暂存修改：暂存区相对本地 HEAD
git -C $repoPath --no-pager diff --cached

# 本地已提交内容与远端提交内容之间的差异
git -C $repoPath --no-pager diff origin/main HEAD

# 两端独有提交的文字图示
git -C $repoPath --no-pager log --oneline --graph --left-right HEAD...origin/main
```

普通 `git diff` 不展示未跟踪文件，应配合 `git ls-files --others --exclude-standard` 并直接阅读新文件。无需为了查看新文件执行 git add。

`git diff origin/main...HEAD` 比较共同祖先到本地 HEAD，适合查看本地分支提交的变化；它不等于本地工作目录与 GitHub 最新内容的完整比较，也不包含未提交修改。

## 5. 忽略文件与二进制文件

```powershell
# 查看某个数据文件被哪条规则忽略
git -C $repoPath check-ignore -v data/turn_on_bias/20260930_3run_pilot/manifest.json

# 列出忽略文件，原始数据较多时输出可能很长
git -C $repoPath -c core.quotepath=false ls-files --others --ignored --exclude-standard
```

被忽略的原始会话、Allan 结果、试跑数据和缓存不会出现在普通 status/diff 中。忽略规则对已跟踪文件不生效，因此本次电动车会话内部分 PNG/MAT 仍会显示修改。

PNG 需要并排查看两份图像；MAT 需要加载两份文件，比较变量名、形状和数值。MAT 保存时间、文件头或压缩方式改变也可能造成二进制差异。SHA-256 只能判断字节是否一致。

## 6. 本次差异快照

下列清单在创建本文之前取样；本文自身是随后新增的未跟踪文件，不计入下面的新增清单。后续请重新 fetch 并运行上述命令。

### stm32-mpu6050-f9p-navigation

Local path: `C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation`.
Remote: [stm32-mpu6050-f9p-navigation](https://github.com/lmy91/stm32-mpu6050-f9p-navigation).
HEAD: `33c564691eb5718cb43b6939ec3185b112d91107`.
origin/main: `33c564691eb5718cb43b6939ec3185b112d91107`.
Ahead / behind: `0	0`.
Tracked changes: **28**; untracked files: **32**.

| Status | Tracked file relative to origin/main |
|---|---|
| M | `.gitignore` |
| M | `data/calib24/calib24_result_tempcomp_azgxgy.mat` |
| M | `data/calib24/fig_temp_order_selection.png` |
| M | `data/calib24/fig_tempcomp_azgxgy_gravity_error.png` |
| M | `data/calib24/fig_tempcomp_azgxgy_temp_drift.png` |
| M | `data/calib24/temp_coeffs_raw.mat` |
| M | `data/calib24/temp_order_selection.mat` |
| M | `data/decoded/20260923104556_电动车2/compare_acc_calibration_static_ins.m` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_00_psins_01.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_00_psins_02.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_00_psins_03.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_01_gnss_vs_ins_velocity.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_02_kf_bias.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_03_velocity_attitude_bias.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_04_gnss_speed_zupt.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_dyn_05_ve_zupt.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_01_velocity_error_axes.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_02_velocity_error_norm.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_03_position_error_axes.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_04_position_error_norm.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_05_attitude_error_axes.png` |
| M | `data/decoded/20260923104556_电动车2/figs/fig_static_06_attitude_error_norm.png` |
| M | `data/decoded/20260923104556_电动车2/static_ins_acc_cal_compare.mat` |
| M | `data/decoded/20260923104556_电动车2/test.m` |
| M | `tools/README.md` |
| M | `tools/README_EN.md` |
| M | `tools/allan_compare_tc_configs.py` |
| M | `tools/fit_temp_bias_raw.m` |

```text
28 files changed, 1005 insertions(+), 493 deletions(-)
```

Untracked files:

```text
data/decoded/20260923104556_电动车2/bias_model_comparison.mat
data/decoded/20260923104556_电动车2/bias_model_comparison_summary.csv
data/decoded/20260923104556_电动车2/compare_acc_calibration_static_ins.asv
data/decoded/20260923104556_电动车2/compare_bias_models.m
data/decoded/20260923104556_电动车2/compare_drift_time_windows.m
data/decoded/20260923104556_电动车2/figs/fig_ieee_01_position_trajectory.png
data/decoded/20260923104556_电动车2/figs/fig_ieee_02_velocity_components.png
data/decoded/20260923104556_电动车2/figs/fig_ieee_03_attitude_components.png
data/decoded/20260923104556_电动车2/figs/fig_ieee_04_bias.png
data/decoded/20260923104556_电动车2/figs/fig_windows_01_velocity_error_3d.png
data/decoded/20260923104556_电动车2/figs/fig_windows_02_position_error_3d.png
data/decoded/20260923104556_电动车2/figs_bias_model_comparison/fig_01_holdout_errors.png
data/decoded/20260923104556_电动车2/figs_bias_model_comparison/fig_02_metric_bars.png
data/decoded/20260923104556_电动车2/figs_bias_model_comparison/fig_03_bias_estimates.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_00_psins_01.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_00_psins_02.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_00_psins_03.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_01_gnss_vs_ins_velocity.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_02_kf_bias.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_03_velocity_attitude_bias.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_04_gnss_speed_zupt.png
data/decoded/20260923104556_电动车2/figs_bias_random_walk/fig_dyn_05_ve_zupt.png
data/decoded/20260923104556_电动车2/static_ins_drift_windows.mat
data/decoded/20260923104556_电动车2/static_ins_drift_windows_summary.csv
data/decoded/20260923104556_电动车2/test.asv
data/decoded/20260923104556_电动车2/test_bias_random_walk.m
docs/MPU6050_温度补偿与24位置标定总结_20260929.docx
tools/TURN_ON_BIAS.md
tools/analyze_turn_on_bias.py
tools/fit_temp_bias_raw.asv
tools/gm_autocorrelation_analysis.m
tools/gm_validate_parameters.m
```

### pi5-mpu6050-f9p-logger

Local path: `C:\Users\12597\Desktop\lowcost\pi5-mpu6050-f9p-logger`.
Remote: [pi5-mpu6050-f9p-logger](https://github.com/lmy91/pi5-mpu6050-f9p-logger).
HEAD: `2020c7106ac2696ee13bd75d4a18ae0e4fe5382e`.
origin/main: `2020c7106ac2696ee13bd75d4a18ae0e4fe5382e`.
Ahead / behind: `0	0`.
Tracked changes: **0**; untracked files: **0**.

No tracked changes.

No untracked files.
