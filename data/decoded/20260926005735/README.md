# 21 h静态会话：精选分析结果

[数据索引](../../README.md) | [GM验证说明](../../../tools/noise_analysis/GM_VALIDATION.md)

本目录保存20260926005735静态记录的脚本和正式分析产物，
**不包含**原始 `imu.csv`、RAWX/UBX或全速率 `imu_tempcomp_multiorder.csv`。
这些大文件仍在本地。不得把本目录当作完整原始数据包。

| 文件/目录 | 目的/使用 |
| --- | --- |
| `plot_temp_drift_21h.m` | 原始IMU/温度时序绘图；补齐同目录imu.csv后运行，会写figs |
| `figs/fig_21h_*.png` | 长时间加计/陀螺与温度关系 |
| `fig_raw_temp_fit.png`、`fig_raw_tempcomp_compare.png` | 原始域多阶温补拟合及块均值对比 |
| `allan_compare_tc_configs/` | 已有raw、TC、TC+calib三方案曲线/参数/图和报告 |
| `allan_raw_tc/` | 新版工具选择raw/TC两阶段的曲线、固定tau摘要、参数、图和报告 |
| `gm_autocorrelation/gm_parameters.csv` | 六轴连续GM候选标准差与相关时间；test实际运行依赖 |
| `gm_autocorrelation/gm_block_series.csv` | 10秒温补标定后块均值，用于诊断趋势与平稳性 |
| `gm_autocorrelation/gm_results.mat` | 参数、序列、元数据、拟合配置，供训练分段复核 |
| `gm_autocorrelation/*.png`、`*.fig` | 自相关拟合、对称协方差、块序列；FIG用MATLAB打开 |
| `gm_validation/*.csv` | 分段/独立记录/30～600秒尺度复核与数值自检 |
| `gm_validation/gm_validation.mat` | 复核结果及冻结模型，用于无需原始数据的重绘 |
| `gm_validation/*.png` | 尺度、趋势敏感性和训练序列图 |

在根目录，仅重绘已有验证：

```matlab
addpath('tools'); setup_tools;
gm_validate_parameters(struct('replotOnly',true));
```

重拟合需恢复本目录imu.csv，运行 `run('tools/noise_analysis/gm_autocorrelation_analysis.m')`。
默认独立验证还需恢复20260921140624和20260921121352的静态原始记录。
10秒平均只抑制短端白噪声，工具做块平均传递函数修正；不能保证单阶GM适用于30～600秒。
参数要与验证summary一起阅读，不能只看自相关曲线拟合。
