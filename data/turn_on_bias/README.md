# IMU零偏重复性：工具与试跑归档

[数据索引](../README.md) | [完整采集/分析方法](../../tools/TURN_ON_BIAS.md)

已发布 `20260930_3run_pilot/` 的分析结果、实验清单和标定快照，未发布其runs原始数据。
此批次是 **IMU连续通电、仅树莓派重启的三轮对照**，不是经过确认的断电上电重复性；
`filter_init.json` 的初始化适用性为false，不能直接用于正式db。

| 文件/目录 | 目的 |
| --- | --- |
| `manifest.json` | 原实验轮次、时序、模式和文件指纹；其原始记录路径在新克隆中不存在 |
| `calibration/` | 本批次固定温补及24位置标定快照，保证统计使用同一模型 |
| `analysis/runs_summary.csv` | 每轮均值、温度、覆盖率和质量检查 |
| `analysis/bias_covariance.csv`、`gyro_bias_covariance.csv` | 加计和陀螺轮均值样本协方差 |
| `analysis/block_means.csv` | 段内固定时间块均值 |
| `analysis/filter_init.json` | 单位、坐标系、估计及初始化适用性，必须先检查适用性 |
| `analysis/manifest_snapshot.json` | 计算时清单快照 |
| `analysis/重复性报告.md`、三张PNG | 统计条件、结果与质量限制 |

重新分析要恢复各轮原始imu.csv和sync.csv，再在根目录运行：

```powershell
python tools/analyze_turn_on_bias.py --experiment data/turn_on_bias/20260930_3run_pilot --coeff data/turn_on_bias/20260930_3run_pilot/calibration/temp_coeffs_raw.mat --calib data/turn_on_bias/20260930_3run_pilot/calibration/calib24_result.mat
```

正式重复性需另建真正IMU断电上电的批次，固定姿态和等待时间、保存依据并采集至少30个合格周期。
加计跨轮mg标准差转MATLAB db的μg需乘1000，但本试跑的数值不满足正式使用条件。
