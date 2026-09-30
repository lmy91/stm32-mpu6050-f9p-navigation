# Allan随机误差结果

[数据索引](../README.md)

本版保留两套不同用途的冻结结果，其他历史运行和兼容性测试留在本地。

| 目录 | 目的 |
| --- | --- |
| `imu_noise/` | 2026-09-10旧参考，解释实时fusion JSON中的噪声初值 |
| `mpu_21.1h_20260930/` | 当前离线test的Allan参数来源，按当前有效轴温补的数据域 |

每套目录：
`allan_parameters.csv` 为轴向噪声辨识；
`allan_deviation.csv` 为曲线数据；
`allan_deviation.png`、`allan_identification.png`、`stability_overview.png` 为图；
`随机误差判读报告.md` 保存处理条件、参数和统计限制。

当前test只从0930参数中读取ARW/VRW白噪声，GM另读自相关CSV。
实时Python读取JSON数值，不是在启动时自动载入这里的CSV。
需要重新生成时，先准备与报告一致的数据域/预热截取，
运行 `python tools/allan_noise_identification.py <IMU路径> --rate 100 --skip-minutes 30`，
并用 `--help`核对输出目录参数；不要对已温补文件再次温补。
原始长数据未上传，现有结果可查看但不代表新克隆可直接全流程重算。
