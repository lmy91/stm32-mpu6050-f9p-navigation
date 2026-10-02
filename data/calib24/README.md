# 24位置标定输入与冻结参数

[数据索引](../README.md) | [温补标定SOP](../../docs/温补标定SOP.md)

| 文件 | 目的 |
| --- | --- |
| `01.csv`～`24.csv` | 24个静止姿态的标准IMU输入；本版发布用于重标定 |
| `temp_coeffs_raw.mat`、`.csv` | 原始域多项式温补；MAT还含Tref、有效温区和来源 |
| `temp_order_selection.mat`、`.csv` | 逐轴1～5阶选择的比较统计，推荐不覆盖显式配置 |
| `温度模型阶数选择报告.md`、`fig_temp_order_selection.png` | 选阶人工复核依据 |
| `calib24_result_tempcomp_azgxgy.mat` | 与当前温补绑定的Ca、ba、陀螺均值及元数据 |
| `calib24_summary_tempcomp_azgxgy.csv` | 各姿态补偿与标定汇总 |
| `fig_tempcomp_azgxgy_gravity_error.png` | 标定前后重力模值误差 |
| `fig_tempcomp_azgxgy_temp_drift.png` | 各姿态温漂/补偿对照 |

当前温补阶数 `[ax ay az gx gy gz]=[0 0 3 5 5 0]`。
`coef`列为 `[c5 c4 c3 c2 c1 c0]`，行序保持传感器轴；补偿只使用温度项，不扣c0。
加计采用 `a_cal=Ca*(a_raw-drift(T)-ba)`，不得先标定后套原始域温补。
这些是离线test默认参数，不自动写入实时Python配置。

在根目录MATLAB运行：

```matlab
addpath('tools'); setup_tools;
run_tempcal_sop('skipStep1',true); % 复用温补，重做24位置标定；覆盖对应输出
```

完整选阶/拟合需另外恢复21 h原始 `imu.csv`，详见SOP。文件中保存的本机来源路径是历史元数据，
当前脚本按仓库相对路径查找运行输入，不需要复制历史绝对路径。
