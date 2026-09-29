# 电动车采集会话完整示例

该目录保存 2026-09-23 的一组电动车实测会话，用于复现数据解码、松组合实验，以及静止区间的温补与 24 位置标定对照。原始记录与分析产物一并保留；两个 `*.bak_20260929` 文件是过期代码备份，不属于示例。

## 文件说明

| 文件 | 说明 |
|---|---|
| `f9p.ubx` | F9P 原始 UBX 数据 |
| `imu.csv` | 100 Hz IMU 物理量与原始量记录 |
| `gnss.csv` | GNSS 导航解 |
| `rawx.csv` | RAWX 逐星逐频观测 |
| `sync.csv` | 同步诊断记录 |
| `rover.obs` | RTKLIB 观测文件 |
| `test.m` | 当前电动车动态松组合实验脚本，已接入温补后加计标定输入 |
| `compare_acc_calibration_static_ins.m` | 从本会话静止区间做三路纯 INS 对照 |
| `static_ins_acc_cal_summary.csv` | 三路静止传播汇总指标 |
| `static_ins_acc_cal_compare.mat` | 完整中间变量和时序结果 |
| `figs/` | 动态松组合 8 张图、静止三路误差对比 6 张 IEEE 风格图 |

## 参数依赖与运行方式

两个 MATLAB 脚本从仓库的 `data/calib24/` 读取：

- `temp_coeffs_raw.mat`：当前有效轴为 az、gx、gy，阶数为 3、5、5；
- `calib24_result_tempcomp_azgxgy.mat`：与上述温补矩阵绑定的 24 位置标定结果。

脚本会强制核对系数、参考温度、有效轴和标定文件，数据温度超出拟合范围时停止。运行还需要 PSINS 工具箱已加入 MATLAB 路径。

```powershell
matlab -batch "set(groot,'defaultFigureVisible','off'); run(fullfile(pwd,'data','decoded','20260923104556_电动车2','test.m'));"
matlab -batch "set(groot,'defaultFigureVisible','off'); run(fullfile(pwd,'data','decoded','20260923104556_电动车2','compare_acc_calibration_static_ins.m'));"
```

`test.m` 会将 PSINS 原始诊断图和 GNSS/INS 速度、滤波器零偏、姿态、车速及 ZUPT 对照图写入 `figs/fig_dyn_*.png`。静止脚本将速度、位置、姿态相对共同初始状态的变化定义为误差；每项分别输出“三轴误差”和“三维模值”两张 IEEE 单栏图，共 6 张 `fig_static_*.png`，并更新 CSV、MAT 和 Markdown 汇总。

## 结果边界

三路对比使用 GNSS 有效序号 20～200 对应的 183.2 秒静止区间：原始、仅温补、温补后加计标定。三路都用各自完整静止区间的陀螺均值做离线去零偏，然后只运行 `insupdate`，不使用 GNSS 更新、KF、ZUPT 或反馈。因此它主要检查加计确定性标定对静止传播的影响，不是动态轨迹精度，也不评价独立的陀螺常值零偏参数。

动态脚本是该次道路采集的算法回放示例，并非精度验收：图中滤波器零偏、航向跳变和停车段 ZUPT 状态用于暴露当前组合算法行为，不能在没有独立参考轨迹和误差统计的情况下解释为绝对定位精度。

详细指标和解释见 [静止传播对比说明](static_ins_acc_cal_summary.md)。
