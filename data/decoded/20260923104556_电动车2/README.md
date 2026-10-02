# 电动车采集会话完整示例

## 目录分类（2026-10-02）

代码已按用途移入 `code/`，说明报告移入 `docs/`，生成的 MAT 与汇总 CSV 按实验归入 `results/`。原始输入 CSV、基站观测和图表保留原来的位置。五个 MATLAB 脚本均从代码位置定位会话根目录，读取根目录数据；三个对比脚本自动将 MAT/CSV 写入对应结果子目录。

```text
20260923104556_电动车2/
├─ code/
│  ├─ navigation/
│  │  ├─ test.m                            当前导航回放入口
│  │  └─ test_bias_random_walk.m            历史随机游走回放
│  └─ experiments/
│     ├─ compare_bias_models.m             GM/RW 历史对比
│     ├─ compare_acc_calibration_static_ins.m  静止标定对比
│     └─ compare_drift_time_windows.m      静止漂移窗口对比
├─ docs/static_ins_acc_cal_summary.md      静止实验说明
├─ imu.csv / gnss.csv / rawx.csv / sync.csv
├─ f9p.ubx / rover.obs                     原始记录与流动站观测
├─ WUH*.rnx                               全天及匹配时段基站观测
├─ brdc2660.26p/                          广播星历
├─ results/
│  ├─ bias_model_comparison/              GM/RW 对比 MAT 与 CSV
│  ├─ static_ins_acc_cal/                 静止标定对比 MAT 与 CSV
│  └─ static_ins_drift_windows/           漂移窗口对比 MAT 与 CSV
├─ figs/ / figs_bias_*/                   图片
├─ kml/                                  轨迹
└─ README.md
```

WUH2 下载工具集中在仓库的 [tools/station_observation_downloader](../../../tools/station_observation_downloader/README.md)，无需在本目录保留下载代码。`.wuh2_pipeline/` 是本地下载状态缓存。

## GitHub 发布与基站观测

本示例发布 `WUH2_20260923_024615_030626_1s.rnx`（GPS 02:46:15～03:06:26，1,212 个一秒历元）和 `brdc2660.26p/BRDC00IGS_R_20262660000_01D_MN.rnx`（当天混合广播星历），可与 `rover.obs` 配合进行 RTKLIB 后处理。

全天观测 `WUH200CHN_R_20262660000_01D_01S_MO.rnx` 约 1.37 GB，超过 GitHub 普通 Git 单文件限制，仅留在本地，下载仓库后不会包含该文件。双击下载工具的 `run_gui.bat`，选择本目录的 `rover.obs` 并勾选“下载全天观测”即可获取。默认模式只需覆盖本次流动站的 02:45、03:00 两个分段；本地已有匹配时段文件时直接复用。下载缓存和中间文件不上传。

## 当前运行入口

本目录保留原始完整示例和既有实验资产。本版更新当前test并归档其他实验，后者不自动跟随test更新。

| 文件/目录 | 目的与使用限制 |
| --- | --- |
| [code/navigation/test.m](code/navigation/test.m) | 当前18状态PSINS回放：温补→标定，读取0930 Allan白噪声和自相关GM；实际dt传播、姿态映射Q、GNSS/ZUPT反馈，自动输出1 Hz KML |
| [code/navigation/test_bias_random_walk.m](code/navigation/test_bias_random_walk.m) | 既有RW版本，Allan RRW设过程噪声；未同步当前test全部实际dt/Q/KML改动 |
| [code/experiments/compare_bias_models.m](code/experiments/compare_bias_models.m) | 历史GM/RW留出实验，GM仍来自Allan BI/平台；留出120～180、300～360、420～480秒GNSS，不是当前自相关GM的受控比较 |
| [code/experiments/compare_drift_time_windows.m](code/experiments/compare_drift_time_windows.m) | 静止纯INS的1/10/30/60/180秒窗口误差，三种补偿层级 |
| [code/experiments/compare_acc_calibration_static_ins.m](code/experiments/compare_acc_calibration_static_ins.m) | 原始/温补/温补标定三路静止纯INS对照，非动态精度 |
| [results/bias_model_comparison/](results/bias_model_comparison/)、`figs_bias_model_comparison/` | 历史GM/RW实验的 MAT、CSV 指标和三张图 |
| [results/static_ins_drift_windows/](results/static_ins_drift_windows/) | 窗口实验 MAT 与 CSV 摘要，对应fig_windows图 |
| [results/static_ins_acc_cal/](results/static_ins_acc_cal/) | 静止三路标定对比 MAT 与 CSV 摘要 |
| `figs/` | 当前动态诊断、IEEE轨迹/速度/姿态/零偏、静止和窗口图，以前缀区分 |
| `figs_bias_random_walk/` | 独立RW回放图，不覆盖当前test图 |
| `kml/combined_navigation_1hz.kml` | 天线端WGS84轨迹，619个1 Hz点，默认贴地；Google Earth直接打开 |

运行依赖已上传：`data/calib24/`温补/标定MAT、
`data/allan_results/mpu_21.1h_20260930/allan_parameters.csv`、
`data/decoded/20260926005735/gm_autocorrelation/gm_parameters.csv`。
路径按仓库定位，不要求21 h原始CSV。GM仍为候选值，见 [验证说明](../../../tools/noise_analysis/GM_VALIDATION.md)。
P阵初始eb/db与GM过程参数不同；连续通电试跑不能作为正式上电重复性db。

在仓库根目录MATLAB，先安装并初始化外部PSINS，再运行：

```matlab
which glvs
which insupdate
run(fullfile('data','decoded','20260923104556_电动车2','code','navigation','test.m'));
```

其余入口按上表选择 `code/navigation` 或 `code/experiments` 中的脚本。脚本会clear/close并覆盖同名图和结果。
KML由连续传播日志按整数秒插值，不外推、不仅抽取反馈点，不做GCJ-02转换。
按海拔显示时将test顶部 `kml_altitude_mode='absolute'`，默认贴地但仍保留高度。
原始示例和KML包含实际道路位置；没有独立参考轨迹时，不将与已融合GNSS一致解释为绝对精度。

该目录保存 2026-09-23 的一组电动车实测会话，用于复现数据解码、松组合实验，以及静止区间的温补与 24 位置标定对照。原始记录与分析产物一并保留。

## 文件说明

| 文件 | 说明 |
|---|---|
| `f9p.ubx` | F9P 原始 UBX 数据 |
| `imu.csv` | 100 Hz IMU 物理量与原始量记录 |
| `gnss.csv` | GNSS 导航解 |
| `rawx.csv` | RAWX 逐星逐频观测 |
| `sync.csv` | 同步诊断记录 |
| `rover.obs` | RTKLIB 观测文件 |
| `code/navigation/test.m` | 当前电动车动态松组合实验脚本，已接入温补后加计标定输入 |
| `code/experiments/compare_acc_calibration_static_ins.m` | 从本会话静止区间做三路纯 INS 对照 |
| `results/static_ins_acc_cal/static_ins_acc_cal_summary.csv` | 三路静止传播汇总指标 |
| `results/static_ins_acc_cal/static_ins_acc_cal_compare.mat` | 完整中间变量和时序结果 |
| `figs/` | 动态诊断与IEEE图、静止三路对比及时间窗口图 |

## 参数依赖与运行方式

本目录 MATLAB 实验从仓库的 `data/calib24/` 读取：

- `temp_coeffs_raw.mat`：当前有效轴为 az、gx、gy，阶数为 3、5、5；
- `calib24_result_tempcomp_azgxgy.mat`：与上述温补矩阵绑定的 24 位置标定结果。

脚本会强制核对系数、参考温度、有效轴和标定文件，数据温度超出拟合范围时停止。运行还需要 PSINS 工具箱已加入 MATLAB 路径。

```powershell
matlab -batch "set(groot,'defaultFigureVisible','off'); run(fullfile(pwd,'data','decoded','20260923104556_电动车2','code','navigation','test.m'));"
matlab -batch "set(groot,'defaultFigureVisible','off'); run(fullfile(pwd,'data','decoded','20260923104556_电动车2','code','experiments','compare_acc_calibration_static_ins.m'));"
```

`test.m` 会将 PSINS 原始诊断图和 GNSS/INS 速度、滤波器零偏、姿态、车速及 ZUPT 对照图写入 `figs/fig_dyn_*.png`。静止脚本将速度、位置、姿态相对共同初始状态的变化定义为误差；每项分别输出“三轴误差”和“三维模值”两张 IEEE 单栏图，共 6 张 `fig_static_*.png`，并更新 CSV、MAT 和 Markdown 汇总。

## 结果边界

三路对比使用 GNSS 有效序号 20～200 对应的 183.2 秒静止区间：原始、仅温补、温补后加计标定。三路都用各自完整静止区间的陀螺均值做离线去零偏，然后只运行 `insupdate`，不使用 GNSS 更新、KF、ZUPT 或反馈。因此它主要检查加计确定性标定对静止传播的影响，不是动态轨迹精度，也不评价独立的陀螺常值零偏参数。

动态脚本是该次道路采集的算法回放示例，并非精度验收：图中滤波器零偏、航向跳变和停车段 ZUPT 状态用于暴露当前组合算法行为，不能在没有独立参考轨迹和误差统计的情况下解释为绝对定位精度。

详细指标和解释见 [静止传播对比说明](docs/static_ins_acc_cal_summary.md)。
