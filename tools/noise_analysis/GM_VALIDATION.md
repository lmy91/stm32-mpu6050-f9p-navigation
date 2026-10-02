# GM 参数复核与实际时间传播

## 发布范围（2026-09-30）

训练GM的CSV/MAT/图和本次验证CSV/MAT/图已上传。21 h训练原始CSV和两份独立记录未上传，新克隆可直接读结果或用replotOnly重绘。默认完整运行需要另行恢复原始数据；缺失记录不算验证通过。已有三方案Allan结果也已上传，可按下文显式传空独立记录，只复核训练分段和尺度。本次整理不重新拟合、不宣布验证通过。

## 运行复核

在仓库根目录的 MATLAB 中运行：

```matlab
addpath(fullfile(pwd, 'tools')); setup_tools;
results = gm_validate_parameters;
```

已有 `gm_validation.mat` 时，只重绘图、不重新扫描原始CSV：

```matlab
gm_validate_parameters(struct('replotOnly', true));
```

函数冻结现有 GM、温补和24位置标定参数，不重新拟合温补，不覆盖
`gm_autocorrelation/gm_parameters.csv`。结果写入21小时记录的 `gm_validation/`。

默认检查包括：同一训练记录的前后半、四分段和线性去趋势敏感性；
30、60、120、300、600秒的“GM＋白噪声”Allan偏差；两份未参与当前温补、
标定或GM训练的固定姿态采集记录。白噪声使用 `raw+TC+calib` 数据域。
训练记录120秒点由全速率Allan曲线对数插值，表格的 `EmpiricalSource` 会注明，
其他已有固定时间点优先使用原始全速率实测结果。

默认独立窗口使用原CSV的 `time_s` 时间域，不是GPS周秒：

| 记录 | 连续窗口 | 用途 |
|---|---|---|
| 20260921140624 | 1800～29000秒，7.56小时 | 主要独立30～600秒检验；避开采集开头30分钟 |
| 20260921121352 | 60～4400秒，1.21小时 | 补充短尺度检验，长平均时间统计不足需标记 |

两份记录均已检查固定姿态候选质量、样本/时间连续性、温区和削顶。
长记录后段温度低于当前温补适用范围，不能删除超区点后拼接。
它们是独立采集记录，但样本号延续，不能据此证明独立断电上电或零偏重复性。
加计方向稳定性也不能单独区分微小夹具倾斜与零偏变化，结果不是零偏真值测量。

主要输出：

- `gm_validation_summary.csv`：各轴综合状态。
- `gm_validation_segments.csv`：分段/趋势敏感性与独立记录的探索性ACF拟合。
- `gm_validation_allan.csv`：模型预测、实测、比值、采样来源与统计量。
- `gm_validation_records.csv`：独立窗口、温区和质量依据。
- `gm_validation_sanity.csv`：数值稳定性、白噪声、趋势和OU过程自检。
- `gm_validation_allan.png`、`gm_validation_sensitivity.png`、
  `gm_validation_training_series.png`：尺度对照、敏感性和训练残差图。
- `gm_validation.mat`：结果以及本次冻结的温补/标定参数快照。

独立记录采用1秒块均值以节省内存；30秒以上Allan计算跨越连续整数块，
不是用10秒块估计短端。分段训练敏感性仍使用原10秒块，因此可与原拟合比较。
短记录与长相关时间不匹配时，工具标记 `Insufficient...`，不能解读为验证通过。
`sigmaRatioBounds`、`tauRatioBounds`、`allanRatioBounds` 都是可调整的工程筛选阈值，
不是统计置信区间；所有参数仍为候选参数。

只运行已有训练结果的分段和尺度检查，可跳过原始独立CSV读取：

```matlab
cfg = struct('independentRecords', struct([]), ...
    'outDir', fullfile(pwd, 'data', 'decoded', '20260926005735', 'gm_validation_training_only'));
results = gm_validate_parameters(cfg);
```

## test 的传播和Q

`data/decoded/20260923104556_电动车2/code/navigation/test.m` 使用每条 `dt_s` 构成角/速度增量，
逐双样本区间同步设置 `ins.ts`、`ins.nts`、`kf.nts` 和 `Qk=Qt*sum(dt_s)`。
PSINS的Phi离散方式保持不变，使用实际区间长度。静止陀螺补偿先估计时间加权
角速率，再乘各样本自己的积分时长；不再给不同时长样本扣同一个平均角增量。

两条时长不等时，在该样本对内按线性角速率/比力重建等时长虚拟半段增量，
保持总角增量和总速度增量，并保留双样本圆锥/划桨补偿；不跨样本对重采样。
时长比大于4时停止并提示核查缺口。末尾单样本也纳入传播。

白噪声通过区间中点姿态映射：`Gammak` 的姿态、速度块分别为 `-Cnb`、`Cnb`，
而GM零偏驱动仍在机体系。`Qt` 保留机体系噪声密度，避免重复旋转。
ZUPT判据和R不变，仅改为按实际时间调度，维持原有效5Hz。

这两项修正不处理GNSS高度慢漂、加计饱和或“常值残余＋GM”的状态定义问题。
结果改善与否必须检查创新，不能用轨迹贴合已融合GNSS代替独立精度验证。
