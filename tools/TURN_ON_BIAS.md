# 手动上电采集后的 IMU 零偏重复性分析

## 发布范围（2026-09-30）

三轮连续通电试跑的报告、摘要、图、实验清单和标定快照已归档，见 [数据README](../data/turn_on_bias/README.md)。试跑runs原始文件和preflight未上传，新克隆可读统计但不能直接重分析该批次。这不是正式断电上电重复性，不能直接用于db；正式使用需按本文另采独立上电周期。

保留工具 `analyze_turn_on_bias.py`，用于读取已经保存的静态数据，逐样本温补、应用固定的 24 位置标定矩阵，并计算各轮均值的样本协方差。它只分析文件，不控制树莓派、不重启设备、不切换电源。

## 手动采集

1. 刚性固定 IMU，全程保持同一姿态，不重新摆放设备或线缆。
2. 每轮实际切断 MPU 电源，再恢复供电；记录断电/上电时间和冷热启动条件。树莓派软件重启不作为断电依据。
3. 使用相同的上电至开始采集等待时间。每轮独立保存一个会话，并在结束保存、文件关闭后复制数据。
4. 建议至少 30 个合格上电周期。先采集少量会话检查采样连续性、温度范围和窗口稳定性。

按以下结构整理文件：

```text
data/turn_on_bias/manual_batch/
├── manifest.json
└── runs/
    ├── 01/imu.csv、sync.csv
    ├── 02/imu.csv、sync.csv
    └── ...
```

`imu.csv` 使用项目现有标准物理量表头。保留 `sync.csv` 供同步质量检查；GNSS、RAWX、UBX 可以一并归档，但计算加表重复性不要求它们。

## 实验清单

在实验目录创建 UTF-8 的 `manifest.json`。以下是两轮的结构示例：

```json
{
  "schema_version": 1,
  "mode": "imu-power-cycle",
  "settings": {"warmup_seconds": 30, "window_seconds": 120},
  "runs": [
    {"index": 1, "status": "collected", "local_session": "runs/01", "power_cycle_verified": false},
    {"index": 2, "status": "collected", "local_session": "runs/02", "power_cycle_verified": false}
  ]
}
```

`power_cycle_verified` 只有在确认该轮 MPU 真正断电再上电后才设为 `true`；这个字段是人工确认记录，不是脚本测量电压的结果。可以在各轮补充 `power_off_at`、`power_on_at` 和备注以保存依据。连续通电对照使用 `mode: "pi-reboot-continuous-imu"`，不要与上电实验混入同一批次。

`warmup_seconds` 从各会话第一条已采集 IMU 样本计时，不是从上电时刻计时。比如先上电等 30 秒才开始记录，再跳过 30 秒数据，则统计窗口约从上电后 60 秒开始。应按滤波器实际启动流程选择时序，保存完整记录，至少覆盖过渡段加统计窗口并留数秒余量。

可以给各轮增加 `files` 数组，保存文件名、字节数和 SHA-256，用于分析前验证数据完整性：

```json
"files": [
  {"name": "imu.csv", "size": 123456, "sha256": "填写该文件的真实SHA256"},
  {"name": "sync.csv", "size": 1234, "sha256": "填写该文件的真实SHA256"}
]
```

这里的长度和哈希仅是结构示例，必须替换为实际值。Windows 可使用 `Get-Item` 查看 `Length`，使用 `Get-FileHash -Algorithm SHA256` 获取哈希。没有 `files` 时仍能统计，但报告会提示缺少数据指纹，初始化适用性不会自动判为通过。

## 运行分析

在 STM32 项目根目录执行：

```powershell
& D:\anaconda\envs\allan-toolkit\python.exe tools\analyze_turn_on_bias.py `
  --experiment data\turn_on_bias\manual_batch `
  --coeff data\calib24\temp_coeffs_raw.csv `
  --calib data\calib24\calib24_result_tempcomp_azgxgy.mat
```

依赖见 `requirements_temp_analysis.txt`。可替换为其他已安装 NumPy、SciPy、pandas、Matplotlib 的 Python 环境。温补 CSV 需要同名 MAT 提供参考温度和有效温区；也可以直接传温补 MAT。建议将这组温补及标定文件复制到实验目录归档，并在命令中指向快照，保证整个批次使用固定模型。

## 补偿、质量检查和统计

每个样本先在原始域进行温度多项式补偿（不扣 c0），再执行 `a_cal = Ca * (a_tc - ba)`。温补与标定文件的模型、参考温度和启用轴必须一致；统计窗口超出有效温区时不外推，该轮判为无效。

对每轮固定窗口取三轴均值，计算跨轮无偏样本协方差（分母为有效轮数减一）。对角线平方根是 1σ，不再除以轮数。单轮样本标准差、分块均值变化、采样缺口和同步异常另行报告。

保持同一姿态时，恒定重力投影不影响跨轮协方差。若每轮用同一加速度数据重新估计倾角再减重力，水平零偏可能被吸收到倾角里，不能以此独立识别三轴绝对零偏。

输出位于标定后的传感器坐标系：协方差单位 `(m/s²)²`，1σ 单位 `m/s²` 和 mg。MATLAB 的 `db` 若使用 μg，应将输出 mg 乘以 1000；实时 Python 配置 `initial_accel_bias_std_mg` 使用 mg。只有实际上电重复性、等待时间、初始化补偿方式和坐标系与滤波器相符时，才能将结果用于初始零偏不确定度。

## 输出与既有试跑

实验目录的 `analysis/` 保存每轮汇总、加表/陀螺协方差、`filter_init.json`、分块均值、三张图、`重复性报告.md` 和分析时实验清单快照。至少 30 个有效且已确认的上电周期、可核查的文件指纹、同步诊断及无质量警告，是程序判定初始化适用性的必要条件；实验条件仍需人工确认。

此前 `20260930_3run_pilot` 的原始数据与报告继续保留。该批次是连续通电对照，不能改名后当作正式上电重复性数据。可直接用保留的 Python 工具重新分析其 `manifest.json`。
