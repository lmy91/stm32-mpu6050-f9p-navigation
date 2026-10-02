# IMU 原始域多阶温补、24 位置标定与 Allan 验证

更新日期：2026-09-29。本说明取代旧版“先标定再温补”说明。

> **怎么操作** → 见 [温补标定 SOP](温补标定SOP.md)（正式流程、命令、参数契约与禁止事项）；本文解释**为什么**这样做（原理、公式与结论边界）。步骤①②已可用 `tools/calibration/run_tempcal_sop.m` 一键跑通。

## 唯一处理顺序

1. 用原始 21h 静态记录逐轴选择温度多项式阶数，然后拟合原始域温漂系数。
2. 对 24 位置记录逐样本温补，再计算加计标定矩阵、加计零偏和陀螺静态零偏。
3. 对待处理 IMU 原始物理量，先使用同一组温补系数，再使用配套标定参数。

不能混用旧版标定参数，也不能把已温补数据再次温补。当前完成的是离线流程；上位机组合导航输入尚未自动接入本次参数，后续接入必须遵循第 3 步。

## 本次数据与选阶

- 原始数据：data/decoded/20260926005735/imu.csv，7,605,132 点，21.1364 h。
- 标定数据：data/calib24/01.csv 至 24.csv；各文件首尾剔除 10 秒。
- 原始单位：加计 m/s²，陀螺 deg/h，温度 °C。
- 参考温度 Tref = 27.44764705882353 °C；拟合范围 27.4418～32.6888 °C。
- ax、ay、az、gx、gy、gz 阶数分别为 **2、1、3、5、5、5**。
- 选阶与系数拟合每 50 点取 1 点；应用温补和 Allan 分析使用全量数据。

选阶候选为 1～5 阶，使用训练残差标准差相邻阶改善低于 0.1% 的首个平台准则；同时保留 BIC 和块均值指标作为参考。它不是独立验证所得的“最优阶数”，也不等同于以 Allan 方差最小为目标。最高阶达到 5 不证明还应继续增加阶数。

## 公式与参数约定

令 dT = T − Tref。每轴拟合包含常数项，但只减去温度变化项：

    drift_i(T) = Σ(k=1…order_i) c_i,k × dT^k
    a_out = Ca × (a_raw − drift_acc(T) − ba)
    gyro_out_deg_h = gyro_raw_deg_h − drift_gyro(T) − gyroBiasMean_deg_h
    gyro_out_rad_s = gyro_out_deg_h × π / (180 × 3600)

系数矩阵为六行六列，行顺序 ax/ay/az/gx/gy/gz，列顺序 c5/c4/c3/c2/c1/c0，低阶模型的高阶系数补零。**温补时不减 c0**，以保留参考温度下的常值与重力，由标定阶段处理零偏。

先在原始单位完成整条公式，再把陀螺整体转换成 rad/s。不能只转换原始角速度，却仍直接减去 deg/h 单位的温漂或零偏。拟合温区外的补偿尚未经验证，当前分析脚本会拒绝越界数据。

Ca 是加计测量矩阵 Ma 的逆矩阵。对照图中的 scale 阶段仅除以 Ma 的对角项，用于隔离标度效应；full 阶段采用完整 Ca，包含轴间耦合修正。24 位置静态数据没有提供可靠的陀螺比例因子和非正交矩阵，陀螺只做温补与静态零偏扣除。

## Windows 复现命令

在 PowerShell 中进入仓库。本次已使用本机 MATLAB R2025b 和科学计算 Python 执行；下面提供可独立复现的虚拟环境方式（需要已安装 Python 3.11 或更高版本；首次安装依赖需要联网）。

```powershell
Set-Location 'C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation'
py -3 -m venv .venv-temp
.\.venv-temp\Scripts\python.exe -m pip install -r tools\requirements_temp_analysis.txt
```

依次执行选阶、拟合、温补后标定。任何一步报错，应停止并排查，不要继续使用旧产物。也可用一键驱动 `run_tempcal_sop.m` 跑完步骤①②（等价于下面三行）：

```powershell
& 'D:\MATLAB2025b\bin\matlab.exe' -wait -nosplash -batch "set(groot,'defaultFigureVisible','off'); run('tools/calibration/fit_temp_order_selection.m'); run('tools/calibration/fit_temp_bias_raw.m'); run('tools/calibration/calib24_static_numbered_tempcomp.m');"
if ($LASTEXITCODE -ne 0) { throw 'MATLAB pipeline failed' }
# 或：& 'D:\MATLAB2025b\bin\matlab.exe' -batch "addpath('tools'); setup_tools; run_tempcal_sop"
```

`run_tempcal_sop.m` 默认执行全部步骤；`run_tempcal_sop('skipStep1',true)` 可复用已有系数只重跑步骤②。

然后做步骤③三方案 Allan 对比：

```powershell
$env:OPENBLAS_NUM_THREADS='2'
$env:OMP_NUM_THREADS='2'
.\.venv-temp\Scripts\python.exe tools\noise_analysis\allan_compare_tc_configs.py data\decoded\20260926005735\imu.csv --coeff data\calib24\temp_coeffs_raw.csv --calib data\calib24\calib24_result_tempcomp_azgxgy.mat --enable 0,0,1,1,1,0
if ($LASTEXITCODE -ne 0) { throw 'Allan comparison failed' }
```

命令从仓库根目录执行，使用脚本默认的本次数据路径。更换数据时先修改两个 MATLAB 温补脚本的输入配置，并检查 Python 的 --help 输入参数；24 位置脚本同样需要配套数据。当前脚本会核对选阶来源、温区、参考温度及标定与温补参数的一致性。

## 输出与复用

data/calib24/ 下：

- temp_order_selection.mat、temp_order_selection.csv、温度模型阶数选择报告.md：选阶结果、评价指标和来源。
- temp_coeffs_raw.mat、temp_coeffs_raw.csv：原始域**通用温补系数矩阵**；列序 `[c5..c0]`，行序 ax/ay/az/gx/gy/gz，未拟合的轴整行为 0。当前配置 `axisOrder=[0 0 3 5 5 0]`，即矩阵里只保留 az/gx/gy 的系数，ax/ay/gz 整行为 0（等价于不温补）。
- calib24_result_tempcomp_azgxgy.mat：与当前温补模型配套的完整标定结果；包含温区、处理顺序与来源。
- calib24_summary_tempcomp_azgxgy.csv：各位置的复核结果。

data/decoded/20260926005735/imu_tempcomp_multiorder.csv 为仅温补的全量物理量数据。

正式 Allan 输出位于 `data/decoded/<session>/allan_compare_tc_configs/`，包含三方案曲线、固定 τ 数值、噪声参数和中文报告。该目录默认属于本地生成结果；如需把某次结果作为示例发布，应明确选择对应会话后单独加入 Git。

以上大体积分析产物在本地生成，不应假定已上传 Git 或部署到树莓派。

## 本次结果与结论

全部 7,605,132 点采用共同时间段和 tau 网格，没有发现断点，没有平滑、重采样或额外去趋势。Allan 计算内部去均值仅用于提高数值稳定性，不改变理论结果。

1000 秒附近的 Allan **偏差**，完整补偿相对原始变化：ax +18.58%、ay −0.37%、az −8.01%、gx −70.66%、gy −21.19%、gz +37.17%。因此不能写“六轴均改善”。方差变化必须用 (补偿后偏差/原始偏差)² − 1 计算。

24 位置重力模长标定残差 RMS：无温补标定 0.944800 mg，先温补再标定 1.016580 mg，并未改善。陀螺位置间离散还包含姿态变化下的地球自转等影响，不能仅凭该指标归因为温补过拟合。

去除常值零偏不改变 Allan 方差；标度调整会缩放曲线数值，不能直接等同于传感器随机噪声降低。长期曲线还可能包含热滞后、其他慢变误差和有限记录长度影响。21h 记录同时用于拟合与评价，因此本次是样本内比较；建议用独立温度循环静态记录验证，尤其关注 ax 与 gz 的长时结果，再决定是否部署当前高阶模型。

Allan 图表中的 mg 使用标准重力 9.80665 m/s² 换算；24 位置拟合采用当地重力 9.7935538578 m/s²。当前三方案结果见[温补配置 Allan 对比报告](../data/decoded/20260926005735/allan_compare_tc_configs/温补配置Allan对比报告.md)。

## 按轴温补开关与三方案 Allan 对比（azgxgy 配置，2026-09-29）

工况前提：GNSS 1 Hz 修正、失锁不超过 600 s；评价重点是 30～1000 s 的 Allan 行为，数小时尺度仅作参考。

按轴配置裁定 `axisOrder = [0 0 3 5 5 0]`（顺序 ax ay az gx gy gz；0 表示该轴不温补）：

- ax、ay：暂停温补（短中期收益不足/无收益），保留静态标定；
- az：保留 3 阶温补（100～1000 s 有改善，不因 3600 s 变差而否定）；
- gx、gy：保留 5 阶温补（中长尺度收益明显）；
- gz：暂停温补（100 s 起劣化；正式系数矩阵中该轴整行为 0）。

`tools/calibration/fit_temp_bias_raw.m` 通过 `cfg.axisOrder` 生成唯一权威系数矩阵；`tools/calibration/calib24_static_numbered_tempcomp.m` 根据矩阵的非零阶数自动确定有效轴和 `outTag`。当前输出为 `calib24_result_tempcomp_azgxgy.mat`、配套 CSV 与两张图，不覆盖其他配置的结果。配套标定结果（24 位置，ΔT = 0.71 °C）：陀螺位置间 STD gx 71.39→33.67、gy 42.48→41.48、gz 55.67→55.67（未补），单位 deg/h；重力模长 RMS 0.9448→1.0139 mg；`ba` 相对无温补版仅 Z 轴 +3.368 mg，X/Y 与无温补版完全一致。

三方案（raw / raw+TC / raw+TC+calib）Allan 对比用 `tools/noise_analysis/allan_compare_tc_configs.py`：

```powershell
.\.venv-temp\Scripts\python.exe tools\noise_analysis\allan_compare_tc_configs.py data\decoded\20260926005735\imu.csv --coeff data\calib24\temp_coeffs_raw.csv --calib data\calib24\calib24_result_tempcomp_azgxgy.mat --enable 0,0,1,1,1,0
```

输出在 `data/decoded/20260926005735/allan_compare_tc_configs/`（六轴三方案曲线图、固定 τ 表、噪声参数表、报告）。按数据时间戳得到的实际采样率 99.9477 Hz 计算，600 s Allan 偏差（raw → raw+TC → raw+TC+calib）：az 0.1603→0.1374→0.1346 mg（最终改善 16.0%）、gx 24.19→8.66→8.66 °/h（64.2%）、gy 10.20→8.84→8.84 °/h（13.3%）；gz 未启用温补，ax/ay 仅受静态标定矩阵的小幅尺度变换。az 在 3600 s 处约增加 5.2%，与“只保留 100～1000 s 收益”的裁定一致。陀螺 raw+TC 与 raw+TC+calib 的 Allan 曲线重合属预期：常值 `gb` 不改变 Allan 曲线；加速度的常数矩阵 `Ca` 会带来很小的尺度变化，标定的主要作用仍体现在均值、尺度因子和非正交误差层面。

边界：Allan 静态指标不等于失锁位置误差；最终确认需组合导航断星回放（尚未执行）。

## 历史工具

`fit_temp_bias_poly3.m`（旧标定域温补路线）与 `allan_compare_before_after.py` 已从正式工具树删除，历史内容仍可从 Git 旧提交查看。当前正式工具为 `allan_compare_tc_configs.py`（步骤③三方案对比）与 `run_tempcal_sop.m`（步骤①②一键驱动）。不要再使用旧 `imu_compensated.csv`、`imu_tempcomp_raw.csv` 或旧 `poly3` 参数。
