# MPU6050 温补 + 标定 工作 SOP

适用范围：MPU6050 原始物理量数据的**温度补偿系数标定**与**24 位置静态标定**。
理论背景、公式与历史结论见 [`IMU温度补偿使用说明.md`](IMU温度补偿使用说明.md)，本文只描述**怎么做**。

工况前提：MPU6050 不长时间独立工作——有 GNSS 时 1 Hz 修正，失锁不超过 600 s。
因此评价重点是 30～1000 s 的短中期表现，**不追求数小时静态最优**。

---

## 0. 唯一处理顺序（不得颠倒、不得跳步）

```
采集 → 解码得到 imu.csv（长时静态）与 01..24.csv（24 位置）
  ①  逐轴选阶 + 拟合原始域温补系数        → temp_coeffs_raw.mat
  ②  逐样本温补（按轴开关）+ 24 位置标定  → calib24_result_tempcomp_<tag>.mat
  ③  三方案 Allan 对比                    → allan_compare_tc_configs/
```

运行时的前端顺序固定为：

```
a_cal = Ca · (a_raw − drift_acc(T) − ba)
g_cal = g_raw − drift_gyr(T) − gb          （单位 deg/h，再按需转 rad/s）
```

先温补、后标定；**两者参数一一绑定，禁止跨配置混用**。

---

## 1. 前置条件

| 项 | 要求 |
| --- | --- |
| MATLAB | R2025b（或支持 `-batch` 的版本），路径示例 `D:\MATLAB2025b\bin\matlab.exe` |
| Python | 3.11+，`python -m venv .venv-temp` 后 `pip install -r tools/requirements_temp_analysis.txt` |
| 温补拟合数据 | 长时静态 `data/decoded/<session>/imu.csv`（本次 `20260926005735`，21.1 h @100 Hz） |
| 标定数据 | `data/calib24/01.csv` … `24.csv`（24 个方位，每个 ≥30 s 静止） |
| 单位约定 | 加计 m/s²、陀螺 deg/h、温度 °C；系数矩阵 `[c5 c4 c3 c2 c1 c0]`，行序 ax ay az gx gy gz |

首次安装依赖需要联网。所有命令在**仓库根目录**执行。

---

## 2. 步骤 ① — 逐轴选阶 + 温补系数矩阵

```powershell
& 'D:\MATLAB2025b\bin\matlab.exe' -wait -nosplash -batch "set(groot,'defaultFigureVisible','off'); run('tools/fit_temp_order_selection.m'); run('tools/fit_temp_bias_raw.m');"
if ($LASTEXITCODE -ne 0) { throw 'step 1 failed' }
```

**脚本**：`tools/fit_temp_order_selection.m`（选阶，仅供参考）→ `tools/fit_temp_bias_raw.m`（拟合系数）

**配置**（`fit_temp_bias_raw.m` 的 USER CONFIG 区，**唯一权威配置**）：

```matlab
cfg.axisOrder = [0 0 3 5 5 0];   % [ax ay az gx gy gz]；0 = 该轴不拟合，输出整行置 0
cfg.decim = 50;  cfg.blockSec = 600;  cfg.maxOrder = 5;
```

- `axisOrder(k) > 0` → 拟合到该阶；`== 0` → 该轴**整行写 0**（下游 `polyval` 得 0，等于不温补）。
- 选阶脚本 `fit_temp_order_selection.m` 的推荐阶数**只作参考打印**，不会覆盖 `cfg.axisOrder`。
- 要改变「补哪些轴」，改 `cfg.axisOrder` → 重跑本步 → 再重跑步骤②。

**产物**（`data/calib24/`）

| 文件 | 内容 |
| --- | --- |
| `temp_order_selection.mat` / `.csv` | 各轴 1–5 阶的残差、BIC/AIC、漂移峰峰值、块均值 std 与推荐阶数（**参考**） |
| `温度模型阶数选择报告.md`、`fig_temp_order_selection.png` | 人工复核用 |
| `temp_coeffs_raw.mat` | **唯一权威系数矩阵**：`coef`(6×6 `[c5..c0]`，未拟合轴整行为 0)、`TCpoly`(`[c5..c1 0]`，补偿即用)、`ord`/`axisOrder`、`maxOrder`、`Tref`、`Tmin`、`Tmax`、`domain='raw'`、`sourceFile/sourceBytes/sourceSamples/sourceTimeRange`、`decim` |
| `temp_coeffs_raw.csv` | 同一内容的表格形式（`axis,order,c0..c5,rms_residual`；未拟合轴 `order=0`、系数 0、残差空） |
| `data/decoded/<session>/imu_tempcomp_multiorder.csv` | 全分辨率"仅温补"数据（1.1 GB 级，供外部复用） |

**校验点**

- `fit_temp_order_selection.m` 判定：相邻阶训练残差 RMS 改善 < 0.1% 即收敛；
- `fit_temp_bias_raw.m` 对选阶文件只做**软提示**（并排打印推荐阶数 vs 你的 `axisOrder`；异源或异 `decim` 仅 warning），不再硬断言；
- 人工确认 `cfg.axisOrder` 与选阶报告的差异是否合理；确认 `Tmin/Tmax` 覆盖你实际使用温度；
- `Tref` 取首个温度采样点，**使用时必须沿用同一个 Tref**。

---

## 3. 步骤 ② — 按轴温补后进行 24 位置标定

**脚本**：`tools/calib24_static_numbered_tempcomp.m`
**配置**（脚本内 USER CONFIG 区）：

```matlab
cfg.outTag       = '';    % 留空 = 按系数矩阵非零轴自动派生（[0 0 3 5 5 0] -> 'azgxgy'）
cfg.useZeroOrder = false; % 固定 false：c0 不参与补偿
```

本步**不再有按轴开关** `cfg.tcEnable`：温补完全由步骤①产出的系数矩阵决定，
有效温补轴由矩阵 `ord>0` 自动派生（并打印核对），未拟合轴整行为 0 → 自动不补偿。

```powershell
& 'D:\MATLAB2025b\bin\matlab.exe' -wait -nosplash -batch "set(groot,'defaultFigureVisible','off'); run('tools/calib24_static_numbered_tempcomp.m');"
if ($LASTEXITCODE -ne 0) { throw 'step 2 failed' }
```

**产物**（`data/calib24/`，均带 `_<outTag>` 后缀）

| 文件 | 内容 |
| --- | --- |
| `calib24_result_tempcomp_azgxgy.mat` | `struct result`：温补版 `ba/Ma/Ca`、无温补版 `ba_noTC/Ma_noTC/Ca_noTC`、`gyroBiasMean_deg_h`、`tempCoeff/tempCoeffFull`、`tempOrder`（0=未拟合）、`tcActive`/`tcEnable`、`outTag`、`Tref`、`tempFitMin/Max`、`tempDomain/tempSourceFile`、`processingOrder` |
| `calib24_summary_tempcomp_azgxgy.csv` | 24 个位置的温度、方位误差、实际施加补偿量、均值等明细 |
| `fig_tempcomp_azgxgy_gravity_error.png` | 标定前 / 无温补标定 / 温补标定 的重力模长误差对比 |
| `fig_tempcomp_azgxgy_temp_drift.png` | 各位置温度与实际施加补偿量 |

**校验点**

- 脚本断言系数文件 `domain='raw'`、24 位置温度全部落在 `[Tmin,Tmax]` 内；
- 脚本打印的"有效温补轴"应与步骤① `cfg.axisOrder` 里 `>0` 的轴一致；
- 逐位置 `dir.err` 全部 OK（>20° 或 |a| 偏差 >100 mg 会标 CHECK）；
- 重力模长 RMS < 5 mg 为优秀；若比"无温补版"明显变差，说明该轴温补配置不适合，回到步骤 ① 调阶数或把该轴置 0。

**当前配置与实测**（`axisOrder = [0 0 3 5 5 0]`，ΔT = 0.71 °C）

| 指标 | 无温补 | azgxgy |
| --- | ---: | ---: |
| 陀螺位置间 STD gx / gy / gz (°/h) | 71.39 / 42.48 / 55.67 | 33.67 / 41.48 / 55.67（gz 未补） |
| 重力模长 RMS (mg) | 0.9448 | 1.0139 |
| `ba` 相对无温补 | — | 仅 Z +3.368 mg（X/Y 相同） |

---

## 4. 步骤 ③ — 三方案 Allan 对比

```powershell
.\.venv-temp\Scripts\python.exe tools\allan_compare_tc_configs.py `
    data\decoded\20260926005735\imu.csv `
    --coeff data\calib24\temp_coeffs_raw.csv `
    --calib data\calib24\calib24_result_tempcomp_azgxgy.mat `
    --enable 0,0,1,1,1,0
```

**脚本**：`tools/allan_compare_tc_configs.py`（算法库 `tools/allan_noise_identification.py`）

**三方案定义**（`--enable` 必须与 `--calib` 的配置一致；本步工具未改，仍用显式开关——其取值应等于步骤① `cfg.axisOrder > 0`，例如 `[0 0 3 5 5 0]` → `0,0,1,1,1,0`）

| 方案 | 含义 |
| --- | --- |
| `raw` | 原始 IMU 物理量，无任何补偿 |
| `raw+TC` | 仅按 `--enable` 指定的轴做温补 |
| `raw+TC+calib` | 在上者基础上套用 `--calib` 的 `ba/Ca/gb`（完整运行时前端） |

**产物**（默认 `data/decoded/<session>/allan_compare_tc_configs/`）

| 文件 | 内容 |
| --- | --- |
| `allan_three_configs.png` | 六轴三方案 Allan 曲线（mg / deg/h），虚线标出 30/60/180/300/600/1000/3600 s |
| `allan_adev_at_tau.csv` | 固定 τ 的 Allan 偏差 + 600 s 块均值漂移 std |
| `allan_three_configs_params.csv` | 三方案各自的 ARW/VRW、BI、RRW、Rate Ramp |
| `allan_deviation.csv` | 完整 Allan 曲线（SI 单位） |
| `温补配置Allan对比报告.md` | 结论、判读要点与局限 |

**判读约定**

- 关注 **30～1000 s**；数小时尺度只作参考；
- `raw+TC` 与 `raw+TC+calib` 的曲线**基本重合是正常的**——Allan 偏差先扣除均值，常值 `gb` 与常数矩阵 `Ca` 不改变曲线形状；标定的作用体现在均值/零偏层面；
- 未启用温补的轴，`raw` 与 `raw+TC` 完全一致（预期）；
- **Allan 静态指标 ≠ 失锁位置误差**。本步只回答"各方案在 30～1000 s 的静态平滑度"，600 s 失锁精度需另行用组合导航断星回放验证。

**当前结果**（600 s Allan 偏差）

| 轴 | raw | raw+TC | raw+TC+calib | 变化 |
| --- | ---: | ---: | ---: | ---: |
| az (mg) | 0.1603 | 0.1374 | 0.1346 | −16.0% |
| gx (°/h) | 24.20 | 8.66 | 8.66 | −64.2% |
| gy (°/h) | 10.20 | 8.84 | 8.84 | −13.4% |
| ax / ay / gz | — | 不变 | 不变 | 未启用温补 |

---

## 5. 参数契约（写代码时照抄）

1. **矩阵布局**：`coef` 6×6，列序 `[c5 c4 c3 c2 c1 c0]`（polyfit 约定），行序 ax ay az gx gy gz；未使用的高阶补 0。
2. **不补 c0**：补偿只用 `c1..c_o`；常数项必须显式给 0，即 `TCpoly = [coef(:,1:5), 0]`，否则 `polyval` 会把 `[c5..c1]` 当成低一阶多项式，少补一个 dT 幂。
3. **Tref 一致**：`dT = T − Tref`，Tref 来自系数文件，不得自行改用别的温度基准。
4. **温区限制**：只在 `[Tmin, Tmax]` 内补偿，越界应拒绝或重新拟合。
5. **单位**：加计 m/s²、陀螺 deg/h；整条公式在原始单位完成后再转 rad/s（不要只转原始角速度）。
6. **配置绑定**：温补矩阵与 `ba/Ca/gb` 必须来自同一次流程——矩阵由步骤① `cfg.axisOrder` 决定，步骤② 按其执行；`result.tempCoeffFile`、`result.tcActive` 可自检。

---

## 6. 禁止事项

- ❌ 混用不同 `outTag` 的标定参数（Z 轴会引入 ~3.4 mg 系统偏差）；
- ❌ 手工改 `temp_coeffs_raw.mat` 的某一行——「补哪些轴 / 几阶」应改 `cfg.axisOrder` 后重跑步骤①，否则 `ord`、矩阵与标定不再自洽；
- ❌ 对已温补的数据再次温补；
- ❌ 把旧 `poly3` 路由的系数（`temp_bias_poly3_coeffs.*`）用于本流程；
- ❌ 从旧提交中恢复已删除的历史脚本并混入正式流程；
- ❌ 把单次失锁表现当结论（需多温度点、多历元复核）。

---

## 7. 目录与文件

**SOP 脚本（5 个）**

| 文件 | 步骤 |
| --- | --- |
| `tools/fit_temp_order_selection.m` | ① 选阶 |
| `tools/fit_temp_bias_raw.m` | ① 拟合系数 |
| `tools/calib24_static_numbered_tempcomp.m` | ② 温补 + 24 位置标定 |
| `tools/allan_compare_tc_configs.py` | ③ 三方案 Allan 对比 |
| `tools/allan_noise_identification.py` | ③ 算法库（也可单独做噪声辨识） |

**辅助**：`tools/calib24_static_numbered.m`（纯静态标定，即"方案A 无温补"基线/不做温补时的运行时参数）、`tools/run_tempcal_sop.m`（一键跑 ①②）、`tools/requirements_temp_analysis.txt`。

**已移除**：旧标定域路线与旧 before/after 分析器不再属于正式工具树，历史结果由 Git 版本记录保留。

**原始数据一律不动**：`imu.csv`、`01–24.csv`、`f9p.ubx`、`data/allan_results/imu_noise/`（组合导航依赖的噪声参数）。
