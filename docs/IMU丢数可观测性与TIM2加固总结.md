# IMU 丢数可观测性与 TIM2 加固总结

本文固定今天完成的「IMU 丢数可观测性」工作：从为什么原有检测不够，到 TIM2 输入捕获层的三处竞态修复，再到固件诊断输出、PC 端差分解析、`sync.csv` 落盘和 Qt 状态栏告警的完整链路，以及最终 11 分钟静止验收结论。

---

## 1. 背景与目标

本系统为 STM32F103C8T6 + MPU6050（100 Hz）+ ZED-F9P 组合导航实验平台。STM32 用 TIM2 的 CH1/CH2 硬件输入捕获分别锁存 F9P 1PPS 与 MPU6050 DATA_RDY 边沿，把每个 IMU 样本标记到统一 GPS 时间轴（见《时间同步方案总结》）。

**核心问题**：如何判断一段 100 Hz IMU 采集**有没有物理丢失 epoch**、丢在硬件捕获层、软件消费层还是串口输出层。

**目标**：让每段采集数据都能提供充分的丢样诊断证据链，而不再依赖「样本序号连续」或「`time_valid=1`」这类不充分的单点判据。

---

## 2. 为什么原有的丢数检测不够

改造前系统已有两个检测手段，但都不充分：

| 手段 | 局限 |
| --- | --- |
| `sample` 序号连续（Qt/采集器的 `lost` 计数） | 极端负载下新的 DATA_RDY 覆盖了尚未处理的捕获值，序号本身仍连续，不能揭示物理缺样 |
| `g_debug_interrupt_overruns`（固件已统计但未输出） | 它只统计「ISR 进入时 `g_data_ready` 仍为 1」的软件单槽 mailbox 覆盖，且从未接入状态协议 |
| `time_valid=1` | 只表示「继承到了最近一次有效 PPS 锚点」，不检查 PPS 新鲜度、TIM-TP 锁定状态，也不保证时间戳精度 |

更深一层的问题是：**硬件输入捕获的 overcapture 标志（CC2OF）根本没有被读取、统计、清除**。它是物理缺样的权威硬件证据，却完全沉默。

---

## 3. 硬件机制：TIM2 输入捕获与 overcapture

- TIM2 以 1 MHz 自由运行，16 位 CNT 每 65.536 ms 回卷一次，用 UIF 中断把时间轴扩展为 48 位。
- CH2 捕获 DATA_RDY 上升沿：硬件在同一时钟节拍把 CNT 锁存进 CCR2，并置 `CC2IF`。
- **读 CCR2 会自动清 CC2IF**（输入捕获模式的硬件行为）。
- 当 `CC2IF` 仍为 1（上次捕获尚未被服务）时又来了新边沿，CCR2 被覆盖，硬件置 `CC2OF`（overcapture）。

关键语义（来自 STM32F1 RM0008）：

> **`CC2OF` 是一位粘滞事件标志，不是计数器。** CPU 阻塞 35 ms 连丢 3 个 epoch，CC2OF 仍然只等于 1。它只能说明「至少发生过一次 overcapture」，无法恢复丢失数量。

因此最终结论必须是：**这套体系能提供充分的丢样诊断证据链，但不能精确恢复所有丢失 epoch 的数量**。

---

## 4. TIM2 ISR 的三处竞态修复

### 4.1 CCxIF stale-clear（软件实现造成，已彻底消除）

读 CCRx 已清 CCxIF，但 ISR 末尾若再依据旧 `status` 快照对 CCxIF 写 0，会把「读 CCR 之后新边沿刚置位的新 CCxIF」误清掉。**修法：清标志掩码中删除 CC1IF/CC2IF，只清 `UIF|CC1OF|CC2OF`。**

### 4.2 CCxOF/UIF stale-clear（软件实现造成，已彻底消除）

同理，若在 ISR 末尾依据旧快照清 CCxOF，会误清本次执行期间新产生的 overcapture。**修法：把 acknowledge 移到 ISR 开头**——读完 SR 快照立即清掉快照中的 `UIF|CCxOF`，之后再处理 CCR；尾部不再写 SR。

### 4.3 timestamp race（回卷边界，已修复）

若 TIM2 回卷发生在 `status = TIM2_SR` 快照之后、读 CCRx 之前，旧 `status` 里 UIF=0，时间戳会少算一次溢出（差 65.536 ms）。**修法：读 CCR 后补读一次 `TIM2_SR & TIM_UIF`，与初始快照 OR 后传入 `timer_capture_time()`。** 新产生的 UIF 只读不清，退出 ISR 后由 UIF 中断再次进入累加 `g_timer_overflows`。

### 4.4 半周期消歧的前提

`timer_capture_time` 用 `if ((status & TIM_UIF) && capture < 0x8000u) ++high` 判断捕获落在回卷前还是回卷后，隐含前提是**从 capture 到软件判定的最大延迟 < 半回卷周期 32.768 ms**。在当前 100 Hz、72 MHz、正常中断负载下远小于该值；只有 CPU 阻塞超 ~32.8 ms 才会歧义，但那已伴随 CC2OF / overrun / dt 跳变等更上层问题，不需在时间戳消歧这里单独解决。

---

## 5. 诊断量语义（已收敛）

| 指标 | 准确含义 |
| --- | --- |
| `g_debug_interrupt_count` | ISR 实际观察并处理的 CC2 capture 次数（**非物理边沿总数**） |
| `g_debug_interrupt_overruns` | 单槽软件 mailbox 尚未消费时又来了新 capture（**软件级覆盖**） |
| `g_debug_cc2_overcapture` | 软件观察到 `CC2OF` 的次数（**硬件级覆盖，事件标志 ≠ 丢失 epoch 数**） |
| `g_debug_dt_gap_count` | 相邻成功样本间隔 `dt > 15000 µs` 的累计数（**最终时间轴缺口**） |
| `g_debug_i2c_errors` | capture 后读取 MPU 数据失败的累计数 |
| `backlog = interrupt_count - sample_count` | 已捕获但尚未输出的在途事件数，正常在 0/1 间相位波动 |

---

## 6. 证据链模型（非一一对应）

四个异常增量共同构成诊断证据链，而非对每一个丢样做数学上的一一归类：

```text
interrupt_overruns 增量 > 0  →  主循环消费不及时，软件 mailbox 覆盖
cc2_overcapture   增量 > 0  →  TIM2 捕获服务不及时，CCR2 硬件覆盖
dt_gap_count      增量 > 0  →  最终保留的 capture 时间轴出现 epoch 缺口
i2c_errors        增量 > 0  →  capture 后 IMU 读取失败
sample_count 增量 < interrupt_count 增量  →  有捕获事件未形成成功输出样本
```

极端竞态或多个错误同时发生时，它们提供的是**证据链**，不能保证唯一归因。

**告警原则**：
- 只看相邻 `#sync` 之间的**周期增量**（无符号 32 位差分 `(cur - prev) & 0xFFFFFFFF`），不看累计绝对值；
- `backlog` 只显示、不设「>1 即报警」硬阈值，真正值得关注的是**是否持续扩大**（如 1,1,2,3,4…）。

---

## 7. 实施过程（四阶段）

### 第一阶段：TIM2 ISR 加固（冻结）

- 定义 `TIM_CC1OF (1u<<9)` / `TIM_CC2OF (1u<<10)`；
- 新增 `g_debug_cc2_overcapture`，在 ISR 开头 ack 后统计 `CC2OF`；
- 修复三处竞态（见第 4 节）；
- 保留 `g_debug_interrupt_overruns` 语义不变。

### 2A：固件诊断输出（冻结）

- 新增 `g_debug_dt_gap_count`，在 `dt > 15000u` 时递增；
- `# sync` 行末尾追加六个累计计数器：`sample_count / interrupt_count / interrupt_overruns / cc2_overcapture / dt_gap_count / i2c_errors`；
- 不改 IMU v3 数据行。

### 2B：PC 端解析落盘（冻结）

- `tools/capture_serial.py` 与 `host/imu_serial_qt.py` 各新增 `parse_sync` / `u32_delta`；
- 采集会话目录**始终生成 `sync.csv`**（每秒一行，13 列：`unix_ms, pps, 6 累计值, 4 增量, backlog`），独立于 `--save` 选择；
- Qt 侧 `_process_sync` 每秒即时 flush，保证异常退出最多损失约 1 秒诊断记录；
- `tools/check_sync.py` 只读核对 `sync.csv` 完整性。

### 2C：Qt 状态栏告警（冻结）

- 状态栏新增 `Capture` 标签：任一周期增量 > 0 显示红色 `Capture warning: SW overrun / HW overcapture / Timestamp gap / I2C error`，否则绿色 `Capture: OK · Backlog N`；
- 不自动停采、不弹模态框、不因累计计数非零而永久标红；
- 断开时重置标签并清空 `_prev_sync`，避免下次连接用旧 pps 做错误差分。

---

## 8. 最终验收

**最新约 11 分钟静止测试（会话 `20260920192407`）**：共采集 65,559 个 IMU 样本，样本序号连续，无 `dt > 15 ms` 异常，`time_valid` 全程有效；软件 mailbox overrun、TIM2 CC2 overcapture、时间戳 gap 和 I2C error 的周期增量均为 0，`backlog` 仅在 0/1 之间正常波动。

由此验证了从 **TIM2 硬件捕获 → 固件诊断输出 → PC 端差分解析 → `sync.csv` 落盘 → Qt 状态栏告警** 的整条 IMU 丢数可观测性链路均正常工作。

此前各阶段的分段验收：

| 阶段 | 验证 | 结果 |
| --- | --- | --- |
| 第一阶段 | 15 min 静止 | 89,962 样本，`lost=0`，dt 均值 10.0046 ms，`time_valid` 全程 1 |
| 2A | 串口直读 `# sync` | 六字段正确输出，`interrupt_count` 比 `sample_count` 恒多 1（在途事件） |
| 2B | 3.3 min Qt 采集 | `sync.csv` 200 行、13 列完整、四个 `d_*` 全 0、尾行完整 |
| 2C | 11 min Qt 采集 | 状态栏绿色 OK，四项增量全 0 |

---

## 9. 遗留事项与边界

1. **CC2OF 是事件标志，不是丢失计数**：多个 overcapture 可能折叠成一次观测，无法精确恢复丢失 epoch 数量。这是硬件特性，不是缺陷。
2. **`invalid=77` 来源未定位**：与 IMU 连续性无直接对应关系，需单独定位（尚未确认该字段在采集器代码的递增位置）。
3. **backlog 持续扩大趋势判定未做**：当前只显示 backlog，尚未实现「1,1,2,3,4…」的趋势告警。
4. **PPS 频偏精确测量未做**：`gps_tow_us` 每秒约 70 µs 的重对齐不能直接归因 STM32 晶振（MPU6050 时钟、构造、舍入、映射算法都参与），需直接统计 PPS 捕获间隔 `Δt_TIM2,PPS` 才能得到 MCU 定时器相对 GPS PPS 的频偏。
5. **亚毫秒时间精度**：固件未使用 TIM-TP 的 `towSubMS`/`qErr` 做亚毫秒修正；`time_valid` 也尚未检查 `TpNotLocked`/`qErrInvalid` 与 PPS 新鲜度（见 `KNOWN_ISSUES.md` P1）。

---

## 附：关键代码位置

| 功能 | 位置 |
| --- | --- |
| TIM2 ISR（ack 时序、CC2OF 统计、timestamp race 修复） | `firmware/Src/main.c` `TIM2_IRQHandler` |
| 诊断变量声明 | `firmware/Src/main.c`（`g_debug_*` 区） |
| `# sync` 六字段输出 | `firmware/Src/main.c` `print_sync` |
| dt>15ms 缺口计数 | `firmware/Src/main.c` 主循环（`dt` 计算后） |
| sync.csv 落盘 + 差分 | `tools/capture_serial.py`（`parse_sync`/`u32_delta`/`CsvRecorder`） |
| Qt 解析 + 状态栏告警 | `host/imu_serial_qt.py`（`_process_sync`/`_update_capture_status`） |
| sync.csv 核对工具 | `tools/check_sync.py` |
| 字段与语义文档 | `tools/README.md`、`host/README.md`、`firmware/README.md` |
