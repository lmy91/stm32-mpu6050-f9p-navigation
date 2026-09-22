# Host RX 背压故障归零报告

| 项目 | 内容 |
| --- | --- |
| 故障名称 | Qt 上位机长时间采集触发「接收队列已达 2 MiB」保护性停采 |
| 故障部位 | `host/serial_worker.py` 串口接收背压队列 |
| 故障时间 | 2026-09-20 19:54:40 ~ 2026-09-21 07:03:37（运行约 11 h 9 min 后触发） |
| 报告日期 | 2026-09-21 |
| 归零类型 | 技术归零（定位准确、机理清楚、问题复现、措施有效、举一反三） |
| 归零状态 | 最终关闭（21 h 长测验收通过） |

---

## 一、故障现象

2026-09-20 19:54:40 启动 Qt 上位机采集（COM8 / 460800 bit/s，勾选 IMU/GNSS/RAWX/AIM/NAV/LOG），连续运行至 2026-09-21 07:03:37，`event.log` 报出：

```
[ERROR] [链路] 串口异常：界面/记录处理停顿，接收队列已达 2 MiB；停止采集以免静默丢数据
[ERROR] [串口] 串口异常：界面/记录处理停顿，接收队列已达 2 MiB；正在关闭数据文件。
```

关键现象特征：

- 触发瞬间 `采集丢帧/无效行 = 0/0`，但该计数仅反映已落盘数据的解析结果；
- 系统为「主动停止并报警」，而非静默丢弃数据；
- 检查已落盘的 11 h 数据：IMU 4,012,116 样本、`sample` 序号连续（跳号 0）、`dt` 稳定在 10.004–10.005 ms、`time_valid` 全程有效、四个诊断量周期增量均为 0。

**结论：检查已落盘数据，停采前未发现 IMU 丢样或时间轴异常证据；故障表现为主机侧保护性停采，而非已证实的持续性数据丢失。保护触发瞬间及之后的尾部数据不作完整性保证。**

---

## 二、故障定位（定位准确）

采用故障树自顶向下逐层定位，最终收敛到单一代码行。

### 2.1 第一层：排除「已证实数据丢失」假设

核对 11 h 已落盘采集数据（样本序号连续、dt 稳定、time_valid 有效），未发现 IMU 丢样或时间轴异常证据。→ 故障性质聚焦为「主机侧保护性停采」，而非「已证实的持续性数据丢失」。

### 2.2 第二层：定位报错触发点

报错由 `host/serial_worker.py` 的 `_read()` 抛出。数据流为：

```text
串口接收线程 --put_nowait--> received 队列 --take_received--> Qt 主线程(解析 + 写 CSV)
```

主线程消费速率低于串口到达速率时，`received` 队列积压至上限，`put_nowait` 抛出 `queue.Full`，触发停采。

### 2.3 第三层：识别阈值语义错误（根本定位）

定位到队列定义：

```python
self.received = queue.Queue(maxsize=512)  # each chunk <=4096 bytes: 2 MiB
```

`queue.Queue(maxsize=512)` 的 `maxsize` 限制的是**队列元素（chunk）个数 = 512**，并非字节数。注释「each chunk <=4096 bytes: 2 MiB」错误地假设每个 chunk 均为满 4096 字节。而 `_read()` 实际执行 `self.port.read(min(waiting, 4096))`，串口线程每 2 ms 轮询一次，绝大多数 chunk 远小于 4096 字节。

→ 报错文案「已达 2 MiB」与真实字节积压量不符，**故障定位为「背压阈值单位语义错误」**。

### 2.4 第四层：区分「阈值过敏」与「吞吐不足」

系统能连续稳定运行约 11 h 才触发一次，且停采前已落盘数据无丢样证据，未发现主线程持续性吞吐不足的迹象。故障更符合**缓冲策略使用了错误的计量单位、过于敏感**。

**定位结论**：`host/serial_worker.py` 接收队列以「chunk 数（512）」冒充「字节数（2 MiB）」作为背压上限，导致偶发主线程停顿（磁盘 IO 等）时 chunk 数短时堆叠至 512 而误触发停采。

---

## 三、机理分析（机理清楚）

### 3.1 触发机理

1. 主线程（串口解析 + CSV 写盘 + GUI 刷新共用一个线程）偶发阻塞（磁盘 IO、系统调度、Defender 等），`take_received()` 暂停；
2. 串口线程持续 `put_nowait()` 写入 chunk，`received` 队列内 chunk 数累积；
3. 当 chunk 数达到 `maxsize=512` 时，`put_nowait` 抛出 `queue.Full`；
4. `_read()` 捕获后抛出 OSError，串口线程 `failed` 信号触发 `disconnect_serial`，停止采集。

### 3.2 单位语义错误分析

- `queue.Queue(maxsize=512)` 的 512 是**元素个数上限**；
- 只有「每个元素恰好 4096 B」时，512 × 4096 B = 2 MiB 才成立；
- 实际 `in_waiting` 反映「此刻缓冲区内可用字节数」，2 ms 轮询周期下通常远小于 4096 B，故单 chunk 通常远小于 4096 B；
- 因此「512 个 chunk」对应的真实字节积压可能仅几十 KiB，远未达到 2 MiB。

### 3.3 与实测数据的一致性

修复后引入字节精确观测，1.5 h 短测（会话 `20260921121352`）测得：

- `Host RX current` 常态 ≈ 0 B，最大值 4.9 KiB；
- `Host RX peak` = 21.6 KiB，约为 2 MiB 的 1.05%。

该数据与「旧阈值过敏、真实积压极小」的机理解释一致；**未发现主线程持续性吞吐不足的证据，并强烈支持旧版停采由 chunk-count 阈值过敏触发**（1.5 h 短测能反驳「持续跟不上」，但不能排除所有 12–24 h 长时场景）。

---

## 四、问题复现 / 机理替代验证

### 4.1 复现方法

原代码未记录「触发瞬间的真实字节 backlog」，无法对旧事故做完全等价复现。故采用**替代验证**：修复后引入字节精确观测，在相同采集条件下短测，观察真实积压水平是否接近旧阈值。

### 4.2 复现结果

1.5 h 短测（同环境、同勾选项）结果：

| 观测项 | 结果 |
| --- | --- |
| Host RX peak | 21.6 KiB（2 MiB 的 1.05%） |
| Host RX current | 常态 ≈ 0，最大 4.9 KiB |
| 四个 d_* | 全 0 |
| backlog | 0/1 波动 |
| 停采告警 | 无 |
| ERROR / WARN | 0 |

**复现结论**：真实 RX 积压远低于 2 MiB，强烈支持旧事故直接触发原因是「512-chunk 假 2 MiB 阈值过敏」，而非「持续性主线程吞吐不足」。因旧事故瞬间真实字节量未记录，本项归因严格表述为「强烈支持」而非「唯一归因」。

---

## 五、纠正措施（措施有效）

| 序号 | 措施 | 内容 |
| --- | --- | --- |
| 1 | 字节精确背压 | `queue.Queue(maxsize=512)` 改为 `deque + threading.Lock + _rx_bytes + _rx_peak_bytes`，`RX_BACKLOG_LIMIT = 2 MiB`（真实字节） |
| 2 | 溢出判断修正 | `_read()` 按 `_rx_bytes + len(data) > RX_BACKLOG_LIMIT` 判断，不再按 chunk 数 |
| 3 | 报错语义修正 | 溢出报错输出 `current/peak/chunks/limit` 真实字节数 |
| 4 | 可观测性 | 新增 `rx_snapshot()`（current/peak 字节与 chunk 数）、`rx_empty()`、`put_received()` |
| 5 | 观测通道修复 | DEBUG 链路状态原仅由 NTRIP 线程周期发出，新增 GUI 定时器使「仅串口采集」场景也能每 10 s 输出 `Host RX=... peak=...` |

### 措施有效性验证

- 语法检查通过；`MonitorBridgeTest` 16 项单元测试全部通过（无回归）；
- 无头验证：注入 103 B 后 `Host RX=103 B, peak=103 B, chunks=2`，消费后归零 `Host RX=0 B, peak=103 B, chunks=0`，字节计数与排水逻辑正确；
- 1.5 h 短测验收通过（见第四节）。

---

## 六、举一反三（举一反三）

1. **单位语义审查**：排查同类「用数量/计数冒充物理量」的阈值与限制。在本次针对接收/转发队列与背压逻辑的审查范围内，未发现其他同类单位语义错误（`ntrip_rtcm.py` 的 `frames` 队列以字节计数）。
2. **可观测性前置**：凡涉及「达到阈值即停止」的保护逻辑，必须同时输出当前/峰值的真实物理量，避免事后无法复现定位。
3. **阈值设定原则**：背压上限必须采用与积压对象一致的计量单位（字节对字节、帧对帧），禁止跨单位换算假设。
4. **故障分级原则固化**：坚持「宁可明确停采、不静默丢数据」的保护策略，此策略在本故障中被验证为正确（触发瞬间丢帧 = 0）。
5. **长测验收标准固化**：确立 5 项验收指标（peak 水平、current 回零、四个 d_*、IMU 连续性、运行连续性），并明确 `peak` 为单调不降历史最大值、须判趋势而非单点。

---

## 七、归零结论

- **定位准确**：故障定位于 `host/serial_worker.py` 接收队列阈值单位语义错误（chunk 数冒充字节数）。
- **机理清楚**：主线程偶发停顿 → chunk 数短时堆叠至 512 → 误触发「2 MiB」保护停采。旧事故未记录触发瞬间真实字节 backlog；修复后同等采集条件短测的真实 peak 为 21.6 KiB，因此强烈支持旧版 512-chunk 阈值过敏这一直接触发机理。
- **问题复现**：以字节精确观测替代验证，1.5 h 短测测得真实 peak 21.6 KiB（2 MiB 的 1.05%），强烈支持「阈值过敏」归因。
- **措施有效**：字节精确背压 + 可观测性已实现并验证，短测通过，单元测试无回归。
- **举一反三**：完成同类缺陷排查、可观测性前置、阈值单位原则与验收标准固化。

**归零状态**：最终关闭。21 h 长测验收通过——`peak` 稳定 39.6 KiB（2 MiB 的 1.9%）不爬升、`current` 长期回零（最大 9.2 KiB）、四个 `d_*` 全 0、IMU 7,555,560 样本连续无丢样、无「GUI RX backlog exceeded」且 0 ERROR/WARN。

---

## 附录：关键代码对比

### 修复前

```python
self.received = queue.Queue(maxsize=512)  # each chunk <=4096 bytes: 2 MiB

def _read(self):
    ...
    try: self.received.put_nowait(data)
    except queue.Full as error:
        raise OSError("界面/记录处理停顿，接收队列已达 2 MiB；停止采集以免静默丢数据") from error
```

### 修复后

```python
RX_BACKLOG_LIMIT = 2 * 1024 * 1024  # 2 MiB, byte-accurate

# __init__ 内：
self._rx_lock = threading.Lock()
self._rx_chunks = deque()
self._rx_bytes = 0
self._rx_peak_bytes = 0
self._rx_peak_chunks = 0

def _read(self):
    ...
    with self._rx_lock:
        if self._rx_bytes + len(data) > self.RX_BACKLOG_LIMIT:
            raise OSError(
                f"GUI RX backlog exceeded: current={self._rx_bytes} B "
                f"peak={self._rx_peak_bytes} B chunks={len(self._rx_chunks)} "
                f"limit={self.RX_BACKLOG_LIMIT} B；停止采集以免静默丢数据")
        self._rx_chunks.append(data)
        self._rx_bytes += len(data)
        ...
```

---

## 八、审批记录

**审批结论：通过，归零关闭。**

故障触发机制定位明确，纠正措施针对根因，短时回归、单元测试与实机验证均通过。21 h 长测（会话 `20260921140624`）最终验收：`Host RX peak` 稳定 39.6 KiB 不爬升、`current` 长期回零、四个 `d_*` 全 0、IMU 连续无丢样、无停采告警。当前证据充分支持「旧版 512-chunk 背压阈值过敏是此次保护性停采的直接触发原因」，本故障正式关闭。
