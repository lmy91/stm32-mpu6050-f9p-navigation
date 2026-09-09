# 已知问题与后续加固 / Known Issues and Hardening

本文记录当前版本已经确认的外部限制和代码加固项。它们不表示现有采集文件已经损坏；2026-09-09 的两次完整实测中，IMU、1 Hz GNSS 和 RAWX 历元均保持完整。

## WUH2 NTRIP 稳定性

一次约 22 分钟的直连记录出现 15 次超过 5 秒无有效 RTCM、3 次超过 15 秒并触发重连，以及 9 次重连 HTTP 404。同期 RTCM CRC、Qt 待发队列、STM32 丢字节/串口错误均为零，STM32 接收与转发字节一致，F9P 也持续回报收到/使用差分帧。

因此，已观测中断位于 WUH2 服务端或到服务端的网络路径，而不是 Qt → STM32 → F9P 本地转发链路。HTTP 404 是 caster 或其上游代理实际返回的响应，是服务端/负载均衡异常的强证据；仅凭客户端日志仍不能把所有短暂停顿唯一归因于服务器本体。

当前客户端会在 5 秒无有效 RTCM 时告警，在 15 秒无有效帧时重连。RTK 浮点/固定可能因差分中断、卫星几何、多路径或载波模糊度而变化，不能仅凭状态切换判断本地链路故障。

## 待加固项

### P1：PPS 新鲜度保护

固件的 `time_valid` 只继承最近一次有效 TIM-TP/PPS 关联，尚未检查该 PPS 距当前样本的时间。若 PA0/TIMEPULSE 在成功同步后断线，固件可能继续从旧 PPS 外推 GPS 时间并保持 `time_valid=1`。

计划：超过约 1.5–2.5 秒未捕获新 PPS 时把时间标为无效，并向状态行输出 PPS 年龄和计数。组合导航只能使用新鲜且有效的时间戳。

### P1：IMU 硬件中断覆盖可观测性

固件已经统计 `g_debug_interrupt_overruns`，但当前状态协议没有输出该计数。Qt 的“丢帧”只比较成功输出的样本序号；极端负载下如果新的 DATA_RDY 覆盖了尚未处理的捕获值，连续样本号本身不能揭示这类物理缺样。

计划：扩展状态行并在 Qt/日志中显示中断覆盖、I2C 错误及异常采样间隔。保持数据协议 v3 的 IMU 行不变。

### P2：RAWX 历元完整性校验

Qt 当前收到 `RAWX_END` 后结束历元，但没有比较头部 `num_meas`、实际收到的 `RAWX_MEAS` 数和结尾计数。发生截断时可能保存一个不完整历元而没有单独告警。

计划：逐历元计数、校验三个数量，丢弃或明确标记不完整历元，并将计数加入日志。当前完整实测中未发现不完整历元。

### P2：观测消息独立存活监测

NTRIP 的 5/15 秒连接判断目前由任意 CRC 正确的 RTCM 帧刷新。若 caster 仍发送站点信息等非观测消息、但 MSM 观测已经停止，连接可能保持“正常”。

计划：保留连接级存活监测，同时独立监测 MSM 观测到达年龄并给出不同告警；不要把消息到达间隔误称为 GNSS 差分龄期。

### P2：STM32 RAM 余量

当前 Release 链接约占 17.2 KiB/20 KiB RAM（包含链接脚本保留的最小堆栈），余量约 3.3 KiB。现有采集固件可运行，但不适合继续加入大型导航状态或缓存。

计划：STM32 保持实时采集、硬件时间戳和 RTCM 转发；松组合与紧组合解算放在 PC 端 Qt/fusion 模块。若继续扩展固件，必须重新检查 map 文件和最坏栈深度。

## 已验证基线

- 会话 `20260909154341`：171,248 个 IMU 样本，间隔 10.004–10.005 ms；1,713 个 GNSS 历元；1,713 个完整 RAWX 历元。
- 会话 `20260909162103`：199,550 个 IMU 样本，间隔 10.004–10.005 ms；1,996 个 GNSS 历元；1,996 个完整 RAWX 历元。
- 两次记录的 IMU 样本号均无跳号，GPS 时间均有效，RAWX 实际观测行数均与历元声明一致。
- 当前自动测试：host 58 项、tools 5 项通过；STM32 Release 编译通过。

---

## English summary

Observed WUH2 outages occur upstream of the local Qt → STM32 → F9P bridge: the local queue, CRC, UART loss and MCU forwarding counters remain healthy, while the caster sometimes returns HTTP 404 during reconnects. Short gaps can still involve the network path, so client logs alone cannot attribute every pause exclusively to the caster host.

Pending hardening items are: invalidate stale PPS-derived timestamps, expose physical IMU interrupt overruns, validate every RAWX epoch count, monitor MSM observation liveness independently from generic RTCM traffic, and preserve STM32 RAM headroom by keeping loose/tight fusion on the PC.
