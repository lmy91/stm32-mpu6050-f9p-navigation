# RTCM/RTK 修复验证（2026-09-09）

## 修复内容

- 将 NTRIP 的 16 帧瞬时队列改为 64 KiB 字节限额和有界等待，保留两秒过期限制。
- COM7 由独立串口线程独占，批量发送并即时读取 STM32 确认；1024 字节在途限制不变。
- 针对 WUH2 流最后由 NavIC 1137 结束 MSM 组、F9P HPG 1.13 不使用该消息的组合，添加 MSM 组兼容层。移除 NavIC，将结束位放在最后保留的 MSM 上并更新 CRC，其他观测位不变。
- 不修改原 v3 / IMU、GNSS、RAWX CSV 格式，不新增采集文件或 raw 文件夹，不需要更新接收机/STM32 固件。

## 实机证据

F9P 查询：HPG 1.13、PROTVER 27.12，TMODE=0，DGNSSMODE=3，UART1 RTCM 输入已开启。

修复队列但未应用 MSM 兼容时，一次 60 秒采集保持零重连/零丢帧，接收机仍为 `carr_soln=0, diffSoln=0`。输入中 GPS/GLO/GAL/SBAS/QZSS/BDS MSM 的 multiple-message 位为 1，NavIC 1137 为 0；接收机反馈 1137 未使用。

应用 MSM 兼容后，独立 USB 查询确认 `carr_soln=2, diffSoln=1`。最终一次 45 秒回归（每十秒故意让 GUI 停顿 0.5 秒）结果：

| 指标 | 结果 |
| --- | --- |
| IMU / GNSS / RAWX 记录数 | 4496 / 45 / 1542 |
| IMU 序号丢帧 / 无效行 | 0 / 0 |
| 网络重连 / 网络 CRC 错误 | 0 / 0 |
| STM32 接收 / 转发增量 | 89575 / 89575 字节 |
| STM32 下行丢字节 / 串口错误增量 | 0 / 0 |
| F9P RTCM CRC 错误增量 | 0 |
| 网络字节队列峰值 | 2996 字节 |
| 最长发送前排队时间（含 MSM 组等待） | 约 1.313 秒 |
| 最终定位状态 | RTK 固定，carr_soln=2，diffSoln=1 |
| 最终接收机水平精度估计 hAcc | 16 mm（不是独立测量的实际误差） |

测试正常退出，检查脚本退出码为 0；共 40 项自动测试通过（host 35、tools 5），包含 100 小帧突发、无 GUI 事件处理时继续转发、部分发送、异常停止、MSM 修改位/CRC 检查和三 CSV 保存。

上述结果证明此次测试条件下问题得到修复，不等于全天候或硬实时保证。网络延迟、天线遮挡、周跳仍可能造成浮点/失锁；GUI/磁盘长时间停顿超过 2 MiB 接收缓存会明确停止并报警。尚需长时间户外/运动场景验收。

## 重复测试

关闭 Qt 并停止其他差分注入后，在仓库根目录执行：

```powershell
python -m unittest discover -s host -p 'test*.py'
python -m unittest discover -s tools -p 'test*.py'
python tools/check_rtcm_bridge.py COM7 --seconds 45 --stall-gui --require-fixed --bnc <私有配置路径>
```

`.bnc` 文件含私人凭据，不提交到仓库。诊断工具只打印统计，不写采集文件。F9P 原生 USB 查询与通过 STM32 发送只读查询的使用条件见 tools/README.md。
