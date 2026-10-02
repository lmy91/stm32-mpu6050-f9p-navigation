# WGS84 大地高输出（2026-10-02）

当前 STM32 与 Pi 采集项目只输出 WGS84 纬度、经度、大地高。WGS84 大地高与
WGS84 椭球高是同一个量；本次去掉的是原有 hMSL 海拔字段。

| 来源 | 纬度 | 经度 | 大地高 |
|---|---|---|---|
| NAV-PVT 字节偏移 | 28，I4，1e-7 度 | 24，I4，1e-7 度 | 32，I4，mm |
| STM32 ASCII | lat_e7 | lon_e7 | height_mm |
| Pi/PC gnss.csv | lat_deg，度 | lon_deg，度 | height_m，米 |

STM32 在 RAM 设置 CFG-NAVSPG-USE_USRDAT（0x10110061）为 false，使用 WGS84。
高度直接来自同一帧 NAV-PVT 的 height，不从 hMSL 估算。GNSS 仍为 1 Hz，IMU
仍为 100 Hz。本次没有修改组合导航算法，也没有进行组合导航验证。

## 协议 v4

新记录使用 GNSS4 前缀，并把原高度位置改为 height_mm。字段总数不变：

```text
GNSS4,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,height_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
```

CSV 原 hmsl_m 位置改成 height_m，不再保存 hmsl_m。网页标签为“WGS84大地高”。
旧 GNSS 前缀记录仍可解析，但新 CSV 的 height_m 留空，防止把历史海拔当作大地高。
不要将旧文件的 hmsl_m 直接重命名成 height_m。旧采集器不认识 GNSS4，需要更新。
原始 f9p.ubx 旁路仍保存接收机原始字节，不改变原始 UBX 消息的结构。

## 部署和使用

1. Pi 更新 tools/acquisition/capture_serial.py、raspberry_pi5/live_dashboard.py 和
   raspberry_pi5/live_dashboard/index.html。先停止保存，再重启
   gnss-imu-logger.service 与 gnss-imu-dashboard.service。
2. STM32 烧录 release/mpu6050_f9p_navigation.hex 或 .elf，复位后日志为
   protocol_v4，导航记录前缀为 GNSS4。
3. 开始新会话，确认 gnss.csv 的 height_m 在有效 3D/RTK 解时有数值。
4. PC 使用更新后的 host/imu_serial_qt.py。旧打包 EXE 需重新打包。
5. 组合导航的数据读取改为 gnss0.height_m；经纬度仍为度，算法需要弧度时乘 pi/180。

仅修改本地 Pi 项目不会更新远端 Pi。对有效观测继续使用已有 GPS周/TOW对时、定位
有效性和精度门限。此次使用 NAV-PVT，未切换至 HPPOSLLH；输出分辨率不等于精度。

历史 CSV 不能仅靠 hMSL 精确恢复大地高。若同会话原始 UBX 包含 NAV-PVT，可按历元
读取偏移32的 height。KML 的 absolute 高度需海平面类高程，不能直接使用 height_m。

参考：[u-blox F9P 接口手册](https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf)。
## 本次本地验证记录

- STM32 和 Pi 固件均完成 Release 编译。
- 主项目采集器10项测试、Pi采集/网页/对时/NTRIP共46项测试通过。
- 2026-10-02 已通过 ST-Link 烧录已接入的 STM32F103（64KB），写入校验成功。
- SWD 读取确认程序运行，并接收 NAV-PVT；检查时 fix_type=0，无有效定位解。
- COM7以460800读取15秒未收到字节，未确认实际串口高度数值。
- 未进行组合导航验证，未更新远端树莓派。
- 烧录前64KB Flash备份保存于工作区 tmp/stm32_before_wgs84_20261002.bin。
