# STM32 MPU6050/F9P 组合导航实验系统

[中文](README.md) | [English](README_EN.md)

这是一个面向低成本 GNSS/INS 的实时实验平台：STM32F103 在同一个硬件定时器时钟域内捕获 MPU6050 DATA_RDY 与 ZED-F9P 1PPS，把每个 IMU 样本标记为 GPS 周/周内微秒，同时以 1 Hz 输出 GNSS 导航解、天空图及 RAWX 原始观测。Qt 上位机完成实时显示、地图轨迹和 IMU/GNSS/RAWX 分文件记录。

当前版本完成的是组合导航的同步采集与可视化基础层，尚未把松组合 EKF 或紧组合伪距/多普勒滤波写入导航解算输出。建议先用本项目完成数据质量、时间同步和杆臂标定验证，再在 `fusion/` 中加入后续算法。

## 当前能力

- MPU6050：100 Hz，加速度、角速度、温度
- F9P：内部导航 10 Hz，对 STM32 输出 NAV-PVT/NAV-SAT/RXM-RAWX/TIM-TP 各 1 Hz
- PA0/TIM2_CH1 捕获 GNSS PPS，PA1/TIM2_CH2 捕获 IMU DATA_RDY
- 每条 IMU 数据直接携带 `gps_week`、`gps_tow_us` 和 `time_valid`
- GNSS 数据包含 WGS-84 坐标、海拔、NED/地面速度、定位类型、卫星数、PDOP
- Qt 显示 IMU 曲线、速度曲线、本地轨迹/高德地图和多星座天空图
- 原始观测包含伪距、载波相位、多普勒、锁定时间、C/N0、质量位及信号/频点标识
- Qt 可独立选择保存 IMU/GNSS导航/RAWX原始观测 CSV，命令行工具也支持分文件记录
- Allan方差工具直接读取采集器生成的当前21列IMU v3文件
- Qt 直连 NTRIP v2，经同一个 COM7 将 RTCM3 下发到 STM32，再由 PA2 转发给 F9P；显示各级接收/转发计数及 RTK 浮点/固定状态

## 硬件与接线

| 设备 | STM32F103C8T6 |
| --- | --- |
| MPU6050 VCC/GND | 3.3V/GND |
| MPU6050 SCL/SDA | PB6/PB7 |
| MPU6050 INT | PA1/TIM2_CH2 |
| C099 TP | PA0/TIM2_CH1 |
| C099 TX_ZED | PA3/USART2_RX |
| C099 RX_ZED | PA2/USART2_TX |
| C099 GND | GND |
| USB-TTL RX/GND | PA9/GND |
| USB-TTL TX（3.3V TTL） | PA10/USART1_RX |
| ST-LINK | PA13 SWDIO、PA14 SWCLK、3.3V、GND |

所有设备必须共地。USB-TTL RX 接 PA9，TX 接 PA10，电脑口为 460800 bit/s；F9P 与 STM32 之间为 115200 bit/s。保持 BOOT0=0，C099 J4 仅选择 `ARD`，不要同时短接 `UART1`/`UART3`。纯采集可以不接 PA10，RTCM 下发必须接。

## 数据链路

```text
MPU6050 DATA_RDY ──PA1/TIM2_CH2──┐
                                 ├─ STM32 时间关联 ─PA9/460800─ Qt/CLI
F9P 1PPS ─────────PA0/TIM2_CH1───┤
F9P UBX ──────────PA3/USART2_RX──┘
WUH2 ──NTRIP── Qt ──COM7/PA10── STM32 ──PA2/USART2_TX──→ F9P
```

## 快速开始

安装电脑端依赖：

```powershell
D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r host\requirements.txt
D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt
```

编译与烧录固件：

```powershell
cmake --preset Release -S firmware
cmake --build firmware\build\Release --clean-first
& "$env:LOCALAPPDATA\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR reset=HWrst -w "firmware\build\Release\mpu6050_f9p_navigation.elf" -v -rst
```

启动 Qt：

```powershell
D:\anaconda\envs\allan-toolkit\python.exe host\imu_serial_qt.py
```

选择 PA9 USB-TTL 对应端口（当前设备为 CH340 COM7）和 460800。勾选保存后，程序创建：

- Qt 每次采集建立 `YYYYMMDDHHMMSS` 会话文件夹，按选择保存 `imu.csv`、`gnss.csv` 和 `rawx.csv`
- `imu.csv`：21列IMU v3，含GPS时间、本地捕获时间、原始值和物理量
- `gnss.csv`：GNSS 时间、WGS-84 位置、速度、PDOP、定位类型和卫星数
- `rawx.csv`：逐星逐频伪距、载波相位、多普勒、质量指标和实际信号频点

高德地图使用 Web JS API Key 和 `securityJsCode`。密钥只保存在本机 Qt 设置中，不应写入仓库；没有 Key 时本地米制轨迹仍正常工作。

### WUH2 实时 RTK

当前 Qt 包含 F9P MSM 历元兼容处理：WUH2 若把组结束标志放在 F9P 不支持的 NavIC MSM 上，会等待完整组到达，去掉 NavIC，给最后一条保留的 MSM 设置结束标志并重算 CRC。其他观测位不变，普通已正确结束的组不修改。该处理不影响 IMU/GNSS/RAWX CSV，也不需要升级 F9P 或重刷 STM32。详见 [上位机说明](host/README.md)。

先连接 COM7，等待 `RTCM 就绪`，在“基站设置…”填写 `ntrip.gnsswhu.cn:2101`、挂载点 `WUH200CHN0` 和账号密码，或导入你自己的 BNC 配置，再点“连接基站”。该链路不使用 BNC，避免其他程序同时注入差分数据。网络线程使用直接 TCP 连接，不继承系统 HTTP 代理；VPN 若使用 TUN/全局路由，仍需对基站域名/IP 设置直连。

Qt 做 HTTP chunk 解包和 RTCM CRC24Q 校验，仅下发完整有效帧；STM32 中断转发，Qt 根据回传计数限制未确认数据为 1024 字节。缓冲有界，积压/确认超时会停止下发并显示原因，采集继续。`F9P 收/使用` 来自 UBX-RXM-RTCM，不能把网络字节数当成接收机已用差分。`距接收` 是距最后 RTCM 状态的间隔，不是观测历元差分龄期。原 GNSS v3 和三类 CSV 不变；`gnss.csv` 的 `carr_soln=1/2` 分别表示 RTK 浮点/固定，`fix=3` 本身不代表是否 RTK。

## 命令行采集与分析

```powershell
# 分别保存 IMU/GNSS，0 小时表示持续到 Ctrl+C
D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 0

# 当前21列IMU v3可直接用于Allan分析
D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\20260908180500\imu.csv --rate 100 --skip-minutes 30
```

详细说明见 [固件](firmware/README.md)、[Qt 上位机](host/README.md)、[工具](tools/README.md)、[组合导航算法规划](fusion/README.md) 和 [Allan 方差说明](docs/Allan方差知识总结.md)。

## 串口协议 v3

```text
IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
RAWX_END,num_meas
```

只有 `time_valid=1` 时GPS时间有效。`rx_timer_us` 是完整UBX帧通过校验时的STM32本地接收时刻。RAWX 浮点量用 IEEE-754 位模式十六进制无损传输，上位机恢复为 SI 数值；每个 `RAWX_MEAS` 的 `gnss_id/sig_id/freq_id` 用于识别星座与频点。固件把一个RAWX历元分散到多个100 Hz IMU周期发送，以维持串口和采样实时性。F9P坐标按WGS-84保存；高德界面显示时才转换为GCJ-02。

## 后续组合导航路线

1. 完成长时间静态采集、Allan 噪声辨识和安装角/杆臂标定。
2. 加入惯导机械编排、静止检测和零速更新。
3. 实现 F9P 位置/速度 + MPU6050 的松组合误差状态 EKF。
4. 接入 RTCM/NTRIP 与 RTK 状态，再实现原始观测量紧组合。

## 许可与来源

MIT License。项目演化自 [lmy91/stm32-mpu6050-allan-toolkit](https://github.com/lmy91/stm32-mpu6050-allan-toolkit)，保留原作者版权与许可声明。
