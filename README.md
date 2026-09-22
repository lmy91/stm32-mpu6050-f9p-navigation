# STM32 MPU6050/F9P 组合导航实验系统

[中文](README.md) | [English](README_EN.md)

这是一个面向低成本 GNSS/INS 的实时实验平台：STM32F103 在同一个硬件定时器时钟域内捕获 MPU6050 DATA_RDY 与 ZED-F9P 1PPS，把每个 IMU 样本标记为 GPS 周/周内微秒，同时以 1 Hz 输出 GNSS 导航解、天空图及 RAWX 原始观测。Qt 上位机完成实时显示、地图轨迹、分文件记录，并提供 PC 端实时自瞄/松组合原型。

当前版本已加入使用同步 IMU 与 F9P 位置/速度的局部 NED 松组合自瞄原型；紧组合伪距/多普勒滤波尚未实现。该算法用于联调研究，正式使用前仍需完成数据质量、安装角、杆臂、延迟和参考轨迹验证。

## 当前能力

- MPU6050：100 Hz，加速度、角速度、温度
- F9P：内部导航10 Hz；NAV-PVT/NAV-SAT/RXM-RAWX/TIM-TP以1 Hz输出，RXM-SFRBX逐条输出
- PA0/TIM2_CH1 捕获 GNSS PPS，PA1/TIM2_CH2 捕获 IMU DATA_RDY
- 每条 IMU 数据直接携带 `gps_week`、`gps_tow_us` 和 `time_valid`
- GNSS 数据包含 WGS-84 坐标、海拔、NED/地面速度、定位类型、卫星数、PDOP
- Qt 显示 IMU 曲线、速度曲线、本地轨迹/高德地图和多星座天空图
- 原始观测包含伪距、载波相位、多普勒、锁定时间、C/N0、质量位及信号/频点标识
- Qt 可独立选择保存 IMU/GNSS导航/RAWX原始观测 CSV，并可选保存 PC 对准结果 `aim.csv` 和实时松组合结果 `nav.csv`；选择AIM/NAV时同时生成不含密钥的 `session.json`，记录本次实际生效参数
- Allan方差工具直接读取采集器生成的当前21列IMU v3文件
- Qt 直连 NTRIP v2，经同一个 COM7 将 RTCM3 下发到 STM32，再由 PA2 转发给 F9P；显示各级接收/转发计数及 RTK 浮点/固定状态

## Raspberry Pi 生产部署

Raspberry Pi 5 的正式采集与部署请使用独立仓库：

https://github.com/lmy91/pi5-mpu6050-f9p-logger

该仓库是本项目的 Raspberry Pi 部署子项目，负责 Web Dashboard、Wi-Fi 热点、
systemd、NTRIP、原始 UBX 保存、GNSS 校时和设备端采集运维。

本仓库中的 `raspberry_pi5/` 当前仅作为开发与历史参考保留，不再作为正式生产部署入口。

STM32 固件和串口协议仍以本仓库为上游；稳定版本冻结后再同步到 Pi 部署仓库。

## 树莓派服务切换：ICM ↔ MPU

> 本节仅用于开发/测试时在同一块树莓派上切换两套实验服务；正式生产部署请使用上文的
> [`pi5-mpu6050-f9p-logger`](https://github.com/lmy91/pi5-mpu6050-f9p-logger)。

同一块树莓派上，ICM 项目（`pi5-icm42688p-f9p-logger`）与本项目（MPU）使用
**完全相同的 systemd 服务名**（`gnss-imu-logger.service`、
`gnss-imu-dashboard.service`），并且都独占 `/dev/ttyAMA0`（460800）和
`/dev/ttyAMA2`（115200）。因此两套服务不能同时运行，切换时必须先停掉一套
再启用另一套，否则会发生串口争用。

### 从 ICM 切换到 MPU（关闭 ICM，打开 MPU）

在树莓派上执行：

```bash
# 1. 若 ICM 正在采集，先安全停止保存
gnss-imu-record-stop

# 2. 停止并禁止 ICM 的开机自启（ICM 还有第三个 time-sync 服务）
sudo systemctl stop gnss-imu-logger.service gnss-imu-dashboard.service gnss-imu-time-sync.service
sudo systemctl disable gnss-imu-logger.service gnss-imu-dashboard.service gnss-imu-time-sync.service

# 3. 安装并启用 MPU 的服务（把服务文件复制到位并 enable --now）
cd /home/lmy/stm32-mpu6050-f9p-navigation
sudo cp raspberry_pi5/systemd/gnss-imu-logger.service \
  /etc/systemd/system/gnss-imu-logger.service
sudo cp raspberry_pi5/systemd/gnss-imu-dashboard.service \
  /etc/systemd/system/gnss-imu-dashboard.service
sudo systemctl daemon-reload
sudo systemctl enable --now gnss-imu-logger.service
sudo systemctl enable --now gnss-imu-dashboard.service
```

校验 MPU 服务已正常运行：

```bash
systemctl status gnss-imu-logger.service gnss-imu-dashboard.service
journalctl -u gnss-imu-logger.service -f
```

### 从 MPU 切回 ICM（关闭 MPU，打开 ICM）

```bash
# 1. 若 MPU 正在采集，先停止保存
gnss-imu-record-stop

# 2. 停止并禁止 MPU 的服务
sudo systemctl stop gnss-imu-logger.service gnss-imu-dashboard.service
sudo systemctl disable gnss-imu-logger.service gnss-imu-dashboard.service

# 3. 回到 ICM 项目重新生成并启用服务
cd /home/lmy/pi5-icm42688p-f9p-logger
./scripts/install_services.sh
```

### 切换注意事项

- 两个项目的 `gnss-imu-record-start/stop`、`gnss-imu-status`、`gnss-imu-base`
  等命令名也相同，都是软链到各自项目的脚本。切换服务后要确认这些软链指向
  当前生效的项目，否则采集/基站控制会落到错误的脚本上。
- 服务文件里的用户和路径是硬编码的：MPU 为 `lmy` 和
  `/home/lmy/stm32-mpu6050-f9p-navigation`，ICM 由 `install_services.sh` 按当前
  用户和项目路径生成。若用户名或路径不同，需先修改服务文件再复制。
- 正常切换顺序永远是「先 `record-stop`，再 `stop`，再 `disable`，最后
  `enable --now` 新服务」。不要直接拔电源，避免损坏正在写入的 CSV/UBX 文件和
  SD 卡文件系统。

## 硬件与接线

树莓派5替代USB-TTL并为STM32供电的已验证方案见
[`raspberry_pi5/README.md`](raspberry_pi5/README.md)。

树莓派开机后可在PC浏览器打开`http://192.168.137.2:8080`实时查看GNSS位置、
轨迹、速度、RTK状态、卫星数和PDOP。定位服务持续运行但默认不保存；输入命令
或点击网页“开始采集”后才创建三个CSV和F9P原始`f9p.ubx`，停止保存不影响实时位置。网页服务不
会再次打开串口；手机接入方法和高德地图设置见
[`raspberry_pi5/LIVE_DASHBOARD.md`](raspberry_pi5/LIVE_DASHBOARD.md)。
基站也可在树莓派终端通过`gnss-imu-base connect/status/reconnect/disconnect`
控制；密码为隐藏输入，网页与终端显示的是同一个NTRIP会话。
网页底部提供受限服务控制台和结果窗口，可查询状态、日志、网络与磁盘，控制
保存和NTRIP，并安全重启采集或网页服务；不开放任意Linux Shell。

停止采集后，可在Windows PowerShell中用RTKLIB的`convbin.exe`把`f9p.ubx`
转换成完整的RINEX 3.04观测文件和混合导航文件：

```powershell
& "C:\Users\12597\Desktop\convbin.exe" -r ubx -v 3.04 -f 5 -od -os -oi -ot -ol -o ".\rover.obs" -n ".\rover.nav" ".\f9p.ubx"
```

应先停止采集并进入包含`f9p.ubx`的时间戳采集目录。参数解释、结果检查和
树莓派等价命令见
[`raspberry_pi5/LOGGER_SERVICE.md`](raspberry_pi5/LOGGER_SERVICE.md#转换rinex)。

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
| ST-LINK | PA13 SWDIO、PA14 SWCLK、GND；供电线按当前供电模式选择，禁止与树莓派双路供电 |

所有设备必须共地。树莓派与STM32的UART为460800 bit/s，F9P与STM32之间为115200 bit/s。保持BOOT0=0，C099 J4仅选择`ARD`，不要同时短接`UART1`/`UART3`。当前DCDC方案用VADJ经两根5V和两根GND专供Pi，固定5V分别独立供STM32和C099；ST-Link只接SWDIO、SWCLK、GND和可选RST，不接3.3V/5V。完整接线见[硬件接线详解](docs/硬件接线详解.md)。

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

Qt 做 HTTP chunk 解包和 RTCM CRC24Q 校验，仅下发完整有效帧；STM32 中断转发，Qt 根据回传计数限制未确认数据为 1024 字节。缓冲有界；单帧排队超过两秒只告警并继续追赶，只有 STM32 状态/确认无进展或串口写入失败才停止下发，采集继续。`F9P 收/使用` 来自 UBX-RXM-RTCM，不能把网络字节数当成接收机已用差分。`距接收` 是距最后 RTCM 状态的间隔，不是观测历元差分龄期。原 GNSS v3 和三类 CSV 不变；`gnss.csv` 的 `carr_soln=1/2` 分别表示 RTK 浮点/固定，`fix=3` 本身不代表是否 RTK。

## 命令行采集与分析

```powershell
# 分别保存 IMU/GNSS，0 小时表示持续到 Ctrl+C
D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 0

# 当前21列IMU v3可直接用于Allan分析
D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\20260908180500\imu.csv --rate 100 --skip-minutes 30
```

详细说明见 [硬件接线详解](docs/硬件接线详解.md)、[时间同步方案总结](docs/时间同步方案总结.md)、[固件](firmware/README.md)、[Qt 上位机](host/README.md)、[工具](tools/README.md)、[组合导航算法规划](fusion/README.md)、[已知问题与后续加固](docs/KNOWN_ISSUES.md)、[Allan 方差说明](docs/Allan方差知识总结.md) 和 [IMU 丢数可观测性与 TIM2 加固总结](docs/IMU丢数可观测性与TIM2加固总结.md)。

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
3. 校准、回放并完善当前 F9P 位置/速度 + MPU6050 松组合误差状态滤波。
4. 在已接入 RTCM/NTRIP 与 RTK 状态基础上，实现原始观测量紧组合。

## 许可与来源

MIT License。项目演化自 [lmy91/stm32-mpu6050-allan-toolkit](https://github.com/lmy91/stm32-mpu6050-allan-toolkit)，保留原作者版权与许可声明。
