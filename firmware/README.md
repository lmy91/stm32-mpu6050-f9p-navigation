# MPU6050/F9P 组合导航下位机固件

[项目主页](../README.md) | 中文 | [English](README_EN.md)

本目录包含 STM32F103C8T6 的 MPU6050/ZED-F9P 时间同步固件。MPU6050 通过 I2C1 接入，PA1/TIM2_CH2 硬件捕获 DATA_RDY；PA0/TIM2_CH1 捕获 F9P TIMEPULSE。STM32 解析 USART2 上的 UBX，并通过 USART1 以 460800 bit/s 输出带 GPS 时间戳的 IMU、GNSS 和卫星天空图数据。

## 当前配置

- 目标芯片：STM32F103C8T6
- I2C：PB6/SCL、PB7/SDA
- 系统时钟：板载 8 MHz HSE，经 PLL 倍频至 72 MHz
- GNSS PPS：PA0/TIM2_CH1，上升沿硬件捕获
- 数据就绪：PA1/TIM2_CH2，上升沿硬件捕获
- GNSS 串口：PA2/USART2_TX、PA3/USART2_RX，115200 bit/s
- 串口输出：PA9/USART1_TX，460800 bit/s
- 标称输出频率：约 100 Hz
- 程序启动方式：上电或复位后自动采集

## 接线

| 外设 | STM32 引脚 |
| --- | --- |
| GY-521 VCC | 3.3V |
| GY-521 GND | GND |
| GY-521 SCL | PB6 |
| GY-521 SDA | PB7 |
| GY-521 INT | PA1/TIM2_CH2 |
| C099 TP | PA0/TIM2_CH1 |
| C099 RX_ZED | PA2/USART2_TX |
| C099 TX_ZED | PA3/USART2_RX |
| C099 GND | GND |
| USB-TTL RX | PA9 |
| USB-TTL GND | GND |
| ST-LINK SWDIO | PA13/SWDIO |
| ST-LINK SWCLK | PA14/SWCLK |
| ST-LINK GND | GND |
| ST-LINK 3.3V | 3.3V |

保持 BOOT0=0。所有设备必须共地。USB-TTL 的 TX 不需要连接；不要让多个电源同时向开发板 VCC 反向供电。

C099 的 J4 必须只在 `ARDUINO MODE`（7-8，板上丝印 `ARD`）放置跳帽，才能让 STM32 的 PA2 驱动 ZED-F9P RXD。`ARD`、`UART1`、`UART3` 三个位置只能选择一个。

STM32 每次启动会把 F9P UART1 配置到 115200 bit/s，允许 UBX/RTCM3 输入，只输出 UBX；内部测量与导航保持 10 Hz，`UBX-NAV-PVT`、`UBX-NAV-SAT` 和 `UBX-TIM-TP` 均输出 1 Hz。TIMEPULSE 为 GPS 时间网格、1 Hz、100 ms 高电平、上升沿对齐周内整秒。配置首先写入 RAM；本次部署也已通过 C099 COM4 写入 BBR/Flash。

## 编译

需要 CMake、Ninja 和 GNU Arm Embedded Toolchain。STM32Cube VS Code 扩展的工具可按下面方式加入当前 PowerShell：

    $ninjaDir = "$env:LOCALAPPDATA\stm32cube\bundles\ninja\1.13.2+st.1\bin"
    $gccDir = "$env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin"
    $env:Path = "$ninjaDir;$gccDir;$env:Path"

在仓库根目录构建 Release：

    Push-Location firmware
    cmake --preset Release
    cmake --build --preset Release
    Pop-Location

调试版本将两个 Release 替换为 Debug。主要输出：

    firmware\build\Release\mpu6050_f9p_navigation.elf

build/ 是可重建目录，不提交到 Git。

## 烧录

可以在 STM32CubeProgrammer 中选择 ELF 文件，也可以在仓库根目录运行：

    & "$env:LOCALAPPDATA\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR reset=HWrst -w "firmware\build\Release\mpu6050_f9p_navigation.elf" -v -rst

烧录后复位。PA9 会输出四种带记录类型的 CSV 数据：

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs

`IMU` 以 100 Hz 输出。每个样本的 `gps_tow_us` 由同一 TIM2 时钟域内硬件捕获的 MPU6050 DATA_RDY 和 F9P 1PPS 直接换算，单位为 GPS 周内微秒；`time_valid=1` 才表示 GPS 时间有效。首次收到有效 TIM-TP/PPS 之前，GPS 周和 TOW 输出 0。

`GNSS` 以 1 Hz 输出。`rx_timer_us` 是完整NAV-PVT帧通过校验时的STM32本地微秒时刻。经纬度单位为 `1e-7 deg`，高程与位置精度为mm，NED/地面速度与速度精度为mm/s，PDOP比例为0.01。`flags`、`flags2`保留NAV-PVT原始质量位，`carr_soln` 从 `flags[7:6]` 提取（0=无载波解，1=RTK浮点，2=RTK固定）。`SAT`/`SAT_END` 提供1 Hz天空图快照。每个PPS还会输出一行以 `# sync` 开头的诊断状态。

## 工作原理

TIM2 以 1 MHz 自由运行并用溢出计数扩展到 48 位。CH1 和 CH2 分别在硬件中锁存 PPS 与 DATA_RDY，因此二者处于同一个微秒时间域。中断只搬运捕获值，主循环再执行 I2C、UBX解析和串口输出。

## 常见问题

- 没有 IMU 数据：检查 INT→PA1、PA9→USB-TTL RX、共地、460800 波特率。
- STM32 能接收 GNSS 但不能配置：把 C099 J4 跳帽移到 `ARD`，并移除 `UART1/UART3` 跳帽。
- 编译找不到 Ninja/GCC：确认上面的版本目录与本机一致。
- 烧录失败：检查 BOOT0=0、ST-LINK 接线和驱动，降低 SWD 频率后重试。
- 采样率异常：确认 MPU6050 INT 接到 PA1/TIM2_CH2，并检查主循环是否被阻塞。

数据查看和分析请继续阅读 [Qt 上位机说明](../host/README.md) 与 [工具说明](../tools/README.md)。
