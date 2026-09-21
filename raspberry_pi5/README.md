# 树莓派5 + STM32 + MPU6050/F9P 完整接线

本目录记录已经上电验证成功的树莓派5采集接线。树莓派替代原来的
USB-TTL和ST-Link供电，STM32继续负责IMU/GNSS硬件时间同步、协议v3输出
以及RTCM转发。

记录日期：2026-09-12

## 系统数据流

```text
WUH2 NTRIP
    ↓ 树莓派网络
树莓派5
    ├─ UART0，460800 bit/s，3.3 V TTL ↔ STM32
    ├─ GPIO5（只输入）← C099 TX_ZED 原始UBX旁路
    └─ 网页、NTRIP、CSV/UBX记录
外部降压模块
    ├─ VADJ 5.10～5.20 V → 树莓派5（两根5V、两根GND）
    └─ 固定5 V → STM32、C099-F9P独立分路
STM32F103C8T6
    ├─ I2C1 + DATA_RDY ─ MPU6050
    └─ USART2 + TIMEPULSE ─ ZED-F9P / C099-F9P
```

电脑通过网线连接树莓派。树莓派负责串口采集、RTCM下发、数据记录和后续
组合导航；电脑只承担远程操作或Qt显示。若树莓派需要连接WUH2，它还必须
通过Wi-Fi或电脑网络共享获得互联网访问。

电脑直连网络、Windows网络共享、SSH密钥登录和IPv6应急登录的实机配置见
[REMOTE_CONNECTION.md](REMOTE_CONNECTION.md)。

树莓派后台采集服务、数据目录和启停命令见
[LOGGER_SERVICE.md](LOGGER_SERVICE.md)。

手机和PC通过浏览器实时查看位置时，使用只读网页看板：

```text
http://192.168.137.2:8080
```

网页服务读取采集器发布的实时状态，不会打开或争用`/dev/ttyAMA0`或`/dev/ttyAMA2`。网页还以1 Hz显示三轴加速度、三轴角速度、温度及最近120秒IMU时间曲线，不增加STM32串口输出。未开始保存
时地图只保留一个当前位置点；开始保存后才累计当前会话的离散点。网页还可将
易失的NTRIP配置交给串口采集进程，由后者统一向STM32下发RTCM。
无需浏览器时，可用`gnss-imu-base connect/status/reconnect/disconnect`在终端
连接、检查或断开同一个基站会话。
网页底部另有受限服务控制台，可远程查询运行状态、日志、网络和磁盘，以及
控制数据保存和NTRIP，不开放任意Linux Shell。
地图下方的卫星天空图直接使用STM32输出的完整`SAT...SAT_END`历元，显示星座、
PRN、方位角、仰角、信号强弱和是否参与导航，不依赖CSV保存状态。
详细安装、网络和地图设置见[LIVE_DASHBOARD.md](LIVE_DASHBOARD.md)。

## 1. 电源与树莓派5

当前使用同一个降压模块作星形供电，树莓派不再作为STM32和C099的电源中转：

| 降压模块 | 目标 | 功能 |
|---|---|---|
| VADJ | 树莓派 Pin 2、Pin 4 / 5V | 两根正极线并联供电 |
| GND | 树莓派 Pin 6、Pin 9 / GND | 两根地线并联回流 |
| 固定5V | STM32 `5V` | 独立支路，MPU6050再由STM32 3.3V供电 |
| 固定5V | C099 `DC_IN 5-12V` | 独立支路给F9P供电 |
| GND | STM32 GND、C099 GND | 全系统公共参考地 |

VADJ必须先与树莓派断开，用万用表调到5.10～5.20V，确认始终不超过5.25V后
才能连接。树莓派主电源不得经过面包板，建议使用短的18～20 AWG导线和可靠
压接端子。当前实测模块端5.16V、树莓派带载端约4.91V，说明线路仍有约0.25V
压降；这个状态只能临时调试，不作为正式采集验收状态。不得继续升高VADJ补偿，
应降低导线和接点电阻，使树莓派带载端尽量保持5.0～5.15V，并用
`vcgencmd get_throttled`确认结果为`0x0`。

## 2. 树莓派5与STM32信号

| 树莓派5物理引脚 | 方向 | STM32F103C8T6 | 功能 |
|---|---:|---|---|
| Pin 8 / GPIO14 / TXD0 | → | PA10 / USART1_RX | RTCM、控制数据下发 |
| Pin 10 / GPIO15 / RXD0 | ← | PA9 / USART1_TX | IMU、GNSS、SAT、RAWX及状态上传 |
| Pin 29 / GPIO5 | ← | C099 `TX_ZED` | 旁路监听F9P原始UBX，必须保持为输入 |

串口参数：

```text
460800 bit/s，8 data bits，no parity，1 stop bit，no flow control
```

注意：

- UART必须交叉连接，即TX接RX、RX接TX。
- 树莓派GPIO只能连接3.3 V TTL信号，严禁向GPIO14或GPIO15输入5 V。
- 树莓派Pin 2和Pin 4是同一条5V电源轨，本方案用它们并联输入，不从这里向
  STM32或C099分配电源。
- C099必须接标注为`DC_IN 5-12V`的输入，不能接`5V OUT`或`3.3V OUT`。
- GPIO5与PA3同时监听同一个3.3V `TX_ZED`信号，不得将GPIO5设为输出。当前
  实际接线为直接分支，没有串联电阻；若后续重做线束，可串330Ω～1kΩ保护。
- 正常运行时断开ST-Link的3.3V/5V供电，禁止多个电源并联给STM32供电。
- 当前由降压模块固定5V给STM32供电时，ST-Link只连接`SWDIO→SWIO`、`SWCLK→SWCLK`、
  `GND→GND`，有复位线时可再接`RST→NRST`；不连接ST-Link的3.3V或5V。
- 460800波特率下杜邦线尽量短于20 cm，信号线应靠近公共地线。

## 3. MPU6050与STM32

| MPU6050 / GY-521 | STM32F103C8T6 | 功能 |
|---|---|---|
| VCC | 3.3V | 传感器供电 |
| GND | GND | 公共地 |
| SCL | PB6 / I2C1_SCL | I2C时钟 |
| SDA | PB7 / I2C1_SDA | I2C数据 |
| INT | PA1 / TIM2_CH2 | 100 Hz DATA_RDY硬件时间捕获 |

MPU6050仍由STM32的3.3V稳压输出供电，不直接连接树莓派3.3V。

## 4. C099-F9P与STM32

| C099-F9P | 方向 | STM32F103C8T6 | 功能 |
|---|---:|---|---|
| TX_ZED | → | PA3 / USART2_RX | F9P UBX导航及原始观测输出 |
| RX_ZED | ← | PA2 / USART2_TX | F9P配置和RTCM差分数据输入 |
| TP | → | PA0 / TIM2_CH1 | F9P 1PPS/TIMEPULSE硬件捕获 |
| GND | ↔ | GND | 与STM32、MPU6050和树莓派共地 |

STM32与F9P之间串口参数保持：

```text
115200 bit/s，8N1，无流控
```

C099的J4只短接`ARD`位置，即7-8脚（ARDUINO MODE）。不要同时短接
`UART1`或`UART3`位置，否则多个发送端可能同时驱动ZED-F9P RX。

C099当前由降压模块固定5V独立支路供电，连到C099的`DC_IN 5-12V`输入。
不得从STM32 GPIO取电，也不能把C099的`5V OUT`或`3.3V OUT`当作电源
输入。树莓派、STM32、MPU6050和C099的GND必须公地。

`TX_ZED`同时分支给STM32 PA3和树莓派GPIO5：

```text
C099 TX_ZED ──┬──→ STM32 PA3 / USART2_RX
              └──→ 树莓派 Pin 29 / GPIO5（当前直接连接，可选330Ω～1kΩ）
```

这是一个3.3 V发送端驱动两个高阻输入，不影响STM32接收。GPIO5只用于
旁路监听，不向F9P发送数据。加载`uart2-pi5`后，GPIO4/5分别成为UART2的
TXD2/RXD2，对应`/dev/ttyAMA2`；GPIO4无需接线。采集服务持续清空该接收口，
点击“开始采集”后才把原始字节写入当前会话的`f9p.ubx`。

## 5. 完整接线速查

```text
降压模块 VADJ 5.10～5.20V → 树莓派 Pin 2 / 5V
降压模块 VADJ 5.10～5.20V → 树莓派 Pin 4 / 5V
降压模块 GND              → 树莓派 Pin 6 / GND
降压模块 GND              → 树莓派 Pin 9 / GND
降压模块 固定5V           → STM32 5V
降压模块 固定5V           → C099 DC_IN 5-12V
降压模块 GND              ↔ STM32 GND、C099 GND
树莓派 Pin 8  / GPIO14 TX → STM32 PA10 / USART1_RX
树莓派 Pin 10 / GPIO15 RX ← STM32 PA9  / USART1_TX
树莓派 Pin 29 / GPIO5     ← C099 TX_ZED（当前直连，可选串330Ω～1kΩ）

MPU6050 VCC               → STM32 3.3V
MPU6050 GND               ↔ STM32 GND
MPU6050 SCL               → STM32 PB6
MPU6050 SDA               ↔ STM32 PB7
MPU6050 INT               → STM32 PA1 / TIM2_CH2

C099 TX_ZED               → STM32 PA3 / USART2_RX
C099 RX_ZED               ← STM32 PA2 / USART2_TX
C099 TP                   → STM32 PA0 / TIM2_CH1
C099 GND                  ↔ STM32 GND
C099 J4                   = 仅ARD 7-8短接
```

## 6. 上电前检查

1. 树莓派、STM32和C099全部断电后检查接线。
2. 树莓派断开时把VADJ调到5.10～5.20V；确认Pin 2/4同接VADJ，Pin 6/9同接GND。
3. 确认固定5V分别独立连接STM32 `5V`和C099 `DC_IN 5-12V`。
4. 确认PA9没有误接树莓派TX，PA10没有误接树莓派RX。
5. 确认GPIO5只连接`TX_ZED`分支，软件配置为输入，没有任何其他输出驱动该节点。
6. 确认所有设备GND共地，ST-Link供电线已断开。
7. 确认STM32 BOOT0=0，C099 J4只有ARD跳帽。
8. 上电后在树莓派Pin 2/4对Pin 6/9测带载电压；低于约5.0V先整改线路。
9. 测量STM32 `3.3V-GND`应约为3.3V，C099应正常启动且设备无异常发热。
10. 运行`vcgencmd get_throttled`，正常结果应为`throttled=0x0`。

## 7. ST-Link烧录接线

STM32开发板丝印`SWIO`就是芯片的`PA13/SWDIO`，不是另一种接口。当前系统由
降压模块固定5V给STM32供电，ST-Link只负责SWD调试信号：

| ST-Link | STM32F103C8T6板载接口 | 说明 |
|---|---|---|
| SWDIO | SWIO / PA13 | SWD双向数据 |
| SWCLK | SWCLK / PA14 | SWD时钟 |
| GND | GND | 必须共地 |
| RST / NRST（若有） | NRST | 推荐连接，便于硬件复位烧录 |
| 3.3V | 不连接 | 避免与STM32板载3.3V稳压输出并联 |
| 5V | 不连接 | STM32已经由降压模块固定5V供电 |

```text
ST-Link SWDIO ───→ STM32 SWIO
ST-Link SWCLK ───→ STM32 SWCLK
ST-Link GND   ───↔ STM32 GND
ST-Link NRST  ───→ STM32 NRST（可选但推荐）
ST-Link 3.3V/5V   × 不连接
```

接线前先停止采集并正常关闭树莓派，保持STM32的`BOOT0=0`。全部接好后重新给
树莓派供电，再把ST-Link插入电脑。若只有四针`3.3V/SWDIO/SWCLK/GND`接口，
只连接`SWDIO`、`SWCLK`和`GND`三根即可。

## 8. 树莓派UART检查

树莓派5需要关闭串口登录控制台并启用串口硬件：

```bash
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
grep -qxF 'dtoverlay=uart0-pi5' /boot/firmware/config.txt || \
  echo 'dtoverlay=uart0-pi5' | sudo tee -a /boot/firmware/config.txt
grep -qxF 'dtoverlay=uart2-pi5' /boot/firmware/config.txt || \
  echo 'dtoverlay=uart2-pi5' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

重启后检查：

```bash
ls -l /dev/serial* /dev/ttyAMA*
pinctrl get 4,5,14,15
```

树莓派5的`/dev/serial0 -> ttyAMA10`是专用调试UART，不连接40针GPIO14/15，
本项目不得使用它。加载`uart0-pi5`覆盖层后，GPIO14/15对应`/dev/ttyAMA0`，
并分别显示`TXD0/RXD0`；加载`uart2-pi5`后GPIO4/5对应`/dev/ttyAMA2`，分别显示
`TXD2/RXD2`。先用`/dev/ttyAMA0`以460800、8N1只读测试STM32输出，
确认能连续看到`IMU`、`GNSS`、`SAT`、`RAWX`和RTCM状态记录，再允许NTRIP
线程向同一串口写入。完整命令见[REMOTE_CONNECTION.md](REMOTE_CONNECTION.md)。

不要使用`cat`读取二进制UBX。需要在停止定位服务后验证GPIO5时，可运行：

```bash
sudo stty -F /dev/ttyAMA2 115200 raw -echo -ixon -ixoff
sudo timeout 3 dd if=/dev/ttyAMA2 bs=4096 status=none | od -An -tx1 -N64
```

正常输出中应反复出现UBX同步字节`b5 62`。测试结束后重新启动定位服务。

## 9. 正常工作判据

- IMU约100 Hz，序号连续，`time_valid=1`。
- GNSS导航解约1 Hz，GPS周和周内时间持续递增。
- 每秒出现F9P TIMEPULSE，STM32时间同步不中断。
- 启用NTRIP后，树莓派网络字节、STM32接收/转发字节持续增加。
- F9P从3D进入差分/RTK浮点或固定；基站断流时IMU/GNSS采集仍继续。
- 开始采集后，同一时间戳目录包含`imu.csv`、`gnss.csv`、`rawx.csv`和持续增长的
  `f9p.ubx`；终端状态中的`UBXrx/save`接收/保存字节数持续增加。

参考资料：

- [Raspberry Pi UART配置](https://www.raspberrypi.com/documentation/computers/configuration.html)
- [C099-F9P Application Board User Guide](https://content.u-blox.com/sites/default/files/documents/C099-F9P-AppBoard_UserGuide_UBX-18063024.pdf)
