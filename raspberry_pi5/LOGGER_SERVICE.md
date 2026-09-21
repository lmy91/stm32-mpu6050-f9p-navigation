# 树莓派实时定位与按需存储服务

服务名称：`gnss-imu-logger.service`

主串口使用树莓派5 GPIO14/15对应的`/dev/ttyAMA0`，参数为460800 bit/s、8N1、
无流控；F9P原始旁路使用GPIO5/RXD2对应的`/dev/ttyAMA2`，参数为115200 bit/s。
服务调用`tools/capture_serial.py`并在开机后持续读取两个串口、解码和发布实时位置，
但默认不创建采集文件。输入命令或点击网页“开始采集”后才创建：

```text
/home/lmy/stm32-mpu6050-f9p-navigation/data/decoded/YYYYMMDDHHMMSS/
├── imu.csv
├── gnss.csv
├── rawx.csv
└── f9p.ubx
```

## PC项目与树莓派运行文件的对应关系

树莓派当前实际运行的代码和配置已经同步保存在本项目中。修改、学习或备份时，
以PC项目内的文件为准：

| 树莓派上的运行文件 | PC项目中的源码副本 | 用途 |
|---|---|---|
| `/home/lmy/stm32-mpu6050-f9p-navigation/tools/capture_serial.py` | `tools/capture_serial.py` | 协议v3解码与三类CSV、原始UBX同步采集 |
| `/etc/systemd/system/gnss-imu-logger.service` | `raspberry_pi5/systemd/gnss-imu-logger.service` | 开机实时定位/解码服务，按需保存 |
| `/usr/local/bin/gnss-imu-status` | `raspberry_pi5/watch_logger_status.sh` | 终端单行刷新采集状态 |
| `/usr/local/bin/gnss-imu-record-start` | `raspberry_pi5/record_start.sh` | 开始一个新的四文件采集会话 |
| `/usr/local/bin/gnss-imu-record-stop` | `raspberry_pi5/record_stop.sh` | 刷新并停止保存，定位继续运行 |
| `/usr/local/bin/gnss-imu-base` | `raspberry_pi5/base_station_ctl.py` | 连接、重连、查询或断开NTRIP基站 |
| `/usr/local/bin/gnss-imu-clear-data` | `raspberry_pi5/clear_logger_data.sh` | 安全清理时间戳采集目录 |
| `/etc/systemd/system/gnss-imu-dashboard.service` | `raspberry_pi5/systemd/gnss-imu-dashboard.service` | 手机/PC实时网页地图服务 |
| `/etc/systemd/system/gnss-imu-time-sync.service` | `raspberry_pi5/systemd/gnss-imu-time-sync.service` | 开机从有效F9P时间一次性校正系统时钟 |

树莓派5的UART运行配置也已记录在本目录文档中：GPIO14/15使用
`/dev/ttyAMA0`，`/boot/firmware/config.txt`启用了`dtparam=uart0=on`、
`dtoverlay=uart0-pi5`和`dtoverlay=uart2-pi5`；GPIO5旁路对应`/dev/ttyAMA2`。
`/boot/firmware/cmdline.txt`不包含串口控制台参数。
完整配置过程见[README.md](README.md)和[REMOTE_CONNECTION.md](REMOTE_CONNECTION.md)。

可以分别在PC和树莓派上计算SHA-256，对照确认部署版本没有发生漂移：

```powershell
# PC PowerShell，在项目根目录运行
Get-FileHash -Algorithm SHA256 tools/capture_serial.py,
  raspberry_pi5/watch_logger_status.sh,
  raspberry_pi5/clear_logger_data.sh,
  raspberry_pi5/systemd/gnss-imu-logger.service
```

```bash
# 树莓派
sha256sum \
  /home/lmy/stm32-mpu6050-f9p-navigation/tools/capture_serial.py \
  /usr/local/bin/gnss-imu-status \
  /usr/local/bin/gnss-imu-record-start \
  /usr/local/bin/gnss-imu-record-stop \
  /usr/local/bin/gnss-imu-clear-data \
  /etc/systemd/system/gnss-imu-logger.service
```

常用命令：

```bash
# 查看定位/解码服务状态
sudo systemctl status gnss-imu-logger.service

# 实时查看运行日志
journalctl -u gnss-imu-logger.service -f

# 仅显示最新运行状态，并在同一行原位刷新
gnss-imu-status

# 开始保存：新建时间戳目录、三个CSV和f9p.ubx
gnss-imu-record-start

# 停止保存：刷新并关闭CSV，实时定位和网页继续运行
gnss-imu-record-stop

# 基站连接、状态、重连和断开
gnss-imu-base connect
gnss-imu-base status
gnss-imu-base reconnect
gnss-imu-base disconnect

# 维护定位服务（正常使用无需执行）
sudo systemctl stop gnss-imu-logger.service
sudo systemctl start gnss-imu-logger.service
sudo systemctl restart gnss-imu-logger.service

# 查看最新数据目录
ls -ltd /home/lmy/stm32-mpu6050-f9p-navigation/data/decoded/* | head

# 安全清理全部时间戳采集目录（会要求输入yes确认）
gnss-imu-clear-data
```

`gnss-imu-base connect`会隐藏密码输入；账号只保存在
`/run/gnss-imu/ntrip.json`，重启或断开后清除，不会进入采集目录和Git。

`gnss-imu-clear-data`只删除固定`data/decoded`目录下名称为14位时间戳的采集
会话。若服务正在运行，它会先安全停止定位服务，删除后自动恢复服务；恢复后
默认不保存，也不会自动新建会话。
如已明确确认且需要用于脚本，可执行`gnss-imu-clear-data --yes`跳过交互确认。

服务启动后默认每5秒更新一次状态，`REC=OFF/ON`分别表示仅实时定位和正在保存：

```text
[20:18:31] IMU=998 99.95Hz GNSS=10 RAWX=432 SAT=550 UBXrx/save=220160/128640B lost=0 invalid=0 REC=ON t=0.2min
```

`IMU`后的Hz数值是最近一个状态周期的实际频率；`UBXrx/save`依次是服务启动后
收到的F9P原始字节和当前会话已保存的字节。`lost`是IMU序号检测出的丢帧数，
`invalid`是无法按协议v3解析的输入行数。直接运行采集器时可用
`--status-interval`调整显示周期，设置为0可关闭周期状态输出。直接运行时状态
使用回车在同一行刷新；后台服务使用`gnss-imu-status`查看时也只刷新一行。
状态超过当前终端宽度时会自动截断，调整终端窗口宽度后下一次刷新自动适配，
不会因自动换行不断向下产生新行。

定位服务每秒将最新GNSS和采集状态原子更新到`/run/gnss-imu/live.json`，网页从
这个易失状态文件读取实时位置。开始保存后，GNSS文件每条1 Hz结果立即刷新；
IMU、RAWX CSV和`f9p.ubx`按批次刷新，避免高频采集产生不必要的磁盘同步开销。
停止采集时四个文件都会刷新并关闭。正常执行系统关机时systemd会向服务发送
SIGINT并刷新文件；不要直接拔掉树莓派电源。

不要同时用`cat`、Qt、另一个采集器或串口工具打开`/dev/ttyAMA0`或
`/dev/ttyAMA2`。需要临时查看原始输出时，先停止服务；查看结束后重新启动服务。

## 转换RINEX

`f9p.ubx`保存F9P UART1的原始UBX字节，包含`RXM-RAWX`观测量；烧录当前固件后还
包含`RXM-SFRBX`广播导航字。必须先停止当前采集，等待`f9p.ubx`关闭后再转换，
不要直接转换仍在增长的文件。

### Windows使用convbin.exe

先进入某次采集的时间戳目录，例如：

```powershell
Set-Location "C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260912180500"
```

当前电脑已确认程序位于`C:\Users\12597\Desktop\convbin.exe`，整行执行：

```powershell
& "C:\Users\12597\Desktop\convbin.exe" -r ubx -v 3.04 -f 5 -od -os -oi -ot -ol -o ".\rover.obs" -n ".\rover.nav" ".\f9p.ubx"
```

参数含义：

- `-r ubx`：输入是u-blox UBX二进制文件。
- `-v 3.04`：输出RINEX 3.04，支持多星座混合记录。
- `-f 5`：不按RTKLIB频率序号截掉观测类型；文件中只会写入F9P实际收到的信号。
- `-od -os`：在观测文件中保留多普勒和信噪比；伪距和载波相位默认保留。
- `-oi -ot -ol`：在导航文件头中写入已收到的电离层改正、时间系统改正和闰秒。
- `-o rover.obs`：生成RINEX观测文件（O文件）。
- `-n rover.nav`：生成RINEX 3混合导航文件（N文件）。

转换后检查文件是否存在且非空：

```powershell
Get-Item .\rover.obs, .\rover.nav | Select-Object Name, Length, LastWriteTime
Get-Content .\rover.obs -TotalCount 12
Get-Content .\rover.nav -TotalCount 12
```

`rover.obs`应包含`OBSERVATION DATA`及各星座观测类型；`rover.nav`首部应为
`NAVIGATION DATA`，多星座文件通常显示`M: MIXED`。某个星座能否写入取决于本次
采集是否实际收到该星座的`RXM-RAWX`和`RXM-SFRBX`，不能由`convbin.exe`补出
未接收到的导航电文。

### 树莓派安装convbin后的等价命令

```bash
convbin -r ubx -v 3.04 -f 5 -od -os -oi -ot -ol \
  -o rover.obs -n rover.nav f9p.ubx
```

开始采集后的前几分钟，广播导航电文可能尚未收齐。为了得到尽量完整的混合导航
文件，建议在开阔环境连续记录至少30分钟，并在结束记录后再执行转换。RTKLIB
`convbin`的参数说明可参考
[RTKLIB Explorer CONVBIN说明](https://deepwiki.com/rtklibexplorer/RTKLIB/3.4-convbin)。不同构建版
的参数可能有差异，本节命令已经用桌面这份`convbin.exe -h`核对；实际程序帮助信息
优先于网页说明。
